# [交通管理器](https://carla.readthedocs.io/en/latest/adv_traffic_manager/#traffic-manager)

- [__什么是交通管理器？__](#what-is-the-traffic-manager)
	- [结构化设计](#structured-design)
	- [用户定制](#user-customization)
- [__架构__](#architecture)
	- [概述](#overview)
	- [代理的生命周期和状态管理](#alsm)
	- [车辆注册表](#vehicle-registry)
	- [仿真状态](#simulation-state)
	- [控制循环](#control-loop)
	- [内存地图](#in-memory-map)
	- [路径缓存和车辆轨迹](#pbvt)
	- [PID 控制器](#pid-controller)
	- [命令数组](#command-array)
	- [控制循环的阶段](#stages-of-the-control-loop)
- [__使用交通管理器__](#using-the-traffic-manager)
	- [车辆行为考虑因素](#vehicle-behavior-considerations)
	- [创建交通管理器](#creating-a-traffic-manager)
	- [配置自动驾驶行为](#configuring-autopilot-behavior)
	- [停止交通管理器](#stopping-a-traffic-manager)
- [__确定性模式__](#deterministic-mode)
- [__混合物理模式__](#hybrid-physics-mode)
- [__运行多个流量管理器__](#running-multiple-traffic-managers)
	- [流量管理器服务端和客户端](#traffic-manager-servers-and-clients)
	- [多客户端仿真](#multi-client-simulations)
	- [多交通管理器仿真](#multi-tm-simulations)
	- [多重仿真](#multi-simulation)
- [__同步模式__](#synchronous-mode)
- [__大地图中的交通管理器__](#traffic-manager-in-large-maps)

---
## 什么是交通管理器？

交通管理器 (Traffic Manager, TM) 是在仿真中以自动驾驶模式控制车辆的模块。其目标是在模拟中填充真实的城市交通状况。用户可以自定义一些行为，例如设置特定的学习环境。

### 结构化设计

交通管理器构建于 Carla 的客户端之上。执行流程分为多个阶段，每个阶段都有独立的操作和目标。这有利于相位相关功能和数据结构的开发，同时提高计算效率。每个阶段都在不同的线程上运行。与其他阶段的通信通过同步消息传递进行管理。信息朝一个方向流动。

### 用户定制

用户可以通过设置允许、强制或鼓励特定行为的参数来对流量进行一定程度的控制。用户可以根据自己的喜好改变流量行为，无论是在线还是离线。例如，可以允许汽车忽略速度限制或强制变道。在尝试仿真现实时，能够尝试各种行为是必不可少的。驾驶系统需要在特定和非典型情况下进行训练。

---
## 架构

### 概述

![Architecture](img/tm_2_architecture.png)

上图是交通管理器的内部架构示意图。每个组件的 C++ 代码可以在  `LibCarla/source/carla/trafficmanager` 中找到。以下各节详细解释了每个组件。逻辑概述如下：

__1. 存储并更新仿真的当前状态。__

- [代理的生命周期和状态管理](#alsm) (Agent Lifecycle & State Management, ALSM)  扫描世界，跟踪所有存在的车辆和行人，并清理不再存在的条目。所有数据均从服务器检索并经过多个[阶段](#stages-of-the-control-loop)。ALSM 是唯一调用服务器的组件。
- [车辆注册表](#vehicle-registry) 包含一系列处于自动驾驶状态的车辆（由交通管理器控制）以及一系列不处于自动驾驶状态（不受交通管理器控制控制）的行人和车辆。
- [仿真状态](#simulation-state) 是仿真中所有车辆和行人的位置、速度和附加信息的缓存存储。

__2. 计算每辆自动驾驶车辆的运动。__

交通管理器根据[仿真状态](#simulation-state)为[车辆注册表](#vehicle-registry)中的所有车辆生成可行的命令。每辆车的计算都是单独进行的。这些计算分为不同的[阶段](#stages-of-the-control-loop)。[控制循环](#control-loop)通过在阶段之间创建同步屏障来确保所有计算的一致性。在当前阶段的所有车辆计算完成之前，没有车辆进入下一阶段。每辆车都会经历以下阶段：

>__2.1 - [定位阶段](#stage-1-localization-stage)__

>路径是使用从[内存地图](#in-memory-map)中收集的附近路径点列表动态创建的，内存地图是仿真地图作为路径点网格的简化。路口的方向是随机选择的。每辆车的路径均由路径缓存和车辆轨迹(Path Buffers & Vehicle Tracking, [PBVT](#pbvt)) 组件存储和维护，以便在未来阶段轻松访问和修改。

>__2.2 - [碰撞阶段](#stage-2-collision-stage)__

>边界框延伸到每辆车的路径上，以识别和导航潜在的碰撞危险。

>__2.3 - [交通灯阶段](#stage-3-traffic-light-stage).__

>与碰撞阶段类似，会识别由于交通灯影响、停车标志和路口优先级而影响每辆车路径的潜在危险。

>__2.4 - [运动规划器阶段](#stage-4-motion-planner-stage).__

>车辆运动是根据定义的路径计算的。[PID 控制器](#pid-controller)确定如何到达目标路径点。然后将其转换为 Carla 命令以供下一步应用。

>__2.5 - [车灯阶段](#stage-5-vehicle-lights-stage).__

> 车灯根据环境因素（例如阳光和雾或雨的存在）和车辆行为（例如，如果车辆将在下一个路口左转/右转，则打开方向指示灯；如果制动，则打开刹车灯）。


__3. 在仿真中应用命令__

上一步生成的命令被收集到[命令数组](#command-array)中，批量发送到 Carla 服务器，在同一帧中应用。

以下部分将更详细地解释上述交通管理器逻辑中的每个组件和阶段。

### 代理的生命周期和状态管理

代理的生命周期和状态管理。它是交通管理器逻辑周期的第一步，提供仿真当前状态的上下文。

代理的生命周期和状态管理组件：

- 扫描世界以跟踪所有车辆和行人的位置和速度。如果启用物理功能，则通过[Vehicle.get_velocity()](python_api.md#carla.Vehicle)检索速度。否则，将使用位置随时间更新的历史记录来计算速度。
- 存储[仿真状态](#simulation-state)组件中每辆车和行人的位置、速度和附加信息（交通灯影响、边界框等）。
- 更新[车辆注册表](#vehicle-registry)中交通管理器控制的车辆列表。
- 更新[控制循环](#control-loop)和[路径缓存和车辆轨迹](#pbvt)组件中的条目以匹配车辆注册表。

__相关的 .cpp 文件：__ `ALSM.h`, `ALSM.cpp`.

### 车辆注册表

车辆注册表记录仿真中的所有车辆和行人。

车辆注册表：

- 从[代理的生命周期和状态管理](#alsm)传递来最新的车辆和行人列表。
- 将注册到交通管理器的车辆存储在单独的数组中，以便在[控制循环](#control-loop)期间进行迭代。

__相关的 .cpp 文件：__ `MotionPlannerStage.cpp`.

### 仿真状态

仿真状态存储仿真中所有车辆的信息，以便在后期阶段轻松访问和修改。

仿真状态：

- 从[代理的生命周期和状态管理](#alsm)接收数据，包括当前参与者位置、速度、交通灯影响、交通灯状态等。
- 将所有信息存储在缓存中，避免在[控制循环](#control-loop)期间对服务器的后续调用。

__相关的 .cpp 文件：__ `SimulationState.cpp`, `SimulationState.h`.

### Control loop

The control loop manages the calculations of the next command for all autopilot vehicles so they are performed synchronously. The control loop consists of five different [stages](#stages-of-the-control-loop); localization, collision, traffic light, motion planner and vehicle lights.

The control loop:

- Receives an array of TM-controlled vehicles from the [vehicle registry](#vehicle-registry).
- Performs calculations for each vehicle separately by looping over the array.
- Divides calculations into a series of [stages](#stages-of-the-control-loop).
- Creates synchronization barriers between stages to guarantee consistency. Calculations for all vehicles are finished before any of them move to the next stage, ensuring all vehicles are updated in the same frame.
- Coordinates the transition between [stages](#stages-of-the-control-loop) so all calculations are done in sync.
- Sends the [command array](#command-array) to the server when the last stages ([Motion Planner Stage](#stage-4-motion-planner-stage) and [Vehicle Lights Stage](#stage-5-vehicle-lights-stage)) finishes so there are no frame delays between the command calculations and the command application.

__Related .cpp files:__ `TrafficManagerLocal.cpp`.

### In-Memory Map

The In-Memory Map is a helper module contained within the [PBVT](#pbvt) and is used during the [Localization Stage](#stage-1-localization-stage).  

The In-Memory Map:

- Converts the map into a grid of discrete waypoints.
- Includes waypoints in a specific data structure with more information to connect waypoints and identify roads, junctions, etc.
- Identifies these structures with an ID used to locate vehicles in nearby areas quickly.

__Related .cpp files:__ `InMemoryMap.cpp` and `SimpleWaypoint.cpp`.

### PBVT

PBVT stands for __Path Buffer and Vehicle Tracking__. The PBVT is a data structure that contains the expected path for every vehicle and allows easy access to data during the [control loop](#control-loop).

The PBVT:

- Contains a map of deque objects with one entry per vehicle.
- Contains a set of waypoints for each vehicle describing its current location and near-future path.
- Contains the [In-Memory Map](#in-memory-map) used by the [Localization Stage](#stage-1-localization-stage) to relate every vehicle to the nearest waypoint and possible overlapping paths.

### PID controller

The PID controller is a helper module that performs calculations during the [Motion Planner Stage](#stage-4-motion-planner-stage).

The PID controller:

- Estimates the throttle, brake, and steering input needed to reach a target value using the information gathered by the [Motion Planner Stage](#stage-4-motion-planner-stage).
- Makes adjustments depending on the specific parameterization of the controller. Parameters can be modified if desired. Read more about [PID controllers](https://en.wikipedia.org/wiki/PID_controller) to learn how to make modifications.

__Related .cpp files:__ `PIDController.cpp`.
### Command Array

The Command Array represents the last step in the TM logic cycle. It receives commands for all the registered vehicles and applies them.

The Command Array:

- Receives a series of [carla.VehicleControl](python_api.md#carla.VehicleControl)'s from the [Motion Planner Stage](#stage-4-motion-planner-stage).
- Batches all commands to be applied during the same frame.
- Sends the batch to the CARLA server calling either __apply_batch()__ or __apply_batch_synch()__ in [carla.Client](../python_api/#carla.Client) depending on whether the simulation is running in asynchronous or synchronous mode, respectively.

__Related .cpp files:__ `TrafficManagerLocal.cpp`.

### Stages of the Control Loop

##### Stage 1- Localization Stage

The Localization Stage defines a near-future path for vehicles controlled by the TM.

The Localization Stage:

- Obtains the position and velocity of all vehicles from the [simulation state](#simulation-state).
- Uses the [In-Memory Map](#in-memory-map) to relate every vehicle with a list of waypoints that describes its current location and near-future path according to its trajectory. The faster the vehicle goes, the longer the list will be.
- Updates the path according to planning decisions such as lane changes, speed limit, distance to leading vehicle parameterization, etc.
- Stores the path for all vehicles in the [PBVT](#pbvt) module.
- Compares paths with each other to estimate possible collision situations. Results are passed to the Collision Stage.

__Related .cpp files:__ `LocalizationStage.cpp` and `LocalizationUtils.cpp`.

##### Stage 2- Collision Stage

The Collision Stage triggers collision hazards.

The Collision Stage:

- Receives from the [Localization Stage](#stage-1-localization-stage) a list of vehicle pairs whose paths could potentially overlap.
- Extends bounding boxes along the path ahead (geodesic boundaries) for each vehicle pair to check if they actually overlap and determine whether the risk of collision is real.
- Sends hazards for all possible collisions to the [Motion Planner Stage](#stage-4-motion-planner-stage) to modify the path accordingly.

__Related .cpp files:__ `CollisionStage.cpp`.

##### Stage 3- Traffic Light Stage

The Traffic Light stage triggers hazards due to traffic regulators such as traffic lights, stop signs, and priority at junctions.

The Traffic Light stage:

- Sets a traffic hazard if a vehicle is under the influence of a yellow or red traffic light or a stop sign.
- Extends a bounding box along a vehicle's path if it is in an unsignaled junction. Vehicles with overlapping paths follow a "First-In-First-Out" order to move. Wait times are set to a fixed value.

__Related .cpp files:__ `TrafficLightStage.cpp`.

##### Stage 4- Motion Planner Stage

The Motion Planner Stage generates the CARLA commands to be applied to vehicles.

The Motion Planner Stage:

- Gathers a vehicle's position and velocity ([simulation state](#simulation-state)), path ([PBVT](#pbvt)), and hazards ([Collision Stage](#stage-2-collision-stage) and [Traffic Light Stage](#stage-3-traffic-light-stage)).
- Makes high-level decisions about how a vehicle should move, for example, computing the brake needed to prevent a collision hazard. A [PID controller](#pid-controller) is used to estimate behaviors according to target values.
- Translates the desired movement to a [carla.VehicleControl](python_api.md#carla.VehicleControl) for application to the vehicle.
- Sends the resulting CARLA commands to the [Command Array](#command-array).

__Related .cpp files:__ `MotionPlannerStage.cpp`.


##### Stage 5- Vehicle Lights Stage

The Vehicle Lights Stage activates the lights based on the condition of the vehicle and the surrounding environment.
	
The Vehicle Lights Stage:

- Retrieves the planned waypoints for the vehicle, information about vehicle lights (eg. light state and the planned command to be applied) and the weather conditions.

- Determines the new state of the vehicle lights:
	- Turns on the blinkers if the vehicle is planning to turn left/right at the next junction.
	- Turns on the stop lights if the applied command is asking the vehicle to brake.
	- Turns on the low beams and the position lights from sunset to dawn, or under heavy rain.
	- Turns on the fog lights under heavy fog conditions.

- Update the vehicle lights state if it has changed.

__Related .cpp files:__ `VehicleLightStage.cpp`.

---
## Using the Traffic Manager

### Vehicle behavior considerations

The TM implements general behavior patterns that must be taken into consideration when you set vehicles to autopilot:

- __Vehicles are not goal-oriented,__ they follow a dynamically produced trajectory and choose a path randomly when approaching a junction. Their path is endless.
- __Vehicles' target speed is 70% of their current speed limit__ unless any other value is set.
- __Junction priority does not follow traffic regulations.__ The TM uses its own priority system at junctions. The resolution of this restriction is a work in progress. In the meantime, some issues may arise, for example, vehicles inside a roundabout yielding to a vehicle trying to get in.

TM behavior can be adjusted through the Python API. For specific methods, see the TM section of the Python API [documentation](../python_api/#carla.TrafficManager). Below is a general summary of what is possible through the API:

| Topic | Description |
| ----- | ----------- |
| **General:** | - Create a TM instance connected to a port. <br> - Retrieve the port where a TM is connected. |
| **Safety conditions:** | - Set a minimum distance between stopped vehicles (for a single vehicle or for all vehicles). This will affect the minimum moving distance. <br> - Set the desired speed as a percentage of the current speed limit (for a single vehicle or for all vehicles). <br> - Reset traffic lights. |
| **Collision  managing:** | - Enable/Disable collisions between a vehicle and a specific actor. <br> - Make a vehicle ignore all other vehicles. <br> - Make a vehicle ignore all walkers. <br> - Make a vehicle ignore all traffic lights. |
| **Lane  changes:** | - Force a lane change, ignoring possible collisions. <br> - Enable/Disable lane changes for a vehicle. |
| **Hybrid physics mode:** | - Enable/Disable hybrid physics mode. <br> - Change the radius in which physics is enabled. |




### Creating a Traffic Manager

!!! Note
	TM is designed to work in synchronous mode. Using TM in asynchronous mode can lead to unexpected and undesirable results. Read more in the section [__Synchronous mode__](#synchronous-mode).

A TM instance is created by a [`carla.Client`](python_api.md#carla.Client), passing the port to be used. The default port is `8000`.

To create a TM instance:

```python
tm = client.get_trafficmanager(port)
```

To enable autopilot for a set of vehicles, retrieve the port of the TM instance and set `set_autopilot` to `True`, passing the TM port at the same time. If no port is provided, it will try to connect to a TM in the default port (`8000`). If the TM does not exist, it will create one:

```python
tm_port = tm.get_port()
 for v in vehicles_list:
     v.set_autopilot(True,tm_port)
```
!!! Note 
    Creating or connecting to a TM in multi-client situations is different from the above example. Learn more in the section [__Running multiple Traffic Managers__](#running-multiple-traffic-managers).

The `generate_traffic.py` script in `/PythonAPI/examples` provides an example of how to create a TM instance using a port passed as a script argument and register every vehicle spawned to it by setting the autopilot to `True` in a batch:

```py
traffic_manager = client.get_trafficmanager(args.tm-port)
tm_port = traffic_manager.get_port()
...
batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True,tm_port)))
...
traffic_manager.global_percentage_speed_difference(30.0)
```

### Configuring autopilot behavior

The following example creates a TM instance and configures dangerous behavior for a specific vehicle so it will ignore all traffic lights, leave no safety distance from other vehicles, and drive 20% faster than the current speed limit:

```python
tm = client.get_trafficmanager(port)
tm_port = tm.get_port()
for v in my_vehicles:
  v.set_autopilot(True,tm_port)
danger_car = my_vehicles[0]
tm.ignore_lights_percentage(danger_car,100)
tm.distance_to_leading_vehicle(danger_car,0)
tm.vehicle_percentage_speed_difference(danger_car,-20)
``` 

The example below sets the same list of vehicles to autopilot but instead configures them with moderate driving behavior. The vehicles drive 80% slower than the current speed limit, leaving at least 5 meters between themselves and other vehicles, and never perform lane changes:

```python
tm = client.get_trafficmanager(port)
tm_port = tm.get_port()
for v in my_vehicles:
  v.set_autopilot(True,tm_port)
danger_car = my_vehicles[0]
tm.global_distance_to_leading_vehicle(5)
tm.global_percentage_speed_difference(80)
for v in my_vehicles: 
  tm.auto_lane_change(v,False)
``` 

#### Delegating the Traffic Manager to automatically update vehicle lights

By default, vehicle lights (brake, turn indicators, etc...) of the vehicles managed by the TM are never updated. It is possible to delegate the TM to update the vehicle lights of a given vehicle actor:

```python
tm = client.get_trafficmanager(port)
for actor in my_vehicles:
  tm.update_vehicle_lights(actor, True)
```

Vehicle lights management has to be specified on a per-vehicle basis, and there could be at any given time both vehicles with and without the automatic light management.



### Stopping a Traffic Manager

The TM is not an actor that needs to be destroyed; it will stop when the client that created it stops. This is automatically managed by the API, the user does not have to do anything. However, when shutting down a TM, the user must destroy the vehicles controlled by it, otherwise they will remain immobile on the map. The script `generate_traffic.py` does this automatically:

```py
client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
```

!!! Warning
    Shutting down a __TM-Server__ will shut down the __TM-Clients__ connecting to it. To learn the difference between a __TM-Server__ and a __TM-Client__, read about [__Running multiple Traffic Managers__](#running-multiple-traffic-managers).

---
## 确定性模式

在确定性模式下，交通管理器将在相同条件下产生相同的结果和行为。不要将确定性论误认为是记录器。虽然记录器允许您存储仿真日志以进行回放，但确定性可确保只要维持相同的条件，交通管理器在脚本的不同执行过程中始终具有相同的输出。

确定性模式 __仅在同步模式下__ 可用。在异步模式下，对仿真的控制较少，并且无法实现确定性。在开始之前，请阅读[同步模式](#synchronous-mode)部分的更多信息。

要启用确定性模式，请使用以下方法：

```py
my_tm.set_random_device_seed(seed_value)
```

`seed_value` 是一个将生成随机数的数字的 `int` 种子数。该值本身并不相关，但相同的值将始终导致相同的输出。具有相同条件、使用相同种子值的两次仿真将是确定性的。

为了保持多次仿真运行的确定性，__必须为每次仿真设置种子__。例如，每次[重新加载](python_api.md#carla.Client.reload_world)世界时，都必须重新设置种子：


```py
client.reload_world()
my_tm.set_random_device_seed(seed_value)
```

可以通过将种子值作为参数传递来在示例脚本`generate_traffic.py`中测试确定性模式。以下示例在同步模式下使用 50 个自动驾驶参与者填充地图，并将种子设置为任意值`9`：

```sh
cd PythonAPI/examples
python3 generate_traffic.py -n 50 --seed 9
```

!!! 警告
    在启用确定性模式之前，Carla 服务器和交通管理器必须处于同步模式。在此处阅读有关交通管理器中同步模式的[更多信息](#synchronous-mode)。

---
## 混合物理模式

混合模式允许用户禁用所有自动驾驶车辆或标记为`英雄`的车辆特定半径之外的自动驾驶车辆的大多数物理计算。这消除了仿真中的车辆物理瓶颈。物理功能被禁用的车辆将通过隐形传送移动。维持线性加速度的基本计算，以确保位置更新和车辆速度保持真实，并且车辆上物理计算的切换是流畅的。

混合模式使用 [`Actor.set_simulate_physics()`](https://carla.readthedocs.io/en/latest/python_api/#carla.Actor.set_simulate_physics) 方法来切换物理计算。默认情况下它是禁用的。有两个选项可以启用它：

*   [__`TrafficManager.set_hybrid_physics_mode(True)`__](https://carla.readthedocs.io/en/latest/python_api/#carla.TrafficManager.set_hybrid_physics_mode) — 此方法为调用它的交通管理器对象启用混合模式。
*   __以`--hybrid`标志运行 `generate_traffic.py`__ — 此示例脚本创建一个交通管理器并在自动驾驶仪中生成车辆。当标志`--hybrid`作为脚本参数传递时，它将这些车辆设置为混合模式。

要修改混合模式的行为，请使用以下两个参数：

*   __Radius__ *(默认 = 50 米)* — 半径相对于标记有 `英雄` 的车辆。该半径内的所有车辆都将启用物理功能；半径之外的车辆将禁用物理功能。使用 [`traffic_manager.set_hybrid_physics_radius(r)`](python_api.md#carla.TrafficManager.set_hybrid_physics_radius) 修改半径的大小。
*   __Hero vehicle__ — 带有标记 `role_name='hero'` 的车辆作为半径的中心。
	*   __如果没有英雄车辆，__ 所有车辆的物理功能将被禁用。
	*   __如果有不止一辆英雄车辆，__ 则会考虑所有英雄车辆的半径，从而在启用物理功能的情况下创建不同的影响区域。

下面的剪辑显示了混合模式处于活动状态时如何启用和禁用物理功能。__英雄车辆__ 标有 __红色方块__。__禁用物理功能__ 的车辆标有 __蓝色方块__。当在英雄车辆的影响半径内时，__启用物理功能__ 并且标签变为 __绿色__。

![Welcome to CARLA](img/tm_hybrid.gif)


---
## 运行多个交通管理器

### 交通管理器的服务端和客户端

Carla 客户端通过向服务器指定要使用的端口来创建交通管理器。如果未指定端口，将使用默认 `8000` 端口。如果在同一端口上创建更多交通管理器，它们将成为 __交通管理器的客户端__，而原始交通管理器将成为 __交通管理器的服务端__。这些标题定义了交通管理器在仿真中的行为方式。

###### 交通管理器的服务端

如果交通管理器的服务端是第一个连接到空闲端口的交通管理器，然后其他交通管理器（交通管理器的客户端）连接到它正在运行的同一端口，则创建交通管理器服务端。__交通管理器服务端将规定所有交通管理器客户端的行为__，例如，如果 __交通管理器的服务端__ 停止，则所有 __交通管理器客户端__ 将停止。

以下代码创建两个交通管理器服务端。每个端口都连接到不同且都未使用的端口：

```py 
tm01 = client01.get_trafficmanager() # tm01 --> tm01 (p=8000)
```
```py
tm02 = client02.get_trafficmanager(5000) # tm02(p=5000) --> tm02 (p=5000)
```

###### 交通管理器的客户端

当一个交通管理器连接到另一个交通管理器（交通管理器服务端）占用的端口时，就会创建交通管理器客户端。交通管理器客户端的行为将由交通管理器服务端决定。

以下代码创建两个交通管理器客户端，每一个都与上一节中创建的交通管理器服务端连接。

```py
tm03 = client03.get_trafficmanager() # tm03 --> tm01 (p=8000). 
```
```py
tm04 = client04.get_trafficmanager(5000) # tm04(p=5000) --> tm02 (p=5000)
```

Carla 服务器通过存储链接到它们的端口和客户端 IP（对用户隐藏）来保存所有交通管理器实例的寄存器。目前无法检查到目前为止已创建的交通管理器实例。尝试创建实例时始终会尝试连接，并且它将创建新的 __交通管理器服务端__ 或 __交通管理器客户端__ 。


### 多客户端仿真

在多客户端模拟中，在同一端口上创建多个TM。第一个 TM 将是 TM 服务器，其余的将是连接到它的 TM 客户端。TM-Server 将规定所有 TM 实例的行为：

```py
terminal 1: ./CarlaUE4.sh -carla-rpc-port=4000
terminal 2: python3 generate_traffic.py --port 4000 --tm-port 4050 # TM-Server
terminal 3: python3 generate_traffic.py --port 4000 --tm-port 4050 # TM-Client
```

### 多交通管理器仿真

在多交通管理器仿真中，在不同的端口上创建多个交通管理器实例。每个交通管理器实例都会控制自己的行为：

```py
terminal 1: ./CarlaUE4.sh -carla-rpc-port=4000
terminal 2: python3 generate_traffic.py --port 4000 --tm-port 4050 # TM-Server A
terminal 3: python3 generate_traffic.py --port 4000 --tm-port 4550 # TM-Server B
```

### 多重仿真

多重仿真是指多个 Carla 服务器同时运行。交通管理器需要连接到相关的 Carla 服务器端口。只要计算能力允许，交通管理器可以一次运行多个仿真，不会出现任何问题：

```py
terminal 1: ./CarlaUE4.sh -carla-rpc-port=4000 # simulation A 
terminal 2: ./CarlaUE4.sh -carla-rpc-port=5000 # simulation B
terminal 3: python3 generate_traffic.py --port 4000 --tm-port 4050 # TM-Server A connected to simulation A
terminal 4: python3 generate_traffic.py --port 5000 --tm-port 5050 # TM-Server B connected to simulation B
```

多重仿真的概念独立于交通管理器本身。上面的示例并行运行两个 Carla 仿真 A 和 B。在每个仿真中，都独立创建一个交通管理器服务端。仿真 A 可以运行多客户端交通管理程序，而仿真 B 则运行多交通管理器或根本不运行交通管理器。

上述设置最可能出现的问题是客户端尝试连接到未在所选仿真上运行的现有交通管理器。如果发生这种情况，将会出现错误消息，并且连接将被中止，以防止仿真之间的干扰。

---
## 同步模式


交通管理器设计为在同步模式下工作。Carla 服务器和交通管理器应设置为同步才能正常运行。__在异步模式下使用交通管理器可能会导致意外和不良结果__，但是，如果需要异步模式，则仿真应至少以 20-30 fps 运行。

下面的脚本演示了如何将服务器和交通管理器设置为同步模式：

```py
...

# Set the simulation to sync mode
init_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
# After that, set the TM to sync mode
my_tm.set_synchronous_mode(True)

...

# Tick the world in the same client
world.apply_settings(init_settings)
world.tick()
...

# Always disable sync mode before the script ends to prevent the server blocking whilst waiting for a tick
settings.synchronous_mode = False
my_tm.set_synchronous_mode(False)
```

示例脚本 `generate_traffic.py` 启动交通管理器并用车辆和行人填充地图。它自动将交通管理器和 Carla 服务器设置为同步模式：

```sh
cd PythonAPI/examples
python3 generate_traffic.py -n 50
```

如果需要异步模式，请在运行上述命令时使用 `--async` 标志。

如果多个交通管理器设置为同步模式，同步将会失败。请遵循以下准则以避免出现问题：

- 在[多客户端](#multiclient)情况下，只有 __交通管理器服务端__ 应设置为同步模式。
- 在[多交通管理器](#multitm)情况下，只需将一台 __交通管理器服务端__ 设置为同步模式。
- [ScenarioRunner 模块](https://carla-scenariorunner.readthedocs.io/en/latest/)自动运行交通管理器。ScenarioRunner 内的交通管理器将自动设置为同步模式。

!!! 警告
    在管理时钟的脚本完成之前禁用同步模式（对于世界和交通管理器），以防止服务器阻塞，永远等待时钟。

---

## 大地图中的交通管理器

要了解交通管理器如何在大型地图上工作，请务必首先阅读[此处](large_map_overview.md)的文档来熟悉大型地图的工作原理。

自动驾驶车辆在大地图中的行为取决于是否存在英雄车辆：


__不存在英雄车辆__

所有自动驾驶车辆都将被视为休眠参与者。休眠的自动驾驶参与者将像混合模式一样在地图上移动。由于没有英雄车辆来触发地图瓦片流，因此不会渲染车辆。


__呈现英雄车辆__

当自动驾驶车辆超过 `actor_active_distance` 定义的值时，将进入休眠状态actor_active_distance。要设置此值，请使用 Python API：

```py
settings = world.get_settings()

# Actors will become dormant 2km away from the ego vehicle
settings.actor_active_distance = 2000

world.apply_settings(settings)
```

在交通管理器中，休眠的参与者可以配置为在英雄车辆周围不断重生，而不是在地图的其他部分保持休眠状态。可以使用`set_respawn_dormant_vehicles` Python API 中的方法配置此选项。车辆将在英雄车辆的用户定义距离内重生。可重生距离的上下边界可以使用`set_boundaries_respawn_dormant_vehicles`方法设置。注意，上距离不会大于大地图的瓦片流距离，距离最小为 20m。

要使英雄车辆 25 米和 700 米范围内的休眠车辆重生：

```py
my_tm.set_respawn_dormant_vehicles(True)
my_tm.set_boundaries_respawn_dormant_vehicles(25,700)
```

如果碰撞阻止休眠的参与者重生，交通管理器将重试下一个仿真步骤。

如果休眠车辆没有重生，它们的行为将取决于是否启用混合模式。如果启用了混合模式，则休眠的参与者将在地图上传送。如果未启用混合模式，则不会计算休眠参与者的物理特性，并且它们将保持在原位置，直到不再休眠。

---

If you have any questions about the TM, then you can ask in the [forum](https://github.com/carla-simulator/carla/discussions).

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
CARLA forum</a>
</p>
</div>
