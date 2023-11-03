# CARLA 代理

CARLA 代理脚本允许车辆沿着随机的、无限的路线行驶，或者采用最短的路线到达给定的目的地。代理遵守交通信号灯并对道路上的其他障碍物做出反应。提供三种代理类型。可以修改目标速度、制动距离、尾随行为等参数。可以根据用户的需要修改 Actor 类或将其用作基类来创建自定义代理。

- [__代理脚本概述__](#overview-of-agent-scripts)
    - [计划与控制](#planning-and-control)
    - [代理行为](#agent-behaviors)
- [__实现一个代理__](#implement-an-agent)
- [__行为类型__](#behavior-types)
    - [创建自己的行为类型](#create-your-own-behavior-type)
- [__创建代理__](#creating-an-agent)

---

## 代理脚本概述

CARLA 代理中涉及的主要脚本位于`PythonAPI/carla/agents/navigation`中。它们分为两类； __计划和控制__ 和 __代理行为__。

### 计划与控制

- __`controller.py`:__ 将纵向和横向 PID 控制器组合成一个类，__VehiclePIDController__，用于从 CARLA 客户端对车辆进行低级控制。
- __`global_route_planner.py`:__ 从 CARLA 服务器获取详细的拓扑结构以构建世界地图的图形表示，为 __Local Planner__ 提供航点和道路选项信息。
- __`local_planner.py`:__ 根据来自 __VehiclePIDController__ 的控制输入跟踪航路点。航点可以由 __Global Route Planner__ 提供，也可以动态计算，在路口选择随机路径，类似于 [Traffic Manager](adv_traffic_manager.md)。

### 代理行为

- __`basic_agent.py`:__ 包含一个代理基类，它实现了一个 __Basic Agent__，它在地图上漫游或以尽可能短的距离到达目标目的地，避开其他车辆，响应交通信号灯但忽略停车标志。
- __`behavior_agent.py`:__ 包含一个实现更复杂的 __Behavior Agent__ 的类，它可以在尽可能短的距离内到达目标目的地，跟随交通信号灯、标志和速度限制，同时尾随其他车辆。有三种预定义的类型决定了代理的行为方式。
- __`behavior_types.py`:__ 包含影响 __Behavior Agent__ 的行为类型的参数；谨慎、正常和进取。

---

## 实现一个代理

本节将解释如何在您自己的脚本中使用示例 CARLA 代理类。在本节的最后，您将了解如何运行一个示例脚本来显示不同代理的运行情况。

__1.__ 导入要使用的代理类：

```py
# 导入基本代理
from agent.navigation.basic_agent import BasicAgent

# 导入行为代理
from agent.navigation.behavior_agent import BehaviorAgent
```

__2 .__ 任何车辆都可以变成代理人。 [生成车辆](core_actors.md#spawning) 并将其作为参数传递给代理类以实例化它：

```py
# 启动一个基本代理
agent = BasicAgent(vehicle)

# 启动具有攻击性配置文件的行为代理
agent = BehaviorAgent(vehicle, behavior='aggressive')
```

在 [__behavior types__](#behavior-types) 部分中阅读有关行为类型以及如何配置您自己的更多信息。

__3.__ 您可以设置代理前往的目的地。如果您不为代理设置目的地，它将在地图上无休止地漫游。要设置目的地，请为代理提供 [位置](python_api.md#carlalocation)：

```py
destination = random.choice(spawn_points).location
agent.set_destination(destination)
```

__5.__ 在导航步骤期间应用车辆控制和行为。在每个步骤中，__Basic Agent__ 将应用车辆控制并通过执行紧急停止来对任何车辆或交通信号灯做出反应。 __Behavior Agent__ 将根据您应用的行为类型对红绿灯做出反应、避开行人、跟随汽车并在十字路口导航：

```py
while True：
    vehicle.apply_control(agent.run_step())
```

__6.__ 您可以检查代理是否已完成其轨迹并在发生这种情况时执行操作。一旦您的车辆到达目的地，以下代码段将结束模拟：

```py
while True：
    if agent.done（）：
        print("The taerget has been reached, stopping the simulation")
        break

    vehicle.apply_control(agent.run_step())
```

__7.__ 不是在代理到达其目标目的地时完成模拟，而是可以生成一条新的随机路线供代理遵循：

```py
while True：
    if agent.done():   
        agent.set_destination(random.choice(spawn_points).location)
        print("The target has been reached, searching for another target")
    vehicle.apply_control(agent.run_step())
```

__Basic Agent__ 提供了一些方法来操纵代理行为或遵循的程序路线：

- __`set_target_speed(speed)`:__ 以公里/小时为单位设置目标速度
- __`follow_speed_limits(value=True)`:__ 设置代理遵循速度限制。
- __`set_destination(end_location, start_location=None)`:__ 代理将通过可能的最短路线从特定的起始位置到结束位置。如果没有提供起始位置，它将使用当前代理位置。
- __`set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)`:__ 为代理添加一个具体的计划。计划参数应包含一个`[carla.Waypoint, RoadOption]`列表，这将是代理需要采取的路径。 `stop_waypoint_creation` 将防止在路径运行后自动创建航点。 `clean_queue` 将重置代理的当前计划。
- __`trace_route(start_waypoint, end_waypoint)`:__ 从 Global Route Planner 获取两个航点之间的最短距离，并将路径作为 `[carla.Waypoint, RoadOption]` 列表返回，供代理遵循。
- __`ignore_traffic_lights(active=True)`:__ 设置代理忽略或服从交通信号灯。
- __`ignore_stop_signs(active=True)`:__ 设置代理忽略或服从停车标志。
- __`ignore_vehicles(active=True)`:__ 设置代理忽略或对其他车辆作出反应。

在 `PythonAPI/examples` 中找到的 `automatic_control.py` 脚本是基本和行为代理的一个示例。要尝试该脚本，请导航到示例目录并运行以下命令：

```sh
# 使用基本代理运行
python3 automatic_control.py --agent=Basic

# 使用行为代理运行
python3 automatic_control.py --agent=Behavior --behavior=aggressive
```

---

## 行为类型

行为代理的行为类型在 `behavior_types.py` 中定义。三个预配置的配置文件是 __'cautious'__、__'normal'__ 和 __'aggressive'__。您可以使用设置的配置文件、修改它们或创建您自己的配置文件。可以调整以下变量：

- __`max_speed`__：您的车辆能够达到的最高速度（以公里/小时为单位）。
- __`speed_lim_dist`__：以 km/h 为单位的值，用于定义车辆的目标速度与当前限速的距离（例如，如果限速为 30km/h 且 `speed_lim_dist` 为 10km/h，则目标速度将是20公里/小时）
- __`speed_decrease`__：当接近前方较慢的车辆时，您的车辆将以多快的公里/小时减速。
- __`safety_time`__：碰撞时间；如果您的车辆突然刹车，它与前面的车辆相撞所需的时间的近似值。
- __`min_proximity_threshold`__：在您的车辆执行避让或尾随等操作之前，与另一辆车或行人的最小距离（以米为单位）。
- __`braking_distance`__：您的车辆将执行紧急停车时与行人或车辆的距离。
- __`tailgate_counter`__：用于避免在最后一个后挡板后过快尾随的计数器。

## 创建自己的行为类型

要创建自己的行为类型：

__1.__ 在 `behavior_types.py` 中为您的行为类型创建类：

```py
class ProfileName(object)：
    # 完整的值定义
```

__2.__ 在 `behavior_agent.py` 脚本中定义和实例化你的行为类型：

```py
# 代理行为参数
if behavior == 'cautious'：
    self._behavior = Cautious()

elif behavior == 'normal'：
    self._behavior = Normal()

elif behavior == 'aggressive'：
    self._behavior = Aggressive()

elif behavior == '<type_name>'：
    self._behavior = <TypeName>()
```

---

## 创建代理

CARLA 代理只是用户可以运行的代理类型的示例。用户可以在 __Basic Agent__ 的基础上创建自己的代理。可能性是无止境。每个代理只需要两个元素，__初始化__和__运行步骤__。

在下面查找自定义代理的最小布局示例：

```py
import carla

from agents.navigation.basic_agent import BasicAgent

class CustomAgent(BasicAgent):
    def __init__(self, vehicle, target_speed=20, debug=False):
        """
        :param vehicle: 应用到本地规划器逻辑的actor
        :param target_speed: 车辆移动的速度（Km/h）
        """
        super().__init__(target_speed, 调试)

    def run_step(self, debug=False):
        """
        执行一步导航
        :return: carla.VehicleControl
        """
        # 在每个模拟步骤中采取的行动
        control = carla.VehicleControl()
        return control
```

查看 `basic_agent.py` 和 `behavior_agent.py` 脚本以探索它们的结构和功能，以获取有关如何创建自己的更多想法。

---

您可以探索提供的代理脚本、扩展它们或将它们用作创建自己的基准。如果您对代理有任何疑问，请随时在 [论坛](https://github.com/carla-simulator/carla/discussions/) 发帖。