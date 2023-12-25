# 参与者和蓝图

Carla 中的参与者是在模拟中执行动作的元素，他们可以影响其他参与者。Carla 的参与者包括车辆和步行者，也包括传感器、交通标志、红绿灯和观看者。对如何操作它们有充分的了解是至关重要的。 

本节将介绍生成、摧毁、类型以及如何管理它们。然而，可能性几乎是无穷无尽的。实验、查看本文档中的 __教程__，并在 [Carla 论坛](https://github.com/carla-simulator/carla/discussions/) 中分享疑虑和想法。

- [__蓝图__](#blueprints)  
	- [管理蓝图库](#managing-the-blueprint-library)  
- [__参与者生命周期__](#actor-life-cycle)  
	- [生成](#spawning)  
	- [处理](#handling)  
	- [摧毁](#destruction)  
- [__参与者类型__](#types-of-actors)  
	- [传感器](#sensors)  
	- [观察者](#spectator)  
	- [交通标志和交通灯](#traffic-signs-and-traffic-lights)  
	- [车辆](#vehicles)  
	- [步行者](#walkers)  

---
## 蓝图

这些布局允许用户将新的参与者平滑地结合到模拟中。它们已经是带有动画和一系列属性的模型。其中一些是可修改的，而另一些则不是。这些属性包括车辆颜色、激光雷达传感器中的通道数量、步行者的速度等等。

[蓝图库](bp_library.md) 列出了可用的蓝图及其属性。车辆和步行者蓝图具有一个生成属性，用于指示它们是新的（第2代）资产还是旧的（第1代）资产。

### 管理蓝图库

[carla.BlueprintLibrary](python_api.md#carla.BlueprintLibrary) 类包含一系列 [carla.ActorBlueprint](python_api.md#carla.ActorBlueprint) 元素。它是能够提供对它访问的世界对象。
```py
blueprint_library = world.get_blueprint_library()
```
蓝图有一个 ID 来识别它们和由此产生的参与者。可以读取库来找到某个 ID，随机选择蓝图，或者使用 [通配符模式](https://tldp.org/LDP/GNU-Linux-Tools-Summary/html/x11655.htm) 过滤结果。

```py
# 找一个指定的蓝图
collision_sensor_bp = blueprint_library.find('sensor.other.collision')
# 随机选择一个车辆蓝图
vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
```

除此之外，每个 [carla.ActorBlueprint](python_api.md#carla.ActorBlueprint) 能获取 _get_ 和改变 _set_ 一些列参与者属性 [carla.ActorAttribute](python_api.md#carla.ActorAttribute) 。
```py
is_bike = [vehicle.get_attribute('number_of_wheels') == 2]
if(is_bike)
    vehicle.set_attribute('color', '255,0,0')
```
!!! 注意
    某些属性无法修改。在[蓝图库](bp_library.md)中查看。

属性有一个参与者属性类型 [carla.ActorAttributeType](python_api.md#carla.ActorAttributeType) 变量。它从枚举列表中声明其类型。此外，可修改的属性还附带一个 __推荐值列表__ 。

```py
for attr in blueprint:
    if attr.is_modifiable:
        blueprint.set_attribute(attr.id, random.choice(attr.recommended_values))
```
!!! 注意
    用户可以创建他们自己的车辆。查看 __教程（资产）__ 以了解相关信息。贡献者可以 [将他们的新内容添加到 Carla](tuto_D_contribute_assets.md) 。

---
## 参与者生命周期  

!!! 重要
    本节提到了关于参与者的不同方法。Python API 提供 __[commands](python_api.md#command.SpawnActor)__ ，以便在一个框架中应用一批最常见的命令。 

### 生成

__世界对象负责生成参与者并跟踪这些参与者。__ 生成只需要一张蓝图和一个 [carla.Transform](python_api.md#carla.Transform) 说明参与者的位置和旋转。 

世界有两个不同的方法生成参与者。 

* [`spawn_actor()`](python_api.md#carla.World.spawn_actor) 如果生成失败抛出一个异常。
* [`try_spawn_actor()`](python_api.md#carla.World.try_spawn_actor) 如果生成失败返回 `None`。

```py
transform = Transform(Location(x=230, y=195, z=40), Rotation(yaw=180))
actor = world.spawn_actor(blueprint, transform)
```

!!! 重要
    Carla 使用 [虚幻引擎坐标系统](https://carla.readthedocs.io/en/latest/python_api/#carlarotation) 。记住： [`carla.Rotation`](https://carla.readthedocs.io/en/latest/python_api/#carlarotation) 构造函数定义为“俯仰、偏航、翻滚” `(pitch, yaw, roll)`，与虚幻编辑器的“翻滚、俯仰、偏航” `(roll, pitch, yaw)` 不同。

如果在指定位置发生冲突，将不会生成参与者。无论这种情况发生在静态对象还是其他参与者身上。可以尝试避免这些不希望的生成碰撞。

* `map.get_spawn_points()` __对于车辆__。 返回推荐生成点的列表。

```py
spawn_points = world.get_map().get_spawn_points()
```

* `world.get_random_location()` __对于行人__。 返回人行道上的随机点。同样的方法也用于为行人设置目标位置。  

```py
spawn_point = carla.Transform()
spawn_point.location = world.get_random_location_from_navigation()
```

一个参与者在生成时可以附加到另一个参与者上。参与者跟随他们所依附的父级。这对于传感器特别有用。该附件可以是刚性的（适合检索精确数据），也可以根据其父级的情况进行轻松移动。它由辅助类 [carla.AttachmentType](python_api.md#carla.AttachmentType) 定义。

下一个示例将相机刚性地连接到车辆上，因此它们的相对位置保持固定。

```py
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle, carla.AttachmentType.Rigid)
```
!!! 重要
    当生成附加的参与者时，提供的变换必须相对于父级参与者。

一旦生成，世界对象就会将参与者添加到列表中。这可以很容易地搜索或迭代。 
```py
actor_list = world.get_actors()
# 根据 id 找参与者
actor = actor_list.find(id)
# 打印世界上所有限速标志的位置。
for speed_sign in actor_list.filter('traffic.speed_limit.*'):
    print(speed_sign.get_location())
```

### 处理

[carla.Actor](python_api.md#carla.Actor) 主要由  _get()_ 和 _set()_ 方法组成，用于管理地图周围的参与者。

```py
print(actor.get_acceleration())
print(actor.get_velocity())

location = actor.get_location()
location.z += 10.0
actor.set_location(location)
```

可以禁用演员的物理特性以将其冻结在适当的位置。

```py
actor.set_simulate_physics(False)
```
除此之外，参与者还有他们的蓝图提供的标签。这些对于语义分割传感器最有用。

!!! 警告
    大多数方法是异步向模拟器发送请求。模拟器每次更新解析它们的时间有限。使用 _set()_ 方法淹没模拟器将积累明显的延迟。


### 摧毁

当 Python 脚本完成时，参与者不会被销毁。他们必须明确地摧毁自己。

```py
destroyed_sucessfully = actor.destroy() # 如果成功返回 True 
```

!!! 重要
    销毁参与者会阻塞模拟器，直到该过程完成。

---
## 参与者类型  
### 传感器

传感器是产生数据流的参与者。他们有自己的部分，[第四部分. 传感器和数据](core_sensors.md)。现在，我们只看一下常见的传感器生成周期。

此示例生成一个相机传感器，将其连接到车辆，并告诉相机将生成的图像保存到磁盘。

```py
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
```
* 传感器也有蓝图。设置属性至关重要。 
* 大多数传感器将安装在车辆上以收集周围环境的信息。
* 传感器 __监听__ 数据。收到数据后，它们会调用用 __[Lambda 表达式](https://docs.python.org/3/reference/expressions.html)__ 描述的函数<small>(提供的链接中的 6.14)</small>。 

### 观察者

由虚幻引擎放置以提供游戏内的视角。它可用于移动模拟器窗口的视角。以下示例将移动观众演员，将视角指向所需的车辆。

```py
spectator = world.get_spectator()
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
carla.Rotation(pitch=-90)))

```

### 交通标志和交通灯

到目前为止，Carela 中只有停靠点、让行路线和交通信号灯被视为参与者。其余的 OpenDRIVE 标志可通过 API 作为地标（[__carla.Landmark__](python_api.md#carla.Landmark)）进行访问。他们的信息可以使用这些实例访问，但他们在模拟中并不作为参与者存在。在接下来的 __第 3 部分 地图和导航__ 中将更详细地解释地标。 


当模拟开始、停止、让行和交通灯时，会使用 OpenDRIVE 文件中的信息自动生成。__这些都无法在蓝图库中找到__，因此无法生成。

!!! 注意
    OpenDRIVE 文件中的 Carela 地图没有交通标志或信号灯。这些是由开发人员手动放置的。 

[__交通标志__](python_api.md#carla.TrafficSign) 未在路线图本身中定义，如下页所述。相反，他们有一个 [carla.BoundingBox](python_api.md#carla.BoundingBox) 来影响其中的车辆。 
```py
# 获得影响车辆的交通灯
if vehicle_actor.is_at_traffic_light():
    traffic_light = vehicle_actor.get_traffic_light()
``` 
路口设有[__交通灯__](python_api.md#carla.TrafficLight)。与任何参与者一样，他们有自己独特的 ID，但对于岔路口也有一个组（`group`） ID 。

同一组中的交通灯遵循一个循环。第一个设置为绿色，而其余的则保持红色。活跃的灯会持续几秒钟呈绿色、黄色和红色，因此有一段时间所有灯都是红色的。然后，下一个红绿灯开始循环，前一个红绿灯与其余红绿灯一起冻结。

可以使用 API 设置交通灯的状态。每个状态花费的秒数也是如此。可能的状态用 [carla.TrafficLightState](python_api.md#carla.TrafficLightState) 描述为一系列枚举值。
```py
#Change a red traffic light to green
if traffic_light.get_state() == carla.TrafficLightState.Red:
    traffic_light.set_state(carla.TrafficLightState.Green)
    traffic_light.set_set_green_time(4.0)
``` 

!!! Note
    Vehicles will only be aware of a traffic light if the light is red.  

### Vehicles

[__carla.Vehicle__](python_api.md#carla.Vehicle) is a special type of actor. It incorporates special internal components that simulate the physics of wheeled vehicles. This is achieved by applying four types of different controls:  

* __[carla.VehicleControl](python_api.md#carla.VehicleControl)__ provides input for driving commands such as throttle, steering, brake, etc. 
```py
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
```
* __[carla.VehiclePhysicsControl](python_api.md#carla.VehiclePhysicsControl)__ defines physical attributes of the vehicle and contains two more controllers:

    * [carla.GearPhysicsControl](python_api.md#carla.GearPhysicsControl) which controls the gears. 
    * [carla.WheelPhysicsControl](python_api.md#carla.WheelPhysicsControl) which provides specific control over each wheel.  

```py
    vehicle.apply_physics_control(carla.VehiclePhysicsControl(max_rpm = 5000.0, center_of_mass = carla.Vector3D(0.0, 0.0, 0.0), torque_curve=[[0,400],[5000,400]]))
```

Vehicles have a [carla.BoundingBox](python_api.md#carla.BoundingBox) encapsulating them. This bounding box allows physics to be applied to the vehicle and enables collisions to be detected.  

```py
    box = vehicle.bounding_box
    print(box.location)         # Location relative to the vehicle.
    print(box.extent)           # XYZ half-box extents in meters.
```

The physics of vehicle wheels can be improved by enabling the [sweep wheel collision parameter][enable_sweep]. The default wheel physics uses single ray casting from the axis to the floor for each wheel but when sweep wheel collision is enabled, the full volume of the wheel is checked against collisions. It can be enabled as such:

```py
    physics_control = vehicle.get_physics_control()
    physics_control.use_sweep_wheel_collision = True
    vehicle.apply_physics_control(physics_control)
```

[enable_sweep]: https://carla.readthedocs.io/en/latest/python_api/#carla.VehiclePhysicsControl.use_sweep_wheel_collision


Vehicles include other functionalities unique to them:

* __Autopilot mode__ will subscribe a vehicle to the [Traffic Manager](adv_traffic_manager.md) to simulate real urban conditions. This module is hard-coded, not based on machine learning.  

```py
    vehicle.set_autopilot(True)
```
* __Vehicle lights__ have to be turned on and off by the user. Each vehicle has a set of lights listed in [__carla.VehicleLightState__](python_api.md#carla.VehicleLightState). Not all vehicles have lights integrated. At the time of writing, vehicles with integrated lights are as follows:  
	*   __Bikes:__ All bikes have a front and back position light.  
	*   __Motorcycles:__ Yamaha and Harley Davidson models.  
	*   __Cars:__ Audi TT, Chevrolet Impala, both Dodge police cars, Dodge Charger, Audi e-tron, Lincoln 2017 and 2020, Mustang, Tesla Model 3, Tesla Cybertruck, Volkswagen T2 and the Mercedes C-Class.  

The lights of a vehicle can be retrieved and updated anytime using the methods [carla.Vehicle.get_light_state](python_api.md#carla.Vehicle.get_light_state) and [carla.Vehicle.set_light_state](#python_api.md#carla.Vehicle.set_light_state). These use binary operations to customize the light setting.  

```py
# Turn on position lights
current_lights = carla.VehicleLightState.NONE
current_lights |= carla.VehicleLightState.Position
vehicle.set_light_state(current_lights)
```

### Walkers

[__carla.Walker__](python_api.md#carla.Walker) work in a similar way as vehicles do. Control over them is provided by controllers.  

* [__carla.WalkerControl__](python_api.md#carla.WalkerControl) moves the pedestrian around with a certain direction and speed. It also allows them to jump. 
* [__carla.WalkerBoneControl__](python_api.md#carla.WalkerBoneControl) provides control over the 3D skeleton. [This tutorial](tuto_G_control_walker_skeletons.md) explains how to control it. 

Walkers can be AI controlled. They do not have an autopilot mode. The [__carla.WalkerAIController__](python_api.md#carla.WalkerAIController) actor moves around the actor it is attached to.  

```py
walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
world.SpawnActor(walker_controller_bp, carla.Transform(), parent_walker)
```
!!! Note
    The AI controller is bodiless and has no physics. It will not appear on scene. Also, location `(0,0,0)` relative to its parent will not cause a collision.  


__Each AI controller needs initialization, a goal and, optionally, a speed__. Stopping the controller works in the same manner. 

```py
ai_controller.start()
ai_controller.go_to_location(world.get_random_location_from_navigation())
ai_controller.set_max_speed(1 + random.random())  # Between 1 and 2 m/s (default is 1.4 m/s).
...
ai_controller.stop()
```
When a walker reaches the target location, they will automatically walk to another random point. If the target point is not reachable, walkers will go to the closest point from their current location.

A snipet in [carla.Client](python_api.md#carla.Client.apply_batch_sync) uses batches to spawn a lot of walkers and make them wander around.

!!! Important
    __To destroy AI pedestrians__, stop the AI controller and destroy both, the actor, and the controller. 

---
That is a wrap as regarding actors in CARLA. The next step takes a closer look into the map, roads and traffic in CARLA.  

Keep reading to learn more or visit the forum to post any doubts or suggestions that have come to mind during this reading. 
<div text-align: center>
<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="CARLA forum">
CARLA forum</a>
</p>
</div>
<div class="build-buttons">
<p>
<a href="../core_map" target="_blank" class="btn btn-neutral" title="3rd. Maps and navigation">
3rd. Maps and navigation</a>
</p>
</div>
</div>
