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

An actor can be attached to another one when spawned. Actors follow the parent they are attached to. This is specially useful for sensors. The attachment can be rigid (proper to retrieve precise data) or with an eased movement according to its parent. It is defined by the helper class [carla.AttachmentType](python_api.md#carla.AttachmentType).  

The next example attaches a camera rigidly to a vehicle, so their relative position remains fixed. 

```py
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle, carla.AttachmentType.Rigid)
```
!!! Important
    When spawning attached actors, the transform provided must be relative to the parent actor. 

Once spawned, the world object adds the actors to a list. This can be easily searched or iterated on. 
```py
actor_list = world.get_actors()
# Find an actor by id.
actor = actor_list.find(id)
# Print the location of all the speed limit signs in the world.
for speed_sign in actor_list.filter('traffic.speed_limit.*'):
    print(speed_sign.get_location())
```

### Handling

[carla.Actor](python_api.md#carla.Actor) mostly consists of _get()_ and _set()_ methods to manage the actors around the map. 

```py
print(actor.get_acceleration())
print(actor.get_velocity())

location = actor.get_location()
location.z += 10.0
actor.set_location(location)
```

The actor's physics can be disabled to freeze it in place. 

```py
actor.set_simulate_physics(False)
```
Besides that, actors also have tags provided by their blueprints. These are mostly useful for semantic segmentation sensors. 

!!! Warning
    Most of the methods send requests to the simulator asynchronously. The simulator has a limited amount of time each update to parse them. Flooding the simulator with _set()_ methods will accumulate a significant lag.


### Destruction

Actors are not destroyed when a Python script finishes. They have to explicitly destroy themselves.  

```py
destroyed_sucessfully = actor.destroy() # Returns True if successful
```

!!! Important
    Destroying an actor blocks the simulator until the process finishes. 

---
## Types of actors  
### Sensors

Sensors are actors that produce a stream of data. They have their own section, [4th. Sensors and data](core_sensors.md). For now, let's just take a look at a common sensor spawning cycle.  

This example spawns a camera sensor, attaches it to a vehicle, and tells the camera to save the images generated to disk.  

```py
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=my_vehicle)
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))
```
* Sensors have blueprints too. Setting attributes is crucial.  
* Most of the sensors will be attached to a vehicle to gather information on its surroundings. 
* Sensors __listen__ to data. When data is received, they call a function described with a __[Lambda expression](https://docs.python.org/3/reference/expressions.html)__ <small>(6.14 in the link provided)</small>. 

### Spectator

Placed by Unreal Engine to provide an in-game point of view. It can be used to move the view of the simulator window. The following example would move the spectator actor, to point the view towards a desired vehicle. 

```py
spectator = world.get_spectator()
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
carla.Rotation(pitch=-90)))

```

### Traffic signs and traffic lights

Only stops, yields and traffic lights are considered actors in CARLA so far. The rest of the OpenDRIVE signs are accessible from the API as [__carla.Landmark__](python_api.md#carla.Landmark). Their information is accessible using these instances, but they do no exist in the simulation as actors. Landmarks are explained more in detail in the following step, __3rd. Maps and navigation__.  

When the simulation starts, stop, yields and traffic light are automatically generated using the information in the OpenDRIVE file. __None of these can be found in the blueprint library__ and thus, cannot be spawned. 

!!! Note
    CARLA maps do not have traffic signs nor lights in the OpenDRIVE file. These are manually placed by developers.  

[__Traffic signs__](python_api.md#carla.TrafficSign) are not defined in the road map itself, as explained in the following page. Instead, they have a [carla.BoundingBox](python_api.md#carla.BoundingBox) to affect vehicles inside of it.  
```py
#Get the traffic light affecting a vehicle
if vehicle_actor.is_at_traffic_light():
    traffic_light = vehicle_actor.get_traffic_light()
``` 
[__Traffic lights__](python_api.md#carla.TrafficLight) are found in junctions. They have their unique ID, as any actor, but also a `group` ID for the junction. To identify the traffic lights in the same group, a `pole` ID is used.  

The traffic lights in the same group follow a cycle. The first one is set to green while the rest remain frozen in red. The active one spends a few seconds in green, yellow and red, so there is a period of time where all the lights are red. Then, the next traffic light starts its cycle, and the previous one is frozen with the rest.  

The state of a traffic light can be set using the API. So does the seconds spent on each state. Possible states are described with [carla.TrafficLightState](python_api.md#carla.TrafficLightState) as a series of enum values. 
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
