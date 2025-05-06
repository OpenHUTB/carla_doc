# [传感器和数据](https://carla.readthedocs.io/en/latest/core_sensors/) 

传感器是从周围环境中检索数据的参与者。它们对于为驾驶代理创造学习环境至关重要。

本页总结了开始处理传感器所需的一切。它介绍了可用的类型及其生命周期的分步指南。每个传感器的详细信息可以在 [传感器参考](ref_sensors.md) 中找到。

* [__传感器逐步介绍__](#sensors-step-by-step)  
	*   [设置](#setting)  
	*   [生成](#spawning)  
	*   [监听](#listening)  
	*   [数据](#data)  
* [__传感器类型__](#types-of-sensors)  
	*   [相机](#cameras)  
	*   [检测器](#detectors)  
	*   [其他](#other)  

---
## 传感器逐步介绍 <span id="sensors-step-by-step"></span>

[carla.Sensor](python_api.md#carla.Sensor) 类定义了一种特殊类型的参与者，能够测量和传输数据。

* __这个数据是什么？__ 根据传感器的类型，它变化很大。所有类型的数据都继承自通用的 [carla.SensorData](python_api.md#carla.SensorData)。 
* __他们什么时候检索数据？__ 在每个模拟步骤上或在注册某个事件时。取决于传感器的类型。
* __他们如何检索数据？__ 每个传感器都有一种接收和管理数据的 `listen()` 方法。

尽管存在差异，但所有传感器的使用方式都相似。

### 设置 <span id="setting"></span>

与其他参与者一样，找到蓝图并设置特定属性。这在处理传感器时至关重要。他们的属性将决定所获得的结果。这些在 [传感器参考](ref_sensors.md) 中有详细介绍。

以下示例设置仪表板高清摄像头。

```py
# 找到传感器蓝图。
blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# 修改蓝图属性来设置图像分辨率和视野
blueprint.set_attribute('image_size_x', '1920')
blueprint.set_attribute('image_size_y', '1080')
blueprint.set_attribute('fov', '110')
# 设置传感器捕获之间的时间（以秒为单位）
blueprint.set_attribute('sensor_tick', '1.0')
``` 

### 生成 <span id="spawning"></span>

`attachment_to` 和 `attachment_type` 至关重要。传感器应连接到父参与者（通常是车辆）上，以跟踪它并收集信息。附件类型将确定其关于所述车辆的位置如何更新。

* __刚性附件(Rigid attachment)。__ 动对于其父位置是严格的。这是从模拟中检索数据的合适附件。
* __弹簧臂附件(SpringArm attachment)。__ 运动变得轻松，几乎没有加速和减速。仅建议使用此附件来记录模拟视频。更新摄像机位置时，移动平滑并且避免“跳跃”。
* __弹簧臂幽灵附件(SpringArmGhost attachment)。__ 与前一个类似，但没有进行碰撞测试，因此相机或传感器可以穿过墙壁或其他几何形状。

```py
transform = carla.Transform(carla.Location(x=0.8, z=1.7))
sensor = world.spawn_actor(blueprint, transform, attach_to=my_vehicle)
```
!!! 重要
    当带有附件生成时，位置必须相对于父参与者。

### 监听 <span id="listening"></span>

每个传感器都有一个 [`listen()`](python_api.md#carla.Sensor.listen) 方法。每次传感器获取数据时都会调用此函数。

参数`callback`是一个 [lambda 函数](https://www.w3schools.com/python/python_lambda.asp) 。它描述了传感器在获取数据时应该做什么。这必须将检索的数据作为参数。

```py
# 每次相机生成新图像时，都会调用 do_something()。
sensor.listen(lambda data: do_something(data))

...

# 每次检测到碰撞时，此碰撞传感器都会打印。
def callback(event):
    for actor_id in event:
        vehicle = world_ref().get_actor(actor_id)
        print('Vehicle too close: %s' % vehicle.type_id)

sensor02.listen(callback)
```

### 数据 <span id="data"></span>

大多数传感器数据对象都具有将信息保存到磁盘的功能。这将允许它在其他环境中使用。

不同类型传感器的传感器数据差异很大。查看 [传感器参考](ref_sensors.md) 以获得详细说明。然而，所有这些都总是带有一些基本信息的标签。

| 传感器数据属性                                                                  | 类型                                                                                   | 描述                                                                                                            |
| -------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |---------------------------------------------------------------------------------------------------------------|
| `frame`                                                                                | int                                                                                    | 进行测量时的帧编号。                                                                                                    |
| `timestamp`                                                                            | double                                                                                 | 自轮数开始以来以模拟秒为单位的测量时间戳。 |
| `transform`                                                                            | [carla.Transform](<../python_api#carlatransform>)                                      | 测量时传感器的世界参考。                                                 |

<br>



!!! 重要
    `is_listening`是一种传感器方法，用于检查传感器是否有通过`listen`注册的回调。`stop()`是一种停止传感器监听的 __传感器方法__ 。`sensor_tick`是一个**蓝图属性**，用于设置接收数据之间的模拟时间。

---
## 传感器类型 <span id="types-of-sensors"></span>
 
### 相机 <span id="cameras"></span>

从他们的角度拍摄世界。对于返回 [carla.Image](<../python_api#carlaimage>) 的相机，您可以使用辅助类[carla.ColorConverter](python_api.md#carla.ColorConverter) 修改图像以表示不同的信息。

* 每个模拟步骤 __获取数据__ 。  


|传感器 |输出 | 概述       |
| ----------------- | ---------- | ------------------ |
| 深度 | [carla.Image](<../python_api#carlaimage>)  |在灰度图中渲染视野中元素的深度。          |
| RGB      | [carla.Image](<../python_api#carlaimage>)   | 提供清晰的周围环境视野。看起来就像一张正常的现场照片。   |
| 光流    | [carla.Image](<../python_api#carlaimage>)  | 渲染相机中每个像素的运动。  |
| 语义分割    | [carla.Image](<../python_api#carlaimage>)  | 根据元素的标签以特定颜色渲染视野中的元素。 |
| 实例分割    | [carla.Image](<../python_api#carlaimage>)  | 根据元素的标签和唯一的对象 ID 以特定颜色渲染视野中的元素。 |
| DVS    | [carla.DVSEventArray](<../python_api#carladvseventarray>)  | 作为事件流异步测量亮度强度的变化。  |

<br>



---
### 检测器 <span id="detectors"></span>

当附加的对象注册特定事件时获取数据。 

* 触发时 __获取数据__ 。  

| 传感器                                                                                                                                                                                          | 输出                                                                                                                                                                                          | 概述                                                                                                                                                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 碰撞                                                                   | [carla.CollisionEvent](<../python_api#carlacollisionevent>)                 | 检索其父级与其他参与者之间的碰撞。                   |
| 压线                                                               | [carla.LaneInvasionEvent](<../python_api#carlalaneinvasionevent>)           | 当其父级穿过车道标记时进行注册。                           |
| 障碍                                                                    | [carla.ObstacleDetectionEvent](<../python_api#carlaobstacledetectionevent>) | 检测其父级前方可能存在的障碍物。                             |

<br>



### 其他 <span id="other"></span>

不同的功能，例如导航、物理属性测量和场景的二维/三维点图。


* 每个模拟步骤 __检索数据__ 。  

| 传感器            | 输出                                                                                                                                                                                          | 概述                                                                    |
|----------------| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |-----------------------------------------------------------------------|
| 全球导航卫星系统(GNSS) | [carla.GNSSMeasurement](<../python_api#carlagnssmeasurement>)                                                                                                                                   | 检索传感器的地理位置。                                                           |
| 惯性测量单元(IMU)    | [carla.IMUMeasurement](<../python_api#carlaimumeasurement>)                                                                                                                                     | 包括加速度计、陀螺仪和指南针。                                                       |
| 激光雷达(LIDAR)    | [carla.LidarMeasurement](<../python_api#carlalidarmeasurement>)                                                                                                                                 | 旋转激光雷达。生成包含每个点的坐标和强度的 4D 点云，以对周围环境进行建模。                               |
| 雷达             | [carla.RadarMeasurement](<../python_api#carlaradarmeasurement>)                                                                                                                                 | 二维点图建模视线中的元素及其相对于传感器的运动。                                              |
| 责任敏感安全(RSS)    | [carla.RssResponse](<../python_api#carlarssresponse>)                                                                                                                                           | 根据安全检查修改应用于车辆的控制器。该传感器的工作方式与其他传感器不同，并且有专门的 [责任敏感安全文档](<../adv_rss>) （自 Carla 0.9.15 起已被弃用）。 |
| 语义激光雷达 | [carla.SemanticLidarMeasurement](<../python_api#carlasemanticlidarmeasurement>)                                                                                                                 | 旋转激光雷达。生成 3D 点云，其中包含有关实例和语义分割的额外信息。                                   |


---
这是传感器的包装以及它们如何检索模拟数据。 

Carla 的介绍到此结束。然而，还有很多东西需要学习。

* __继续学习。__ Carla 中有一些高级功能：渲染选项、交通管理器、记录器等等。这是了解更多关于他们的好时机。
 
<div class="build-buttons">
<p>
<a href="../adv_synchrony_timestep" target="_blank" class="btn btn-neutral" title="Synchrony and time-step">
同步和时间步长</a>
</p>
</div>

* __自由实验。__ 查看本文档的 __参考__ 部分。它包含有关 Python API 中的类、传感器、代码片段等的详细信息。

<div class="build-buttons">
<p>
<a href="../python_api" target="_blank" class="btn btn-neutral" title="Python API 参考">
Python API 参考</a>
</p>
</div>


* __发表您的意见。__ 有任何疑问、建议和想法欢迎在论坛中提出。

<div class="build-buttons">
<p>
<a href="https://github.com/OpenHUTB/carla_doc/issues" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
讨论页面</a>
</p>
</div>



