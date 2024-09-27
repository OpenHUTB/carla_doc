# 第一、世界和客户端

客户端和世界是 Carla 的两个基本要素，是操作模拟及其参与者的必要抽象。

本教程从定义这些元素的基础知识和创建，到描述它们的可能性。如果阅读过程中出现任何疑问或问题，[Carla 论坛](https://github.com/carla-simulator/carla/discussions/) 可以为您解决。

*   [__客户端__](#the-client)  
    * [客户端创建](#client-creation)  
    * [世界连接](#world-connection)  
    * [使用命令](#using_command)  
    * [其他客户端实用程序](#other-client-utilities)  
*   [__世界__](#the-world)  
	*   [参与者](#actors)  
	*   [天气](#weather)  
	*   [灯光](#lights)  
	*   [调试](#debugging)  
	*   [世界快照](#world-snapshots)  
	*   [世界设置](#world-settings)  

---
## 客户端 <span id="the-client"></span>

客户端是 Carla 架构中的主要元素之一。它们连接到服务器、检索信息并更改命令。这是通过脚本完成的。客户端识别自己的身份，并连接到世界，然后进行模拟操作。

除此之外，客户还可以访问高级 Carla 模块、功能并应用命令批处理。本节仅介绍命令批处理。这些对于诸如生成大量参与者之类的基本事情很有用。其余功能更加复杂，将在 __高级步骤__ 中各自的页面中进行解决。 

查看 Python API 参考中的 [__carla.Client__](python_api.md#carla.Client) 以了解该类的特定方法和变量。


### 客户端创建 <span id="client-creation"></span>

需要两件事。标识它的 __IP__ 地址以及与服务器通信的 __两个 TCP 端口__ 。可选的第三个参数设置工作线程的数量。默认情况下，该值设置为全部 (`0`)。Python API 参考中的[carla.Client](python_api.md#carla.Client.__init__) 包含一个片段，显示如何在运行脚本时将这些解析为参数。

```py
client = carla.Client('localhost', 2000)
```
默认情况下，Carla 使用本地主机 IP 和端口 2000 进行连接，但这些可以随意更改。在本例中，第二个端口始终为`n+1`,2001。

创建客户端后，设置其 __time-out__ 。这限制了所有网络操作，以便这些操作不会永远阻止客户端。如果连接失败，将返回错误。

```py
client.set_timeout(10.0) # 秒
```

可以连接许多客户端，因为一次运行多个脚本是很常见的。在具有高级 Carla 功能（例如交通管理器）的多客户端方案中工作，必然会使通信更加复杂。

!!! 笔记
    客户端和服务器有不同的`libcarla`模块。如果版本不同，可能会出现问题。这可以使用`get_client_version()`和`get_server_version()`方法进行检查。

### 世界连接 <span id="world-connection"></span>

客户端可以相当轻松地连接和检索当前世界。

```py
world = client.get_world()
```

客户端还可以获得可用地图列表来更改当前地图。这将销毁当前的世界并创造一个新的世界。
```py
print(client.get_available_maps())
...
world = client.load_world('Town01')
# client.reload_world() 创建一个相同地图世界的新实例。
```

每个世界对象都有一个 `id` 或世代。每当客户端调用 `load_world()` 或 `reload_world()` 时，前一个被销毁。一个新的世代是从头开始创建的。在此过程中虚幻引擎不会重新启动。

### 使用命令 <span id="using_command"></span>

__命令__ 是一些最常见的 Carla 方法的改编，可以批量应用。例如，[command.SetAutopilot](python_api.md#command.SetAutopilot) 相当于[command.SetAutopilot](python_api.md#command.SetAutopilot) ，启用车辆的自动驾驶功能。但是，使用[Client.apply_batch](python_api.md#carla.Client.apply_batch) 或[Client.apply_batch_sync()](python_api.md#carla.Client.apply_batch_sync) 方法，可以在一个模拟步骤中应用一系列命令。这对于通常应用于数百个元素的方法来说变得非常有用。

以下示例使用批处理一次性销毁一系列车辆。

```py
client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
```

Python API 参考的 [最新部分](python_api.md#command.ApplyAngularVelocity) 列出了所有可用的命令。

### 其他客户端实用程序 <span id="other-client-utilities"></span>

客户端对象的主要目的是获取或改变世界，并应用命令。但是，它还提供对一些附加功能的访问。 

*   __交通管理器。__ 该模块负责每辆设置为自动驾驶的车辆以重建城市交通。
*   __[记录器](adv_recorder.md).__ 允许重新进行以前的模拟。使用 [快照](core_world.md#world-snapshots) 总结每帧的模拟状态。

---
## 世界 <span id="the-world"></span>

模拟的主要标尺。它的实例应该由客户端检索。它不包含世界本身的模型，这是 [Map](core_map.md) 类的一部分。相反，大多数信息和常规设置可以从此类访问。

*   模拟中的参与者和观察者。
*   蓝图库。
*   地图。
*   模拟设置。
*   快照。
*   天气和灯光管理器。

它的一些最重要的方法是 _getters_ ，精确地检索这些元素的信息或实例。查看 [carla.World](python_api.md#carla.World) 了解更多信息。

### 参与者 <span id="actors"></span>

世界上有不同的与参与者相关的方法，允许不同的功能。

*   生成参与者（但不销毁他们）。
*   获得场景中的每一位参与者，或者找到一个特定的参与者
*   访问蓝图库。  
*   访问观察参与者、模拟的视角。 
*   检索适合生成参与者的随机位置。

生成将在 [第二部分、参与者和蓝图](core_actors.md) 。需要对蓝图库、属性等有一定的了解。

### 天气 <span id="weather"></span>

天气本身并不是一个类，而是一组可从世界获取的参数。参数化包括太阳方向、云量、风、雾等等。辅助类[carla.WeatherParameters](python_api.md#carla.WeatherParameters) 用于定义自定义天气。
```py
weather = carla.WeatherParameters(
    cloudiness=80.0,
    precipitation=30.0,
    sun_altitude_angle=70.0)

world.set_weather(weather)

print(world.get_weather())
```

有一些天气预设可以直接应用于世界。这些列在[carla.WeatherParameters](python_api.md#carla.WeatherParameters) 中并可作为枚举访问。 

```py
world.set_weather(carla.WeatherParameters.WetCloudySunset)
```

还可以使用 Carla 提供的两个脚本来自定义天气。

*   __`environment.py`__ *(在 `PythonAPI/util` 目录下)* — 提供对天气和光照参数的访问，以便可以实时更改这些参数。

<details>
<summary> <b>environment.py</b>  中的可选参数 </summary>

```sh
  -h, --help            show this help message and exit
  --host H              IP of the host server (default: 127.0.0.1)
  -p P, --port P        TCP port to listen to (default: 2000)
  --sun SUN             Sun position presets [sunset | day | night]
  --weather WEATHER     Weather condition presets [clear | overcast | rain]
  --altitude A, -alt A  Sun altitude [-90.0, 90.0]
  --azimuth A, -azm A   Sun azimuth [0.0, 360.0]
  --clouds C, -c C      Clouds amount [0.0, 100.0]
  --rain R, -r R        Rain amount [0.0, 100.0]
  --puddles Pd, -pd Pd  Puddles amount [0.0, 100.0]
  --wind W, -w W        Wind intensity [0.0, 100.0]
  --fog F, -f F         Fog intensity [0.0, 100.0]
  --fogdist Fd, -fd Fd  Fog Distance [0.0, inf)
  --wetness Wet, -wet Wet
                        Wetness intensity [0.0, 100.0]
```
</details><br>

*   __`dynamic_weather.py`__ *(在 `PythonAPI/examples` 目录下)* — 启用开发人员为每张 Carla 地图准备的特定天气周期。

<details>
<summary> <b>dynamic_weather.py</b> 中的可选参数 </summary>

```sh
  -h, --help            show this help message and exit
  --host H              IP of the host server (default: 127.0.0.1)
  -p P, --port P        TCP port to listen to (default: 2000)
  -s FACTOR, --speed FACTOR
                        rate at which the weather changes (default: 1.0)
```
</details><br>

!!! 笔记
    天气的变化不会影响物理。它们只是相机传感器可以捕获的视觉效果。

__当 sun_altitude_angle < 0 时，夜间模式开始__，这被认为是日落。这是灯光变得特别重要的时候。

### 灯光 <span id="lights"></span>

*   当模拟进入夜间模式时， __路灯__ 会自动打开。灯光由地图开发人员放置，并可作为 [__carla.Light__](python_api.md#carla.Light) 对象访问。颜色和强度等属性可以随意更改。[__carla.LightState__](python_api.md#carla.LightState) 类型的变量 __light_state__ 允许在一次调用中设置所有这些。 路灯使用其类型为 [__carla.LightState__](python_api.md#carla.LightState) 的属性 __light_state__ 进行分类。这允许将灯分类为路灯、建筑物灯...可以检索 [__carla.LightManager__] 的实例来在一次调用中处理一组灯。

```py
# 获得灯光管理器和灯光
lmanager = world.get_lightmanager()
mylights = lmanager.get_all_lights()

# 定制一个特定的灯光
light01 = mylights[0]
light01.turn_on()
light01.set_intensity(100.0)
state01 = carla.LightState(200.0,red,carla.LightGroup.Building,True)
light01.set_light_state(state01)

# 定制一组灯
my_lights = lmanager.get_light_group(carla.LightGroup.Building)
lmanager.turn_on(my_lights)
lmanager.set_color(my_lights,carla.Color(255,0,0))
lmanager.set_intensities(my_lights,list_of_intensities)
```

* __车灯__ 必须由用户打开/关闭。每辆车都有一组在[__carla.VehicleLightState__](python_api.md#carla.VehicleLightState) 中列出的灯。到目前为止，并非所有车辆都集成了车灯。以下是截至撰写本文时可用的列表。
	*   __自行车。__ 它们都有前后示廓灯。
	*   __摩托车。__ 雅马哈和哈雷戴维森型号。
	*   __汽车。__ 奥迪 TT、雪佛兰、道奇（警车）、钰创、林肯、野马、特斯拉3S、大众T2以及即将到来的 Carla 新客人。  

可以使用 [carla.Vehicle.get_light_state](python_api.md#carla.Vehicle.get_light_state) 和 [carla.Vehicle.set_light_state](#python_api.md#carla.Vehicle.set_light_state) 方法随时检索和更新车辆的灯光。它们使用二进制运算来自定义灯光设置。

```py
# 打开位置灯
current_lights = carla.VehicleLightState.NONE
current_lights |= carla.VehicleLightState.Position
vehicle.set_light_state(current_lights)
```

!!! 笔记
    还可以使用天气 `environment.py` 部分中描述的实时设置灯光。

### 调试 <span id="debugging"></span>

世界对象有一个 [carla.DebugHelper](python_api.md#carla.DebugHelper) 对象作为公共属性。它允许在模拟过程中绘制不同的形状。这些用于跟踪正在发生的事件。以下示例将在参与者的位置和旋转处绘制一个红色框。

```py
debug = world.debug
debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(0.5,0.5,2)),actor_snapshot.get_transform().rotation, 0.05, carla.Color(255,0,0,0),0)
```

此示例在 [carla.DebugHelper](python_api.md#carla.DebugHelper.draw_box) 的片段中进行了扩展，展示了如何为世界快照中的每个参与者绘制方框。

### 世界快照 <span id="world-snapshots"></span>

包含模拟中每个参与者在单个帧中的状态。一种带有时间参考的静态世界图像。即使在异步模式下，信息也来自相同的模拟步骤。

```py
# 检索当前帧的世界快照。
world_snapshot = world.get_snapshot()
```

[carla.WorldSnapshot](python_api.md#carla.WorldSnapshot) 包含 [carla.Timestamp](python_api.md#carla.Timestamp) 和 [carla.ActorSnapshot](python_api.md#carla.ActorSnapshot) 列表。可以使用参与者 `id` 的搜索参与者快照。快照列出了其中出现的参与者 `id`。

```py
timestamp = world_snapshot.timestamp # 获取时间参考

for actor_snapshot in world_snapshot: # 获取参与者和快照信息
    actual_actor = world.get_actor(actor_snapshot.id)
    actor_snapshot.get_transform()
    actor_snapshot.get_velocity()
    actor_snapshot.get_angular_velocity()
    actor_snapshot.get_acceleration()  

actor_snapshot = world_snapshot.find(actual_actor.id) # 获得参与者快照
```

### 世界设置 <span id="world-settings"></span>

世界可以使用一些高级的模拟配置。这些决定了渲染条件、模拟时间步长以及客户端和服务器之间的同步。它们可以从辅助类[carla.WorldSettings](python_api.md#carla.WorldSettings) 访问。 

目前，默认的 Carla 以最佳图形质量、可变时间步长和异步方式运行。要进一步深入了解此问题，请查看 __高级步骤__ 部分。有关 [同步、时间步长](adv_synchrony_timestep.md) 以及 [渲染选项](adv_rendering_options.md) 的页面可能是一个很好的起点。

---
这是对世界和客户端对象的包装。下一步将仔细研究参与者和蓝图，为模拟赋予生命。

继续阅读以了解更多信息。请访问论坛，发表在阅读过程中想到的任何疑问或建议。

<div text-align: center>
<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="CARLA forum">
Carla 论坛</a>
</p>
</div>
<div class="build-buttons">
<p>
<a href="../core_actors" target="_blank" class="btn btn-neutral" title="2nd. Actors and blueprints">
第二、参与者和蓝图</a>
</p>
</div>
</div>
