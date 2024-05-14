# 学习 Carla 的第一步

Carla 仿真器是一个综合解决方案，用于为自动驾驶 (Autonomous Driving, AD) 和其他机器人应用生成合成训练数据。Carla 仿真高度真实的环境，仿真现实世界中的城镇、城市和高速公路以及占据这些驾驶空间的车辆和其他物体。

Carla 仿真器还可用作评估和测试环境。您可以部署在仿真中训练过的自动驾驶智能体来测试和评估其性能和安全性，所有这些都在仿真环境的安全范围内，不会对硬件或其他道路使用者造成风险。

在本教程中，我们将介绍 Carla 中的标准工作流程，从启动服务器和连接客户端，到添加车辆、传感器和生成用于机器学习的训练数据。本教程旨在重点介绍细节，并尽可能高效地介绍使用 Carla 生成机器学习训练数据的关键步骤。有关工作流程每个部分的更多详细信息，例如蓝图库中可用的多种车辆或可用的替代类型传感器，请查阅文本中的链接或浏览左侧菜单。

* [__启动 Carla__](#Launching-carla-and-connecting-the-client)  
* [__加载地图__](#loading-a-map) 
* [__观察者导航__](#spectator-navigation)  
* [__添加非玩家角色__](#adding-npcs)  
* [__添加传感器__](#add-sensors)  
* [__使用交通管理器仿真车辆__](#animate-vehicles-with-traffic-manager)  
* [__将车辆分配为自我车辆__](#assign-a-vehicle-as-the-ego-vehicle)
* [__选择你的地图__](#choose-your-map) 
* [__选择你的车辆__](#choose-your-vehicles) 


## 启动 Carla 并连接客户端

可以使用 Windows 中的可执行文件或 Linux 中的 shell 脚本通过命令行启动 Carla。按照 [__Linux__](start_quickstart.md) 和 [__Windows__](start_quickstart.md) 的安装说明进行操作，然后从命令行 [__启动 Carla__](start_quickstart.md#running-carla)  ：

```sh
cd /carla/root
./CarlaUE4.sh
```
 
要通过 Python API 操作 Carla，我们需要通过开放端口将Python 客户端连接到服务器。客户端通过 [__客户端和世界对象__](foundations.md#world-and-client) 控制仿真器打开一个 Python 笔记本或创建一个新脚本，然后将以下代码添加到脚本的开头或 main 函数中：


```py
import carla
import random

# 链接到客户端并获取世界对象
client = carla.Client('localhost', 2000)
world = client.get_world()
```

[__客户端__](python_api#carlaclient) 对象用于维护客户端与服务器的连接，并具有许多用于应用命令以及加载或导出数据的功能。我们可以使用客户端对象加载替代地图或重新加载当前地图（重置为初始状态）。

端口可以选择任何可用的端口，默认设置为 2000，您也可以使用计算机的 IP 地址选择与 *localhost* 不同的主机。这样，Carla 服务器可以在联网计算机上运行，而 python 客户端可以从个人计算机运行。这对于区分用于运行 Carla 仿真器的 GPU 和用于神经网络训练的 GPU 特别有用，这两者对图形硬件的要求都很高。

!!! 笔记
    以下假设 Carla 在默认异步模式下运行。如果您使用 [__同步__](adv_synchrony_timestep.md) 模式，以下部分中的某些代码可能无法按预期工作。

## 加载地图 

在 Carla API 中，[__世界__](python_api.md#carla.World) 对象提供对仿真的所有元素的访问，包括地图、地图内的对象，例如建筑物、交通灯、车辆和行人。Carla 服务器通常加载默认地图（通常为 Town10）。如果您想使用备用地图启动 Carla，请使用以下 `config.py`脚本：

```sh
./config.py --map Town05 
```

我们还可以使用 world 对象从客户端加载地图：

```py
client.load_world('Town05')
``` 

请在 [__此处__](core_map.md) 查找有关 Carla 地图的更多信息。


## 观察者导航

观察者是仿真的视图。默认情况下，当您在连接屏幕的计算机上运行 Carla 服务器时，观众会在新窗口中打开，除非您指定`-RenderOffScreen` 命令行选项。

观察者有助于可视化您的仿真。使用观察者，您可以熟悉已加载的地图，并查看所做的任何更改的结果，例如添加车辆、更改天气、打开/关闭地图的各个图层以及用于调试目的。

您可以使用鼠标控制观察者视图的俯仰和偏航，并使用 QWE-ASD 键移动观众，让观众飞到世界各地：

- Q - 向上移动（朝向窗口的顶部边缘）
- E - 向下移动（朝向窗口的下边缘）

- W - 向前移动
- S - 向后移动
- A - 向左移动
- D - 向右移动

左键单击并在观察者窗口中上下拖动鼠标来控制俯仰，左右拖动鼠标来控制偏航。


![flying_spectator](./img/tuto_G_getting_started/flying_spectator.gif)

观察者及其属性可以通过 Python API 访问和操作：

```py
# 获取观察者对象
spectator = world.get_spectator()

# 通过变换获得观察者的位置和旋转
transform = spectator.get_transform()

location = transform.location
rotation = transform.rotation

# 用空变换设置观察者
spectator.set_transform(carla.Transform())
# 这将设置观众在地图的原点，0度俯仰，偏航和滚动-一个很好的方式来定位自己在地图上
```

## 添加非玩家角色

现在我们已经加载了地图并且服务器已启动并运行，我们现在需要用一些车辆填充我们的仿真，以仿真具有交通和其他道路使用者或非玩家角色的真实环境。

要生成车辆，首先，我们需要从蓝图库中选择我们想要的车辆。

```py
# 获取蓝图库和车辆蓝图过滤器
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
```

现在我们有了蓝图，我们需要在地图上找到一些合适的地点来生成我们的车辆。为此，每个 Carla 地图都提供了预定义的生成点，均匀分布在整个地图的道路上。

```py
# 获得地图的生成点
spawn_points = world.get_map().get_spawn_points()

# 在每个生成点随机生成50辆车，我们从蓝图库中随机选择一辆车
for i in range(0,50):
    world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
```

现在我们还应该添加一辆车作为仿真的中心点。为了训练自主智能体，我们需要仿真自主智能体将控制的车辆。用 Carla 的说法，我们经常将这种车辆称为“自我车辆”。

```py
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
```

除了车辆之外，Carla 还提供行人添加到仿真中，以仿真真实的驾驶场景。在 Carla 术语中，车辆和行人被称为参与者，请在 [__此处__](core_actors.md) 了解有关它们的更多信息。


## 添加传感器

现代自动驾驶汽车通过一系列附加传感器来理解和解释其环境。这些传感器包括光学摄像机、光流摄像机、激光雷达、雷达和加速度计等。Carla 内置了多种类型的传感器模型，用于创建机器学习的训练数据。传感器可以连接到车辆上，也可以连接到固定点来建模，例如闭路电视摄像机。

在这里，我们将在自我车辆上安装一个标准摄像头传感器来记录一些视频数据：

```py
# 创建一个变换，将摄像机放置在车辆的顶部
camera_init_trans = carla.Transform(carla.Location(z=1.5))

# 通过定义其属性的蓝图来创建相机
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

# 生成相机并将其连接到我们的自我车辆上
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
```

一旦我们生成了相机，我们需要通过`listen()`方法将其设置为录制。Listen 方法将定义如何处理数据的回调作为参数。您可以将其流式传输到另一个程序或将其保存到磁盘。

我们将使用 lambda 函数作为回调将数据保存到磁盘：

```py
# 用 PyGame 回调启动相机
camera.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))
```

这会将数据作为一系列根据仿真帧编号命名的 PNG 图像文件保存到 `out/` 文件夹中。

有多种不同类型的传感器可供选择。在 [__这里__](core_sensors.md) 您可以更深入地研究可用的传感器阵列以及如何使用它们。


## 使用交通管理器仿真车辆

现在我们已经将交通和自我车辆添加到仿真中并开始记录摄像机数据，现在我们需要使用交通管理器将车辆设置为运动。[__交通管理器__](adv_traffic_manager.md) 是 Carla 的一个组件，它控制车辆在仿真内的地图道路上自动移动，遵循道路惯例并表现得像真正的道路使用者。

我们可以使用`world.get_actors()`方法找到仿真中的所有车辆，对所有车辆进行过滤。然后我们可以使用该`set_autopilot()`方法将车辆的控制权移交给交通管理器。

```py
for vehicle in world.get_actors().filter('*vehicle*'):
    vehicle.set_autopilot(True)
```

现在您的仿真正在运行，有许多车辆在地图上行驶，并且摄像机记录其中一辆车辆的数据。然后，该数据可用于提供机器学习算法来训练自动驾驶智能体。流量管理器具有许多用于自定义流量行为的功能，请在 [__此处__](tuto_G_traffic_manager.md) 了解更多信息。

这是仿真的最基本可能设置，现在您可以更深入地了解有关可用于生成数据的许多额外传感器的文档，以及 Carla 的许多其他功能，这些功能可以使您的仿真更加详细和更多实际的。

---

## 将车辆分配为自我车辆

__自我车辆__ 是使用 Carla 时需要牢记的一个重要概念。自我车辆是指将成为仿真焦点的车辆。在大多数 CARLA 用例中，它可能是您将连接传感器的车辆和/或您的自动驾驶机器学习堆栈将控制的车辆。它很重要，因为它是一些有助于提高仿真效率的仿真操作的基础，例如：

* __加载大型地图的地图图块__: 大型地图（如 Town 12）由图块组成，仅在需要提高 Carla 性能时才加载。自我车辆的位置决定了使用哪些图块。只有最靠近自我车辆的图块才会被加载。

* __混合物理模式__: 如果您的仿真包含由交通管理器控制的大量车辆，则计算所有这些车辆的物理量在计算上非常昂贵。[混合物理模式](adv_traffic_manager.md#hybrid-physics-mode) 使物理计算仅限于自我车辆附近的车辆，从而节省计算资源。

要定义自我车辆，您应该在生成自我车辆时设置`role_name`车辆[carla.Actor](python_api.md#carlaactor) 对象 [蓝图](python_api.md#carlaactorblueprint) 的属性：

```py
ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')

ego_bp.set_attribute('role_name', 'hero')

ego_vehicle = world.spawn_actor(ego_bp, random.choice(spawn_points))
```
---
## 选择你的地图

![maps_montage](./img/catalogue/maps/maps_montage.webp)

Carla 附带了几张预制地图，专注于提供多种功能。这些地图呈现了城市、乡村和住宅等一系列环境。还有不同的建筑风格和多种不同的道路布局，从无标记的乡村道路到多车道高速公路。浏览[目录](catalogue.md)或下表中的地图指南。

| 城镇                      | 概括                                                                                                                                                         |
|-------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [__城镇01__](map_town01.md) | 一个简单的小镇，有一条河流和几座桥梁。                                                                                                                                        |
| [__城镇02__](map_town02.md) | 一个简单的小镇，住宅和商业建筑混合在一起。                                                                                                                                      |
| [__城镇03__](map_town03.md) | 更大的城市地图，带有环岛和大型路口。                                                                                                                                         |
| [__城镇04__](map_town04.md) | 一座镶嵌在群山之中的小镇，有一条特殊的“8字形”*无限*公路。                                                                                                                            |
| [__城镇05__](map_town05.md) | 方形网格城镇，有十字路口和一座桥梁。每个方向有多个车道。对于执行变道很有用。                                                                                                                     |
| [__城镇06__](map_town06.md) | 长的多车道高速公路，有许多高速公路入口和出口。它还有一个 [**密歇根左翼**](<https://en.wikipedia.org/wiki/Michigan_left>) 。                                                                  |
| [__城镇07__](map_town07.md) | 乡村环境，道路狭窄，有玉米、谷仓，几乎没有红绿灯。                                                                                                                                  |
| **城镇08**                | 用于 [排行榜](https://leaderboard.carla.org/) 挑战的秘密“看不见”小镇                                                                                                      |
| **城镇09**                | 用于 [排行榜](https://leaderboard.carla.org/) 挑战的秘密“看不见”小镇  |
| [__城镇10__](map_town10.md) | 市中心的城市环境，拥有摩天大楼、住宅楼和海滨长廊。                                                               |
| __城镇11__                | 未装饰的大地图。                                                                                                                           |
| [__城镇12__](map_town12.md) | 一张包含许多不同区域的大地图，包括高层建筑、住宅区和乡村环境。                                                      |
| [__城镇13__](map_town13.md) | 一张与 12 号镇规模相似的大地图，但具有鲜明的特色。 |
| [__城镇15__](map_town15.md) | 基于巴塞罗那自治大学道路布局的地图。 |

您可以浏览 Carla 安装中的可用地图：

```py
client.get_available_maps()
```

这将包括您自己构建或导入的地图。

当您选择地图时，请像这样加载它：

```py
client.load_world('Town03_Opt')
```

---

## 选择你的车辆


![vehicles_overview](./img/catalogue/vehicles/vehicle_montage.webp)

Carla 提供了一个车辆库，可以让您的仿真充满交通。浏览 [Carla 车辆目录](catalogue_vehicles.md) 中的车辆型号。

您可以通过过滤蓝图库来查看所有可用的车辆蓝图。

```py
for bp in world.get_blueprint_library().filter('vehicle'):
    print(bp.id)
```