# Scenic

本指南概述了如何将 Scenic 与 Carla 结合使用，通过单个场景定义生成多个不同的场景。它假设用户事先了解场景语法。如果您需要了解有关 Scenic 的更多信息，请阅读他们的 [“Scenic 入门”](https://scenic-lang.readthedocs.io/en/latest/quickstart.html) 指南，并查看他们有关创建 [静态](https://scenic-lang.readthedocs.io/en/latest/tutorials/tutorial.html) 和 [动态](https://scenic-lang.readthedocs.io/en/latest/tutorials/dynamics.html) 场景的教程。

在本指南结束时，您将了解：

- 在 Carla 上运行 Scenic 脚本所需的最低要求。
- 如何编写简单的场景定义来生成大量场景模拟。
- 如何在 Carla 上运行 Scenic 脚本。
- 用于在 Carla 上配置场景模拟的参数。

---

- [__在你开始之前__](#before-you-begin)
- [__Scenic 域__](#scenic_domains)
- [__创建与 Carla 一起使用的 Scenic 场景__](#creating-a-scenic-scenario-to-use-with-carla)
- [__运行场景__](#run-the-scenario)
- [__附加参数__](#additional-parameters)

---

## 在你开始之前 <span id="before-you-begin"></span>

在将 Scenic 与 Carla 结合使用之前，您需要满足以下要求：

- 安装 [Python 3.8](https://www.python.org/downloads/) 或更高版本。
- 安装 [Scenic](https://scenic-lang.readthedocs.io/en/latest/quickstart.html#installation)。

---

## Scenic 域 <span id="scenic_domains"></span>

Scenic 有一个通用驾驶域，允许用户定义可以在任何驾驶模拟器上运行的场景。此外，它还有特定于每个模拟器的其他域。请在 [此处](https://scenic-lang.readthedocs.io/en/latest/libraries.html) 查看有关 Scenic 域的更多信息。

每个领域中特别重要的是行为和动作的定义。检查以下链接，获取有关 Scenic 驾驶域和 Carla 域的行为和操作的参考材料：

- [the Scenic 驾驶领域的行为](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.behaviors.html)
- [Carla 域中的行为](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.simulators.carla.behaviors.html)
- [Scenic 驾驶域的动作](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.actions.html)
- [Carla 域中的动作](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.simulators.carla.actions.html#module-scenic.simulators.carla.actions)

---

## 创建与 Carla 一起使用的 Scenic 场景 <span id="creating-a-scenic-scenario-to-use-with-carla"></span>

本节将介绍如何编写基本的场景脚本，其中领头车辆由于道路上的障碍物而突然减速。然后，本车需要突然制动以避免与前车发生碰撞。[完整的脚本](https://github.com/BerkeleyLearnVerify/Scenic/blob/master/examples/carla/Carla_Challenge/carlaChallenge2.scenic) 以及涉及更复杂道路网络的其他示例可以在 Scenic 存储库中找到。


__1.__ 设置地图参数并声明场景使用的模型：

- 应将 `.xodr` 文件设置为 [`map`][scenic_map] 参数值，稍后将使用该文件生成道路网络信息。 
- 参数 `carla_map` 是指您想要在模拟中使用的 Carla 地图的名称。如果已定义，则 Scenic 将加载地图的所有资产（建筑物、树木等），如果未定义，则将使用 [OpenDRIVE 独立模式](adv_opendrive.md) 。
- 该模型包括特定于 Carla 上运行场景的所有实用程序。这应该在您想要在 Carla 上运行的所有脚本中定义。

```scenic
## SET MAP AND MODEL
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model
```

[scenic_map]: https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.model.html?highlight=map#module-scenic.domains.driving.model

__2.__ 定义场景中要使用的常量：

该场景涉及两辆车，领头车辆和自我车辆。我们将定义自我车辆模型、两辆车的速度、制动距离阈值以及制动量。

```scenic
## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = 10
EGO_BRAKING_THRESHOLD = 12

LEAD_CAR_SPEED = 10
LEADCAR_BRAKING_THRESHOLD = 10

BRAKE_ACTION = 1.0
```

__3__. 定义场景行为：

在这种情况下，我们将使用 Scenic [行为库](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.behaviors.html) 来指示自我车辆以预定义的速度沿着车道行驶，然后在与另一辆车相距一定距离时紧急制动。领先的车辆也会以预定的速度沿着车道行驶，并在距离任何物体一定距离内紧急制动：

```scenic
## DEFINING BEHAVIORS
# EGO BEHAVIOR: Follow lane, and brake after passing a threshold distance to the leading car
behavior EgoBehavior(speed=10):
    try:
        do FollowLaneBehavior(speed)

    interrupt when withinDistanceToAnyCars(self, EGO_BRAKING_THRESHOLD):
        take SetBrakeAction(BRAKE_ACTION)

# LEAD CAR BEHAVIOR: Follow lane, and brake after passing a threshold distance to obstacle
behavior LeadingCarBehavior(speed=10):
    try: 
        do FollowLaneBehavior(speed)

    interrupt when withinDistanceToAnyObjs(self, LEADCAR_BRAKING_THRESHOLD):
        take SetBrakeAction(BRAKE_ACTION)
```

__4.__ 生成路网：

Scenic [道路库](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.roads.html) 用于生成道路网络几何形状和交通信息。道路网络由 [`Network`](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.domains.driving.roads.html#scenic.domains.driving.roads.Network) 类的实例表示，并由脚本开头定义的文件生成。

```scenic
## DEFINING SPATIAL RELATIONS
# make sure to put '*' to uniformly randomly select from all elements of the list, 'lanes'
lane = Uniform(*network.lanes)
```

__5.__ 设置场景：

我们现在将定义车辆的起始位置和物体的放置。

- 在车道中间放置垃圾桶：

```scenic
obstacle = Trash on lane.centerline
```

- 将领先车辆以预定速度沿道路行驶，距离障碍物后方 50 至 30 米：

```scenic
leadCar = Car following roadDirection from obstacle for Range(-50, -30),
        with behavior LeadingCarBehavior(LEAD_CAR_SPEED)
```

- 将本车以预定速度沿着道路行驶，距离前车 15 到 10 米：

```scenic
ego = Car following roadDirection from leadCar for Range(-15, -10),
        with blueprint EGO_MODEL,
        with behavior EgoBehavior(EGO_SPEED)
```

- 要求场景发生在距路口 80 米以上的位置：

```scenic
require (distance to intersection) > 80
```

__6.__ 设置结束点，以便脚本知道场景何时完成：

当自我车辆的速度低于每秒 0.1 米并且距离障碍物不到 30 米时，该场景将结束。

```scenic
terminate when ego.speed < 0.1 and (distance to obstacle) < 30
```

---

### 运行场景 <span id="run-the-scenario"></span>

要运行 Scenic 场景：

__1.__ 启动 Carla 服务器。

__2.__ 运行以下命令：

```scenic
scenic path/to/scenic/script.scenic --simulate
```

将出现一个 pygame 窗口，并且场景将重复播放，每次都会在脚本中设置的限制范围内生成一个独特的场景。要停止场景生成，请在终端中按`ctrl + C`。

---

### 附加参数 <span id="additional-parameters"></span>

Carla 模型提供了几个全局参数，可以在使用 [`param` 语句](https://scenic-lang.readthedocs.io/en/latest/syntax_details.html#param-identifier-value) 或通过命令行使用  [`--param` option](https://scenic-lang.readthedocs.io/en/latest/options.html#cmdoption-p) 选项的场景中覆盖这些参数。

下面是 Carla 模型中的可配置参数表：

| 名称          | 值              | 描述                                                                                                                          |
|-------------|----------------|-----------------------------------------------------------------------------------------------------------------------------|
| `carla_map` | `str`          | 要使用的 Carla 地图的名称（例如“Town01”）。如果设置为``None``，Carla 将尝试使用 [`map`][scenic_map] 参数中定义的 `.xodr` 文件在 OpenDRIVE 独立模式下创建世界。          |
| `timestep`  | `float`        | 用于模拟的时间步长（Scenic 中断 Carla 运行行为、检查需求等的频率）以秒为单位。默认值为 0.1 秒。                                                                   |
| `weather`   | `str` 或 `dict` | 用于天气的模拟。可以是标识 Carla 天气预设之一的字符串（例如“ClearSunset”），也可以是指定所有 [天气参数](python_api.md#carla.WeatherParameters) 的字典。默认值是所有天气预设的均匀分布。 |
| `address`   | `str`          | 连接到 Carla 的 IP 地址。默认值为本地主机 (127.0.0.1)。                                                                                     |
| `port`      | `int`          | 连接至 Carla 的端口。默认值为 2000。                                                                                                    |
| `timeout`   | `float`        | 尝试连接到 Carla 时等待的最长时间（以秒为单位）。默认值为 10。                                                                                        |
| `render`    | `int`          | 是否让 Carla 创建一个窗口，从自我客体的角度显示模拟：`1`是，`0`否。默认`1`。                                                                              |
| `record`    | `str`          | 如果非空，则保存用于重放模拟的 Carla 记录文件的文件夹。         |

## 参考链接

* [Carla中交通场景的Scenic世界模型](https://docs.scenic-lang.org/en/3.x/modules/scenic.simulators.carla.model.html) ，对应的 [代码](https://docs.scenic-lang.org/en/3.x/_modules/scenic/simulators/carla/model.html) 。
* 

<br>
