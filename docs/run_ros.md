# ROS 桥包

`carla_ros_bridge` 包是运行基本 ROS 桥接功能所需的主包。在本节中，您将学习如何准备 ROS 环境、运行 ROS 桥、如何配置设置、同步模式的使用、控制自主车辆以及可用的订阅、发布和服务的摘要。

- [__设置 ROS 环境__](#setting-the-ros-environment)
    - [准备 ROS 1 环境](#prepare-ros-1-environment)
    - [准备 ROS 2 环境](#prepare-ros-2-environment)
- [__运行 ROS 桥__](#running-the-ros-bridge)
- [__配置 Carla 设置__](#configuring-carla-settings)
- [__在同步模式下使用 ROS 桥__](#using-the-ros-bridge-in-synchronous-mode)
- [__自我车辆控制__](#ego-vehicle-control)
- [__ROS 应用程序接口__](#ros-api)
    - [订阅](#subscriptions)
    - [发布](#publications)
    - [服务](#services)
---

## 设置 ROS 环境

ROS 桥接器使用具有通用接口的单独实现来支持 ROS 1 和 ROS 2。当您想要运行 ROS 桥接器时，您必须根据您使用的每个终端中的 ROS 版本设置 ROS 环境：

#### 准备 ROS 1 环境：

要运行的命令取决于您是通过 Debian 软件包还是通过源构建安装 ROS 桥。您还需要更改 Debian 选项路径中的 ROS 版本：

```sh
# 用于debian安装ROS桥。根据已安装的ROS版本更改命令。
source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

# 为 GitHub 库安装 ROS 桥
source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

#### 准备 ROS 2 环境：

```sh
source ./install/setup.bash
```

## 运行 ROS 桥

设置 ROS 环境并运行 Carla 服务器后，您需要先启动`carla_ros_bridge`软件包，然后才能使用任何其他软件包。为此，请运行以下命令：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

还有其他启动文件结合了上述功能，在启动其他包或插件的同时启动 ROS 桥接器：

- `carla_ros_bridge_with_example_ego_vehicle.launch` (ROS 1) 和 `carla_ros_bridge_with_example_ego_vehicle.launch.py` (ROS 2) 沿着 [`carla_spawn_objects`](carla_spawn_objects.md) 和 [`carla_manual_control`](carla_manual_control.md) 包一起启动 ROS 桥。

---

## 配置 Carla 设置

配置应该在启动文件中设置，或者在从命令行运行文件时作为参数传递，例如：


```sh
roslaunch carla_ros_bridge carla_ros_bridge.launch passive:=True
```

可以使用以下设置：

* __use_sim_time__: 应设置为 __True__ 以确保 ROS 使用模拟时间而不是系统时间。该参数将使 ROS [`/clock`][ros_clock] 主题与 Carla 模拟时间同步。
*  __host and port__: 使用 Python 客户端连接到 Carla 的网络设置。
* __timeout__: 等待成功连接到服务器的时间。
* __passive__: 被动模式用于同步模式。启用后，ROS 桥接器将退居次要地位，而另一个客户端 __必须__ 与世界打交道。ROS 桥将等待接收来自所有传感器的所有预期数据。
*  __synchronous_mode__:
	*  __如果是 false__: 数据在每个节拍`world.on_tick()`和每个`sensor.listen()`回调时进行发布。
	*  __如果是 true （默认）__: ROS 桥在下一个节拍之前等待所有预期的传感器消息。这可能会减慢整个模拟的速度，但可以确保结果的可重复性。
*  __synchronous_mode_wait_for_vehicle_control_command__: 在同步模式下，暂停计时直到车辆控制完成。
*  __fixed_delta_seconds__: 模拟步骤之间的模拟时间（增量秒）。__它必须低于 0.1__。查看 [文档](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/) 以了解更多相关信息。
*  __ego_vehicle__: 用于识别自我车辆的参与者名称。将创建相关主题，以便能够通过 ROS 控制这些车辆。
* __town__: 使用可用的 Carla 城镇（例如“town01”）或 OpenDRIVE 文件（以 `.xodr` 结尾）。
*  __register_all_sensors__:
	*  __如果是 false__: 仅注册桥生成的传感器。
	*  __如果是 true（默认）__: 模拟中存在的所有传感器均已注册。


[ros_clock]: https://wiki.ros.org/Clock

---

## 在同步模式下使用 ROS 

ROS 桥默认以同步模式运行。它将等待当前帧内预期的所有传感器数据，以确保可重现的结果。

当以同步模式运行多个客户端时，只允许一个客户端向世界发节拍。除非启用被动模式，否则默认情况下，ROS 桥接器将是唯一允许与世界交互的客户端。在 [`ros-bridge/carla_ros_bridge/config/settings.yaml`](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/config/settings.yaml) 中启用被动模式将使 ROS 桥后退一步，并允许另一个客户端来操作世界。__另一个客户必须向世界发节拍，否则 Carla 将冻结。__

如果 ROS 桥接器不处于被动模式（ROS 桥接器是唯一的），则有两种方法将步进控制发送到服务器：

- 用消息向主题 `/carla/control` 发送 [`carla_msgs.CarlaControl`](ros_msgs.md#carlacontrolmsg) 。
- 使用 [控制 rqt 插件](rqt_plugin.md) 。该插件启动一个具有简单界面的新窗口。然后它用于管理步骤并在`/carla/control`主题中发布。要使用它，请在同步模式下使用 Carla 运行以下命令：
```sh
rqt --standalone rqt_carla_control
```

---

## 自我车辆控制

有两种控制自我车辆的模式：

1. 正常模式 - 从 `/carla/<ROLE NAME>/vehicle_control_cmd` 读取命令
2. 手动模式 - 从  `/carla/<ROLE NAME>/vehicle_control_cmd_manual` 读取命令。这允许手动覆盖软件堆栈发布的车辆控制命令。

您可以通过发布到 `/carla/<ROLE NAME>/vehicle_control_manual_override`来在两种模式之间切换。有关使用的示例，请参阅 [Carla 手动控制](carla_manual_control.md)。

要从命令行测试转向：

__1.__ 使用 ego 车辆启动 ROS 桥：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

__2.__ 在另一个终端中，发布到主题 `/carla/<ROLE NAME>/vehicle_control_cmd`

```sh
# 最大向前油门，最大向右转向

# 对于 ros1
rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

# 对于 ros2
ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10
```

当前车辆的状态可以通过主题 `/carla/<ROLE NAME>/vehicle_status` 获得。有关车辆的静态信息可以通过`/carla/<ROLE NAME>/vehicle_info`接收.

可以使用 [AckermannDrive](https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html) 消息来控制自我车辆。这可以通过使用 [Carla 阿克曼控制](carla_ackermann_control.md) 包来实现。

---

## ROS 应用程序接口

#### 订阅

| 主题                       | 类别                                                                                                                                | 描述               |
|--------------------------|-----------------------------------------------------------------------------------------------------------------------------------|------------------|
| `/carla/debug_marker`    | [可视化_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)                        | 在 Carla 世界中绘制标记。 |
| `/carla/weather_control` | [carla_msgs/CarlaWeatherParameters](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaWeatherParameters.msg) | 设置 Carla 天气参数    |
| `/clock`                 | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html)                                      | 在 ROS 中发布模拟时间。 |

<br>

!!! 笔记
    使用 `debug_marker`时，请注意标记可能会影响传感器发布的数据。支持的标记包括：箭头（由两个点指定）、点、立方体和线条。
<br>

#### 发布

| 主题                  | 类型                                                                                           | 描述                |
|---------------------|----------------------------------------------------------------------------------------------|-------------------|
| `/carla/status`     | [carla_msgs/CarlaStatus](ros_msgs.md#carlastatusmsg)                                         | 读取 Carla 的当前状态    |
| `/carla/world_info` | [carla_msgs/CarlaWorldInfo](ros_msgs.md#carlaworldinfomsg)                                   | 有关当前 Carla 地图的信息。 |
| `/clock`            | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | 在 ROS 中发布模拟时间。  |
| `/rosout`           | [rosgraph_msgs/Log](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html)     | ROS 日志记录。      |

<br>

#### 服务

| 主题                      | 类型                                                                                                                                                    | 描述                 |
|-------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------|
| `/carla/destroy_object` | [carla_msgs/DestroyObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/DestroyObject.srv) | 销毁一个物体 |
| `/carla/get_blueprints` | [carla_msgs/GetBlueprints.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/GetBlueprints.srv) | 获取蓝图    |
| `/carla/spawn_object`   | [carla_msgs/SpawnObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/SpawnObject.srv)     | 生成一个对象    |

---
