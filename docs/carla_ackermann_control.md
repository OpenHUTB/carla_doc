# Carla 阿克曼控制

[`carla_ackermann_control` 软件包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ackermann_control) 用于通过阿克曼消息控制 Carla 车辆。该包将 [阿克曼消息][ackermanncontrolmsg] 转换为 [CarlaEgoVehicleControl][carlaegovehiclecontrolmsg] 消息。它从 Carla 读取车辆信息，并将该信息传递给基于 Python 的 PID 控制器，调用`simple-pid`控制器来控制加速度和速度。

[ackermanncontrolmsg]: https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html
[carlaegovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#carlaegovehiclecontrolmsg

- [__配置__](#configuration)
- [__测试控制消息__](#testing-control-messages)
- [__ROS 应用程序接口__](#ros-api)
    - [订阅](#subscriptions)
    - [发布](#publications)

---

### 配置 <span id="configuration"></span>

使用 ROS 1 和 ROS 2 时，可以在 [配置文件][ackermanconfig] 中初始设置参数，也可以在运行时通过 ROS 1 中的 ROS [动态重新配置][rosdynamicreconfig] 来设置参数。

[ackermanconfig]: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/config/settings.yaml
[rosdynamicreconfig]: https://wiki.ros.org/dynamic_reconfigure

---

### 测试控制消息 <span id="testing-control-messages"></span>

通过主题 `/carla/<ROLE NAME>/ackermann_cmd` 向汽车发送命令来测试设置。例如，通过运行以下命令以 10 米/秒的速度移动参与者名称为`ego_vehicle`的自我车辆：

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

或者让车辆向前移动，同时转弯 1.22 弧度的角度：

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

---

### ROS 应用程序接口 <span id="ros-api"></span>

#### 订阅 <span id="subscriptions"></span>

| 主题 | 类型 | 描述 |
|--|--|--|
|`/carla/<ROLE NAME>/ackermann_cmd` | [ackermann_msgs.AckermannDrive][ackermanncontrolmsg] | 转向命令的 __订阅者__  |

<br>

#### 发布 <span id="publications"></span>

|主题|类型|描述|
|--|--|--|
| `/carla/<ROLE NAME>/ackermann_control/control_info` | [carla_ackermann_control.EgoVehicleControlInfo][egovehiclecontrolmsg] | 控制器内使用的当前值（对于调试有用） |

[egovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#egovehiclecontrolinfomsg

<br>
