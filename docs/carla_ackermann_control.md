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

### 配置

Parameters can be set both initially in a [configuration file][ackermanconfig] when using both ROS 1 and ROS 2 and during runtime via ROS [dynamic reconfigure][rosdynamicreconfig] in ROS 1. 

[ackermanconfig]: https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/config/settings.yaml
[rosdynamicreconfig]: https://wiki.ros.org/dynamic_reconfigure

---

### 测试控制消息

Test the setup by sending commands to the car via the topic `/carla/<ROLE NAME>/ackermann_cmd`. For example, move an ego vehicle with the role name of `ego_vehicle` forward at a speed of 10 meters/sec by running this command:

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

Or make the vehicle move forward while turning at an angle of 1.22 radians:

```bash

# ROS 1
rostopic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

# ROS 2
ros2 topic pub /carla/ego_vehicle/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 1.22, steering_angle_velocity: 0.0, speed: 10, acceleration: 0.0, jerk: 0.0}" -r 10

```

---

### ROS 应用程序接口

#### 订阅

|Topic|Type|Description|
|--|--|--|
|`/carla/<ROLE NAME>/ackermann_cmd` | [ackermann_msgs.AckermannDrive][ackermanncontrolmsg] | __Subscriber__ for steering commands |

<br>

#### 发布

|Topic|Type|Description|
|--|--|--|
| `/carla/<ROLE NAME>/ackermann_control/control_info` | [carla_ackermann_control.EgoVehicleControlInfo][egovehiclecontrolmsg] | The current values used within the controller (useful for debugging) |

[egovehiclecontrolmsg]: https://carla.readthedocs.io/en/latest/ros_msgs/#egovehiclecontrolinfomsg

<br>
