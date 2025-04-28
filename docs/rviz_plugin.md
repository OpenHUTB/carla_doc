# RVIZ Carla 插件

[RVIZ 插件](https://github.com/carla-simulator/ros-bridge/tree/master/rviz_carla_plugin) 提供了基于 [RVIZ](https://wiki.ros.org/rviz) ROS包的可视化工具。

- [__使用 RVIZ 运行 ROS 桥__](#run-ros-bridge-with-rviz)
- [__RVIZ 插件的功能__](#features-of-the-rviz-plugin)
- [__ROS 应用程序接口__](#ros-api)
    - [订阅](#subscriptions)
    - [发布](#publications)
    - [服务](#services)

---

## 使用 RVIZ 运行 ROS 桥

![ros_rviz](img/ros_rviz.png)

RVIZ 插件需要一个名为 `ego_vehicle` 的自主车辆。要查看 ROS 桥与 RVIZ 配合使用的示例，请在运行的 Carla 服务器上执行以下命令：

__1.__ 在启用 RVIZ 的情况下启动 ROS 桥：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

__2.__ 启动 RVIZ：

```sh
# ROS 1
rosrun rviz rviz

# ROS 2
ros2 run rviz2 rviz2
```

__2.__ 使用`carla_spawn_objects`包生成一辆自主车辆：

```sh
# ROS 1
roslaunch carla_spawn_objects carla_spawn_objects.launch

# ROS 2
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
```

__3.__ 使用 `carla_manual_control` 包控制自主车辆（按下B可启用手动转向）：

```sh
# ROS 1
roslaunch carla_manual_control carla_manual_control.launch

# ROS 2
ros2 launch carla_manual_control carla_manual_control.launch.py
```

---

## RVIZ 插件的功能

- __自主车辆状态可视化__ - 可视化车辆位置和控制。
- __向其他节点提供 RVIZ 视图姿势__ - 连接`actor.pseudo.control` 到相机后，通过发布姿势消息在 Carla 世界中移动相机。
- __传感器可视化__ - 可视化 RGB、LIDAR、深度、DVS 和语义分割相机信息。
- __执行场景__ - 使用 [carla_ros_scenario_runner](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_scenario_runner) 包触发场景。
- __播放/暂停模拟__ - 如果以同步模式启动，您可以播放和暂停模拟。
- __手动覆盖自主车辆控制__ - 使用 [RVIZ Visualization Tutorials](https://github.com/ros-visualization/visualization_tutorials) 可视化教程中的驱动小部件和从扭曲转换为车辆控制的 [节点](https://github.com/carla-simulator/ros-bridge/blob/master/carla_twist_to_control) ，通过鼠标驾驶车辆。 

---

## ROS 应用程序接口

#### 订阅

| Topic | Type | Description                                                     |
|-------|------|-----------------------------------------------------------------|
| `/carla/status` | [carla_msgs/CarlaStatus](ros_msgs.md#carlastatusmsg) | 读取 Carla 的当前状态                                                  |
| `/carla/ego_vehicle/vehicle_status` | [carla_msgs/CarlaEgoVehicleStatus](ros_msgs.md#carlaegovehiclestatusmsg) | 显示本车当前状态                    |
| `/carla/ego_vehicle/odometry` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | 显示自主车辆的当前姿态                     |
| `/scenario_runner/status` | [carla_ros_scenario_runner_types/CarlaScenarioRunnerStatus](ros_msgs.md#carlascenariorunnerstatusmsg) | 可视化场景运行状态                            |
| `/carla/available_scenarios` | [carla_ros_scenario_runner_types/CarlaScenarioList](ros_msgs.md#carlascenariolistmsg) | 提供要执行的场景列表（在组合框中禁用） |

<br>

#### 发布

| Topic | Type | Description                                      |
|-------|------|--------------------------------------------------|
| `/carla/control` | [carla_msgs/CarlaControl](ros_msgs.md#carlacontrolmsg) | 播放/暂停/步进 Carla                                   |
| `/carla/ego_vehicle/spectator_pose` | [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) | 发布 RVIZ 相机视图的当前姿态 |
| `/carla/ego_vehicle/vehicle_control_manual_override` | [std_msgs/Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html) | 启用/禁用车辆控制覆盖          |
| `/carla/ego_vehicle/twist` | [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | 通过鼠标创建的扭曲命令             |

<br>

#### 服务

| Topic | Type | Description |
|-------|------|-------------|
| `/scenario_runner/execute_scenario` | [carla_ros_scenario_runner_types/ExecuteScenario](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) | 执行选定的场景 |

<br>
