# Carla 路径点发布器

[Carla 路径点发布器](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher) 使路径点计算可用于 ROS 上下文，并提供查询 Carla 路径点的服务。要了解有关路径点的更多信息，请参阅 Carla [文档](https://carla.readthedocs.io/en/latest/core_map/#navigation-in-carla) 。

- [__运行路径点发布器__](#run-the-waypoint-publisher)
    - [设定目标](#set-a-goal)
- [__使用路径点发布器__](#using-the-waypoint-publisher)
- [__ROS 应用程序接口__](#ros-api)
    - [发布](#publications)
    - [服务](#services)

---

## 运行路径点发布器

在 Carla 服务器运行的情况下，执行以下命令：

```sh
# ROS 1
roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch

# ROS 2
ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
```

### 设定目标

如果可用，则从主题 `/carla/<ROLE NAME>/goal` 中读取目标，否则使用固定的生成点。

设置目标的首选方法是单击 RVIZ 中的“2D Nav Goal”。

![rviz_set_goal](img/rviz_set_start_goal.png)

---

### 使用路径点发布器


[Carla 自动驾驶演示](carla_ad_demo.md) 使用 Waypoint Publisher 为[Carla 自动驾驶代理](carla_ad_agent.md) 规划路线。有关如何使用它的示例， 请参阅 Carla 自动驾驶演示 [启动文件](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch) 。

---

## ROS 应用程序接口

#### 发布

| 主题                                    | 类型                                                                       | 描述                             |
|---------------------------------------|--------------------------------------------------------------------------|--------------------------------|
| `/carla/<ego vehicle name>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) | 发布计算出的路线 |

<br>

#### 服务

| 服务                                                                | 类型                                                                                                                                             | 描述              |
|-------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|-----------------|
| `/carla_waypoint_publisher/<ego vehicle name>/get_waypoint`       | [carla_waypoint_types/GetWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetWaypoint.srv)           | 获取特定位置的路径点      |
| `/carla_waypoint_publisher/<ego vehicle name>/get_actor_waypoint` | [carla_waypoint_types/GetActorWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetActorWaypoint.srv) | 获取参与者 ID 的路径点 |

<br>
