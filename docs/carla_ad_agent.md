# Carla 自动驾驶智能体

[Carla 自动驾驶智能体](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_agent) 是一种自动驾驶智能体，可以遵循给定路线，避免与其他车辆发生碰撞，并通过访问真实数据尊重交通信号灯的状态。[Carla 自动驾驶演示](carla_ad_demo.md) 使用它来提供如何使用 ROS 桥的示例。 

- [__需求__](#requirements)
- [__ROS 应用程序接口__](#ros-api)
    - [__自动驾驶智能体节点__](#ad-agent-node)
        - [参数](#parameters)
        - [订阅](#subscriptions)
        - [发布](#publications)
    - [__本地规划器节点__](#local-planner-node)
        - [参数](#parameters)
        - [发布](#subscriptions)
        - [订阅](#publications)

在内部，Carla 自动驾驶智能体使用单独的节点进行 [本地规划](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_agent/src/carla_ad_agent/local_planner.py) 。该节点已针对 `vehicle.tesla.model3` 进行了优化，因为它没有任何换档延迟。

PID 参数通过 [Ziegler-Nichols方法](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method) 采集。

---

## 需求

为了能够使用 `carla_ad_agent`，需要生成最少的传感器集（有关如何生成传感器的信息，请参阅 [Carla 生成对象](carla_spawn_objects.md) ：

- 里程计伪传感器 (`sensor.pseudo.odom`)，其角色名`odometry`附加到车辆上。 
- 一个对象伪传感器 (`sensor.pseudo.objects`)，其角色名`objects`附加到车辆上。
- 具有角色名称 `traffic_lights` 的交通灯伪传感器 (`sensor.pseudo.traffic_lights`) 。

---

## ROS 应用程序接口 

### 自动驾驶智能体节点

#### 参数

| 参数           | 类型                          | 描述                                                                                              |
|--------------|-----------------------------|-------------------------------------------------------------------------------------------------|
| `role_name`  | string (默认：`ego_vehicle`) | 自我车辆的 Carla 角色名称                                                                                |
| `avoid_risk` | bool (default: `true`)      | 如果为真，则避免与其他车辆相撞并尊重交通信号灯  |

<br>

#### 订阅

| 主题                                | 类型                                                                                                                   | 描述                                          |
|-----------------------------------|----------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| `/carla/<ROLE NAME>/target_speed` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)                                       | 自我车辆的目标速度                                   |
| `/carla/<ROLE NAME>/odometry`     | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)                                     | 自我车辆的里程计                                    |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs/CarlaEgoVehicleInfo](ros_msgs.md#carlaegovehicleinfomsg)                                                 | 识别自我车辆的 Carla 参与者 id                        |
| `/carla/<ROLE NAME>/objects`      | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | 其他参与者的信息   |
| `/carla/traffic_lights/status`    | [carla_msgs/CarlaTrafficLightStatusList](ros_msgs.md#carlatrafficlightstatuslistmsg)                                 | 获取交通信号灯的当前状态 |
| `/carla/traffic_lights/info`      | [carla_msgs/CarlaTrafficLightInfoList](ros_msgs.md#carlatrafficlightinfolistmsg)                                     | 获取有关交通灯的信息               |

<br>

#### 发布

| 主题                                 | 类型                                                                             | 描述           |
|------------------------------------|--------------------------------------------------------------------------------|--------------|
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | 目标速度 |

<br>

### 本地规划器节点

#### 参数

| 参数                  | 类型                          | 描述                                            |
|---------------------|-----------------------------|-----------------------------------------------|
| `role_name`         | string (默认值：`ego_vehicle`) | 自我车辆的 Carla 角色名称                              |
| `control_time_step` | float (默认值：`0.05`)     | 控制循环速率                             |
| `Kp_lateral`        | float (默认值： `0.9`)       | 比例项横向PID控制器      |
| `Ki_lateral`        | float (默认值： `0.0`)       | 积分项横向PID控制器          |
| `Kd_lateral`        | float (默认值： `0.0`)       | 微分项横向PID控制器        |
| `Kp_longitudinal`   | float (默认值： `0.206`)     | 比例项纵向PID控制器 |
| `Ki_longitudinal`   | float (默认值： `0.0206`)    | 积分项纵向PID控制器     |
| `Kd_longitudinal`   | float (默认值： `0.515`)     | 微分项纵向PID控制器   |

<br>

#### 订阅

| 主题                                 | 类型                                                                               | 描述                          |
|------------------------------------|----------------------------------------------------------------------------------|-----------------------------|
| `/carla/<ROLE NAME>/waypoints`     | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)         | 遵循的路线             |
| `/carla/<ROLE NAME>/odometry`      | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | 自我车辆的里程计 |
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)   | 目标速度                |

<br>

#### 发布

| 主题                                       | 类型 | 描述 |
|------------------------------------------|------|-------------|
| `/carla/<ROLE NAME>/next_target`         | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | 下一个目标姿势标记 |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs/CarlaEgoVehicleControl](ros_msgs.md#carlaegovehiclecontrolmsg) | 车辆控制指令 |

<br>
