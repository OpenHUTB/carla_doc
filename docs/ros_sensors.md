# ROS 桥传感器

---

## 可用传感器

###### RGB 相机

| 主题                                                           | 类型 |
|--------------------------------------------------------------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image`       | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### 深度相机

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### 语义分割相机

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
|  `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### DVS 相机

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/events` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### 激光雷达

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### 语义激光雷达

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### 雷达

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### 惯性测量单元

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) |

###### 全球导航卫星系统

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) |

###### 碰撞传感器

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaCollisionEvent](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaCollisionEvent.msg) |

###### 压线传感器

| 主题 | 类型 |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaLaneInvasionEvent](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaLaneInvasionEvent.msg) |

伪传感器

###### 变换（TF）传感器



当生成此伪传感器时，将发布自我车辆的转换数据。

注意：传感器在测量完成后发布转换数据。child_frame_id 与传感器主题的前缀相对应。

###### 里程计传感器

| 主题 | 类型 | 描述                                        |
|-------|------|-------------------------------------------|
| `/carla/<PARENT ROLE NAME>/<SENSOR ROLE NAME>` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | 父参与者的里程计。 |

###### 车速表传感器

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/<PARENT ROLE NAME>/<SENSOR ROLE NAME>` | [std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html) | 父参与者的速度。单位：米/秒。 |

###### 地图传感器

| 主题 | 类型 | 描述                                                                                      |
|-------|------|-----------------------------------------------------------------------------------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html) | 以锁定主题上的字符串形式提供 OpenDRIVE 映射。  |

###### 目标传感器

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | 发布所有车辆和行人。如果附加到父级，则不包含父级。 |

###### 标记传感器

| 主题 | 类型 | 描述           |
|-------|------|--------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [visualization_msgs/Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | 车辆和行人的可视化 |

###### 交通灯传感器

| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/status` | [carla_msgs/CarlaTrafficLightStatusList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaTrafficLightStatusList.msg) | 所有交通灯及其状态的列表。 |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/info` | [carla_msgs/CarlaTrafficLightInfoList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaTrafficLightInfoList.msg) | 所有交通灯的静态信息（例如位置）。 |

###### 参与者列表传感器

| 主题 | 类型 | 描述                                          |
|-------|------|---------------------------------------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaActorList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaActorList.msg) | 所有 Carla 参与者的列表。 |

###### 参与者控制传感器

该伪传感器允许通过在 Pose 和 Twist 数据类型中发布姿势和速度来控制其所连接的参与者（例如 ego_vehicle）的位置和速度。注意：此控制方法不考虑车辆限制。它允许现实世界中不可能的运动，例如飞行或旋转。目前，该传感器应用完整的线性矢量，但仅应用角度矢量的偏航。


| 主题 | 类型 | 描述 |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/set_transform` | [geometry_msgs/Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html) | 转换以应用于传感器的父级。 |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/set_target_velocity` | [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | 应用于传感器父级的速度（角度和线性）。 |
