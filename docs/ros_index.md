# ROS 桥文档

这是 ROS 桥的文档，它实现了 ROS 和 Carla 之间的双向通信。来自 Carla 服务器的信息被转换为ROS主题。以同样的方式，ROS中节点之间发送的消息被转换为 Carla 中应用的命令。 

ROS桥与ROS1和ROS2都是相容的。

ROS桥具有以下特点：

- 为激光雷达、语义激光雷达、相机（深度、分割、rgb、dvs）、全球导航卫星系统、雷达和IMU提供传感器数据。
- 提供对象数据，如变换、交通信号灯状态、可视化标记、碰撞和压线。
- 通过转向、油门和制动器控制自动驾驶代理。
- 控制 Carla 模拟的各个方面，如同步模式、播放和暂停模拟以及设置模拟参数。

---

## 开始

- [__为ROS 1安装ROS桥__](ros_installation_ros1.md)
- [__为ROS 2安装ROS桥__](ros_installation_ros2.md)

---

## 了解主要的 ROS 桥接包

- [__CARLA 的 ROS 桥__](run_ros.md) - 运行ROS桥所需的主程序包
- [__ROS 兼容性节点__](ros_compatibility.md) - 允许同一 API 调用 ROS 1 或 ROS 2 函数的接口

---

## 了解其他ROS桥接包

- [__Carla 生成对象__](carla_spawn_objects.md) - 提供生成参与者的通用方法
- [__Carla 手动控制__](carla_manual_control.md)- 用于自主车辆的基于 ROS 的可视化和控制工具（类似于 Carla 提供的`carla_manual_control.py`）  
- [__Carla 阿克曼控制__](carla_ackermann_control.md) - 将阿克曼命令转换为`转向/油门/刹车`的控制器
- [__Carla 路径点发布器__](carla_waypoint.md) - 发布和查询 Carla 路径点
- [__Carla 自动驾驶代理__](carla_ad_agent.md) - 遵循路线、避免碰撞并遵守交通信号灯的示例代理
- [__Carla 自动驾驶示例__](carla_ad_demo.md) - 一个示例包，提供了使用自动驾驶车辆启动Carla ROS环境所需的一切
- [__Carla ROS Scenario Runner__](carla_ros_scenario_runner.md) - 通过 ROS 与 Carla Scenario Runner 一起执行 OpenScenarios 的包装器
- [__Carla 扭曲控制__](carla_twist_to_control.md) - 将扭曲控制转换为 Carla 车辆控制
- [__RVIZ plugin__](rviz_plugin.md) - 可视化/控制 Carla 的RVIZ插件
- [__RQT 插件__](rqt_plugin.md) - 控制 Carla 的RQT插件
- [__PCL 记录器__](pcl_recorder.md) - 根据从模拟中捕获的数据创建点云地图

---

## 探索参考资料

- [__ROS 传感器__](ros_sensors.md) - 不同传感器中提供的参考主题
- [__ROS messages__](ros_msgs.md) - Carla ROS消息中提供的参考参数
