# ROS 桥

__ROS 桥的完整文档可在 [__此处__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/) 找到。__

---

ROS 桥可实现 ROS 和 Carla 之间的双向通信。来自 Carla 服务器的信息被转换为 ROS 主题。以同样的方式，ROS 中的节点之间发送的消息被转换为要在 Carla 中应用的命令。

ROS 桥与 ROS 1 和 ROS 2 兼容。

ROS桥具有以下特点：

- 为 LIDAR、语义 LIDAR、相机（深度、分段、rgb、dvs）、GNSS、雷达和 IMU 提供传感器数据。
- 提供对象数据，例如变换、交通灯状态、可视化标记、碰撞和车道入侵。
- 通过转向、油门和制动控制自动驾驶智能体。
- 控制 Carla 仿真的各个方面，例如同步模式、播放和暂停仿真以及设置仿真参数。
