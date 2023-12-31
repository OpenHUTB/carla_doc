# [机器人操作系统](https://carla.readthedocs.io/en/latest/ecosys_ros/) 

![ros_carla](./img/ros_carla.png)

[__机器人操作系统（Robotic Operating System, ROS）__](https://www.ros.org/) 是一组用于机器人和自动驾驶应用程序的软件库。Carla 可以通过其 ROS 接口直接连接到 ROS，控制信号可以发送到 Carla 参与者，并且可以通过 ROS 主题访问传感器数据。

连接 Carla 和 ROS 有两种选择。

- __Carla 原生接口__: 直接构建到 Carla 服务器中的 ROS 接口
- __ROS 桥__: 一个单独的库，用于在 ROS 和 Carla 之间传输信号

## CARLA 的原生 ROS 接口

这是推荐的接口，因为它提供最佳性能和最低延迟。目前本机接口仅支持 ROS 2。如果您使用 ROS 1，则必须使用 ROS 桥。

## CARLA ROS 桥

[__CARLA ROS 桥__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/) 是一个用于将 ROS 连接到 CARLA 的库，它与 ROS 1 和 ROS 2 兼容。由于 CARLA ROS 桥是一个单独的包，因此与本机接口相比，存在额外的延迟。ROS 桥仍然提供支持 ROS 1 和 ROS 2 的旧实现。

