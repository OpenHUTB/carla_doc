# ROS 兼容性节点

[ROS 兼容性节点](https://github.com/carla-simulator/ros-bridge/tree/master/ros_compatibility) 是一个接口，允许软件包与 ROS 1 和 ROS 2 无缝使用。根据环境变量`ROS_VERSION`，相同的 API 将调用 ROS 1 或 ROS 2 函数。它通过创建继承`CompatibleNode`创建类。

---

## ROS 参数

默认情况下，在 ROS 2 中设置或访问参数之前需要声明参数。ROS 1 中并非如此。为了保持 ROS 1 和 ROS 2 模式以相似的方式工作，在 ROS 2 的`CompatibleNode`版本中将参数 `allow_undeclared_parameters` 设置为`True`，允许使用参数 `TrueCompatibleNode` 而无需事先声明。

---

## 服务

在 ROS 2 中，服务可以异步调用。ROS 1 中并非如此。因此，ROS 2 版本的`call_service()`方法在异步调用后会等待服务器的响应，以仿真 ROS 1 的同步行为。

!!! 笔记
    在等待响应时，ROS 2 的`call_service()`方法会旋转节点。如果另一个线程并行旋转同一节点，这可能会导致问题（错误或死锁）。
