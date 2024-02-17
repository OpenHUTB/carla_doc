
# 智能自动驾驶系统

智能自动驾驶系统（ISS）是一个用Python和C++编写的模块化框架，旨在构建一个适合研究的可扩展工作空间。该框架将包含用于自动驾驶相关任务的传统和深度学习算法，例如感知、定位、映射、预测、规划和控制。对外部库依赖性最小的模块化设计可以为研究人员评估自动驾驶系统的算法提供透明、干净的工作空间。

ISS 的代码可以从其 [*Github 仓库*](https://github.com/CAS-LRJ/ISS)  下载，其中可以找到详细的安装说明。

## 先决条件

ISS 使用 Ubuntu 20.04 进行测试，运行 Python 3.8。

对于那些有兴趣将 ISS 与 Gazebo 和 [ROS-Noetic](https://wiki.ros.org/noetic/Installation) 结合使用的人，您应该按照官方 ROS 文档中提供的说明安装ROS-Noetic 。

要将 ISS 与 Carla 结合使用，必须安装 Carla。我们建议使用版本 0.9.13，可以在 [Carla](https://github.com/carla-simulator/carla/releases) GitHub 发布页面找到该版本。


## 开发版
要安装 ISS 的开发版本，请按照以下说明进行操作。
```
git clone https://github.com/CAS-LRJ/ISS.git
cd ISS && python3 setup.py develop
```

## 构建
要构建 ISS 的 ROS 包，请按照以下说明进行操作。
```
cd ISS/ros1_ws && catkin build
source devel/setup.bash
```

## 运行任务
如果使用 Gazebo，只需执行以下操作：
```
roslaunch robot_gazebo gazebo_demo.launch
```

如果使用 Carla，请先启动 Carla，然后执行以下操作：
```
roslaunch carla_bridge carla_demo.launch
```