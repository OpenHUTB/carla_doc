# Carla 手动控制

[CARLA 手动控制包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control) 是与 Carla 一起打包的 [`manual_control.py`][manualcontrol] 脚本的仅 ROS 版本。所有数据均通过 ROS 主题接收。 

[manualcontrol]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/manual_control.py

- [__需求__](#requirements)
- [__运行包__](#run_the_package)
---

## 需求 <span id="requirements"></span>

为了能够使用`carla_manual_control`，需要将一些特定的传感器连接到自主车辆（有关如何将传感器连接到车辆的信息，请参阅 [Carla 生成对象](carla_spawn_objects.md) 对象）： 

- __to display an image__: 参与者名为`rgb_view`和分辨率为 800x600 的相机。
- __to display the current position__: 具有参与者名称 `gnss` 的全球导航卫星系统传感器和具有参与者名`odometry`的里程计伪传感器。
- __to get a notification on lane invasions__: 具有参与者名`lane_invasion`的压线传感器。
- __to get a notification on collisons__: 具有参与者名`collision`的碰撞传感器。

---

## 运行包 <span id="run_the_package"></span>

要运行包：
 
__1.__ 确保 Carla 正在运行。启动 ROS 桥：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

__2.__ 生成对象：

```sh
# ROS 1
roslaunch carla_spawn_objects carla_spawn_objects.launch

# ROS 2
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py
```

__3.__ 启动 `carla_manual_control` 节点：

```sh
# ROS 1
roslaunch carla_manual_control carla_manual_control.launch

# ROS 2
ros2 launch carla_manual_control carla_manual_control.launch.py
```

__4.__ 要手动驾驶车辆，请按“B”。按“H”查看说明。

或者，将上述所有命令组合成一个单独的启动文件，并可以通过执行以下命令同时运行：

```sh
# ROS 1
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

# ROS 2
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```
---
