# 点云地图创建

[PCL记录器](https://github.com/carla-simulator/ros-bridge/tree/master/pcl_recorder) 包允许您从 Carla 地图创建点云地图。

---

## 在你开始之前

安装 `pcl-tools` 库：

```sh
sudo apt install pcl-tools
```

---

## 使用 PCL 记录器

PCL 记录器包将生成一辆自主车辆，可以通过键盘或 Carla PythonAPI 中的自动驾驶功能进行控制。

__1.__ 启动 Carla 服务器后，在新终端中运行以下命令来启动 PCL 记录器包：

```sh
# ROS 1
roslaunch pcl_recorder pcl_recorder.launch

# ROS 2
ros2 launch pcl_recorder pcl_recorder.launch.py
```
__2.__ 捕获驱动完成后，减小点云的整体大小：

```
# 创建一个点云文件
pcl_concatenate_points_pcd /tmp/pcl_capture/*.pcd

# Filter duplicates
pcl_voxel_grid -leaf 0.1,0.1,0.1 output.pcd map.pcd
```

__3.__ 验证结果：

```sh
pcl_viewer map.pcd
```
