# ROS 2的ROS桥安装

本节是关于如何在 Linux 上安装 ROS 桥以与 ROS 2 一起使用的指南。您将找到先决条件、安装步骤、如何运行基本包以确保一切正常工作以及运行测试的命令。

- [__开始之前__](#before-you-begin)
- [__ROS桥安装__](#ros-bridge-installation)
- [__运行ROS桥__](#run-the-ros-bridge)
- [__测试__](#testing)

!!! 重要
    适用于Windows 的 ROS仍在[实验](http://wiki.ros.org/noetic/Installation) 。它只在Linux系统上进行过测试。 

---

## 开始之前

在使用ROS桥之前，您需要满足以下软件要求：

- 安装 ROS:
    - [__ROS 2 Foxy__](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) — 针对 Ubuntu 20.04 (Focal)
-根据您的需要，可能需要额外的 ROS 包。强烈建议是用 [rviz](https://wiki.ros.org/rviz) 将ROS数据可视化。
- CARLA 0.9.11或更高版本——以前的版本与ROS桥不兼容。遵循[快速启动安装](https://carla.readthedocs.io/en/latest/start_quickstart/) 或者为 [Linux构建](https://carla.readthedocs.io/en/latest/build_linux/) 。建议尽可能将ROS桥接版本与CARLA版本相匹配。

---

## ROS桥安装

!!! 注意
    Debian软件包安装还不能用于ROS 2。

__1.__ 设置项目目录并克隆ROS网桥存储库和子模块：

```sh
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
```

__2.__ 设置 ROS 环境：

```sh
source /opt/ros/foxy/setup.bash
```

__3.__ 安装 ROS 依赖：

```sh
rosdep update
rosdep install --from-paths src --ignore-src -r
```

__4.__ 使用colcon构建ROS桥的工作空间：

```sh
colcon build
```

---

## 运行ROS桥

__1.__ 请根据安装 CARLA 时使用的安装方法启动 CARLA 服务器：

```sh
# 在 Carla的根目录中运行打包好的版本
./CarlaUE4.sh

# 在`opt/carla-simulator/`的 Debian 安装 
./CarlaUE4.sh

# 在 Carla 的根目录中从源代码构建
make launch
```

__2.__ Add the correct CARLA modules to your Python path:

```sh
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
```
__3.__ Add the source path for the ROS bridge workspace:

```sh
    source ./install/setup.bash
```

__4.__ In another terminal, start the ROS 2 bridge. You can run one of the two options below:

```sh
    # Option 1, start the basic ROS bridge package
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

    # Option 2, start the ROS bridge with an example ego vehicle
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

!!! Note

    If you receive the error: `ImportError: no module named CARLA` then the path to the CARLA Python API is missing. The apt installation sets the path automatically, but it may be missing for other installations.

    You will need to add the appropriate `.egg` file to your Python path. You will find the file in either `/PythonAPI/` or `/PythonAPI/dist/` depending on the CARLA installation. Execute the following command with the complete path to the `.egg` file, using the file that corresponds to your installed version of Python:

    `export PYTHONPATH=$PYTHONPATH:path/to/carla/PythonAPI/<your_egg_file>`

    It is recommended to set this variable permanently by adding the previous line to your `.bashrc` file.

    To check the CARLA library can be imported correctly, run the following command and wait for a success message:

            python3 -c 'import carla;print("Success")' # python3

            or

            python -c 'import carla;print("Success")' # python2

---

## 测试

To execute tests using colcon:

__1.__ Build the package:

```sh
    colcon build --packages-up-to carla_ros_bridge
```

__2.__ Run the tests:

```sh
    launch_test carla_ros_bridge/test/ros_bridge_client_ros2_test.py
```

---

## Windows下安装
根据[链接](./ros/windows-install_binary.md) 安装ROS2。


