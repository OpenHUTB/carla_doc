
# 智能自动驾驶系统

智能自动驾驶系统（ISS）是一个用Python和C++编写的模块化框架，旨在构建一个适合研究的可扩展工作空间。该框架将包含用于自动驾驶相关任务的传统和深度学习算法，例如感知、定位、映射、预测、规划和控制。对外部库依赖性最小的模块化设计可以为研究人员评估自动驾驶系统的算法提供透明、干净的工作空间。

ISS 的代码可以从其 [*Github 仓库*](https://github.com/CAS-LRJ/ISS)  下载，其中可以找到详细的安装说明。

## 先决条件

ISS 使用 Ubuntu 20.04 进行测试，运行 Python 3.8。

对于那些有兴趣将 ISS 与 Gazebo 和 [ROS-Noetic](https://wiki.ros.org/noetic/Installation) 结合使用的人，您应该按照官方 ROS 文档中提供的说明安装ROS-Noetic 。

要将 ISS 与 Carla 结合使用，必须安装 Carla。我们建议使用版本 0.9.13，可以在 [Carla](https://github.com/carla-simulator/carla/releases) GitHub 发布页面找到该版本。

## 安装

- 安装 [ROS Noetic](http://wiki.ros.org/noetic/Installation), 以及下载相关依赖:

  ```
  sudo apt-get install ros-noetic-navigation ros-noetic-gmapping ros-noetic-teb-local-planner ros-noetic-ackermann-msgs ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-publisher-gui ros-noetic-ros-control ros-noetic-ros-controllers
  ```


- 安装 [CARLA 0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13/) 并设置环境变量:

  ```
  export CARLA_ROOT=</path/to/carla>
  export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla

  注意
  运行代码时需要 agents 模块， 在 conda 环境 ISS 目录下，还需要配置环境变量
  export PYTHONPATH=$PYTHONPATH:/home/.../carla-0.9.13/carla:/home/.../carla-0.9.13/PythonAPI/carla/agents
  ```


- 创建虚拟环境

  ```
  conda create -n iss python=3.8
  conda activate iss
  注意
  若要退出当前conda环境，执行
  conda deactivate
  ```


- 安装 [git-lfs](https://git-lfs.github.com/)

- 递归克隆项目及其子模块:

  ```
  git clone --recurse-submodules https://github.com/CAS-LRJ/ISS.git && cd ISS
  pip3 install -r Install/requirements.txt
  python3 Install/setup.py develop

  注意：这里必须递归克隆，将子模块一并克隆下来，查看子模块内容是否克隆成功，否则运行报错。
  ```


-   安装 PyTorch 和 torch-scatter:

  ```
  pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.htmlpip3 install Install/torch_scatter-2.0.7-cp38-cp38-linux_x86_64.whl
  ```


- 安装 [CUDA Toolkit 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive)
- 安装 [MMDetection](https://mmdetection.readthedocs.io/en/latest/get_started.html)

## 构建
- 要使用 ROS-Noetic, 首先构建ROS工作空间。

  ```
  cd ros1_ws && catkin build
  source devel/setup.bash # or setup.zsh

  注意：会生成devel目录！
  ```

## 运行任务

- 打开 CARLA 服务器

  ```
  cd carla-0.9.13
  ./CarlaUE4.sh
  ```


- 启动 ros 节点，为智能驾驶车辆作全局路径规划

  ```
  roslaunch carla_bridge carla_demo.launch
  ```

注意：

```
运行过程中要求安装其他包，除CARLA client API 为0.9.13，其他不做要求
```