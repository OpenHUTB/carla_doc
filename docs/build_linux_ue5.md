!!! 警告
    这是一项正在进行的工作！！这个版本的 CARLA 不被认为是一个稳定的版本。在接下来的几个月里，这个分支可能会发生许多重大变化，这可能会破坏您所做的任何修改。我们建议你把这个分支当作实验性的。

# 在Linux中使用 Unreal Engine 5.3 构建

!!! 笔记
    这个构建过程是为Ubuntu 22.04实现和测试的。我们建议使用这个Ubuntu版本。

## 设置环境

本指南详细介绍了如何在Linux上使用 Unreal Engine 5.3 从源代码构建 CARLA。

在您的本地机器上克隆 Carla 的 `ue5-dev` 分支：

```sh
git clone -b ue5-dev https://github.com/carla-simulator/carla.git CarlaUE5
```

运行安装脚本：

```sh
cd CarlaUE5
bash -x Setup.sh
```

`Setup.sh`脚本安装所有必需的软件包，包括 Cmake、debian 软件包、Python软件包和 Unreal Engine 5.3。它还下载 CARLA 内容并构建 CARLA。因此，此脚本可能需要很长时间才能完成。

注意：
* 这个版本的CARLA需要 **创建虚幻引擎 5.3 的 Carla 分叉**。为了获得克隆UE存储库的权限，您需要将GitHub 帐户链接到Epic Games。如果您尚未链接您的帐户，请遵循[本指南]((https://www.unrealengine.com/en-US/ue4-on-github)) 。 
* 要使用 CARLA Unreal Engine 5 以前的版本，**请确保定义了指向 `CARLA_UNREAL_ENGINE_PATH` 环境变量定义的**绝对路径。如果未定义此变量，`Setup.sh`脚本将下载并构建 CARLA Unreal Engine 5，**这需要额外1个多小时的构建和225Gb的磁盘空间**。
* `Setup.sh`脚本检查PATH变量顶部是否安装了任何Python，否则安装 Python。要使用您自己的 Python 版本，**请确保在运行脚本之前为Python正确设置了PATH变量**。
* Carla 不能在外部磁盘上构建，Ubuntu 没有为构建提供所需的读/写/执行权限。


## 构建并运行 CARLA UE5

安装脚本本身启动以下命令，一旦修改代码并希望重新启动，则需要使用以下命令：

* 配置：

```sh
cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/LinuxToolchain.cmake \
-DLAUNCH_ARGS="-prefernvidia" -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS2=ON \
-DBUILD_CARLA_UNREAL=ON -DCARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH
```

* 构建 Carla:

```sh
cmake --build Build
```

* 构建并安装 Python API：

```sh
cmake --build Build --target carla-python-api-install
```

* 启动编辑器：

```sh
cmake --build Build --target launch
```

## 使用 CARLA UE5 构建包

```sh
cmake --build Build --target package
```

该包会在目录 `$CARLA_PATH/Build/Package` 下生成。

## 运行包

使用下列命令运行包。

```sh
./CarlaUnreal.sh
```

如果你想运行原生的 ROS2 接口，添加 `--ros2` 命令

```sh
./CarlaUnreal.sh --ros2
```

如果您想要安装与您所构建的包相对应的 Python API：

```sh
pip3 install PythonAPI/carla/dist/carla-*.whl
```