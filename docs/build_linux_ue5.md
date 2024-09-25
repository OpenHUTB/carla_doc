!!! 警告
    这是一项正在进行的工作！！这个版本的 Carla 不被认为是一个稳定的版本。在接下来的几个月里，这个分支可能会发生许多重大变化，这可能会破坏您所做的任何修改。我们建议你把这个分支当作实验性的。

# 在Linux中使用 Unreal Engine 5.3 构建

!!! 笔记
    这个构建过程是为 Ubuntu 22.04 实现和测试的。我们建议使用这个 Ubuntu 版本。

## 设置环境

本指南详细介绍了如何在 Linux 上使用 Unreal Engine 5.3 从源代码构建 Carla。

在您的本地机器上克隆 Carla 的 `ue5-dev` 分支：

```sh
git clone -b ue5-dev https://github.com/carla-simulator/carla.git CarlaUE5
```

运行安装脚本：

```sh
cd CarlaUE5
bash -x Setup.sh
```

`Setup.sh`脚本安装所有必需的软件包，包括 Cmake、debian 软件包、Python软件包和 Unreal Engine 5.3。它还下载 Carla 内容并构建 Carla。因此，此脚本可能需要很长时间才能完成。

注意：
* 这个版本的 Carla 需要 **创建虚幻引擎 5.3 的 Carla 分叉**。为了获得克隆UE存储库的权限，您需要将GitHub 帐户链接到Epic Games。如果您尚未链接您的帐户，请遵循[本指南]((https://www.unrealengine.com/en-US/ue4-on-github)) 。 

* 要使用 Carla Unreal Engine 5 以前的版本，**请确保定义了指向 `CARLA_UNREAL_ENGINE_PATH` 环境变量定义的**绝对路径。如果未定义此变量，`Setup.sh`脚本将下载并构建 Carla Unreal Engine 5，**这需要额外1个多小时的构建和225Gb的磁盘空间**。

* `Setup.sh`脚本检查 PATH 变量顶部是否安装了任何Python，否则安装 Python。要使用您自己的 Python 版本，**请确保在运行脚本之前为 Python 正确设置了 PATH 变量**。

* Carla 不能在外部磁盘上构建，Ubuntu 没有为构建提供所需的读/写/执行权限。


## 构建并运行 Carla UE5

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


## 问题

### 系统安装
如果引导不了系统，需要将在BIOS中设置为UEFI启动。

Ubuntu 20.04 没有g++-12的源，只有g++-9的源。而Ubuntu 22.04 有g++-12的源。

解决 [ubuntu 安装过程中 安装界面黑屏](https://blog.csdn.net/qq_16963597/article/details/94715979) 的问题：
u盘启动过程中，会出现选择界面，try ubuntu 、install ubuntu等，此时点“e”键，会出现一个黑框，此时在倒数第二行左右，找到
```shell
quiet splash - - - 改成 quiet splash acpi=off
```
acpi=off是关闭高级电源管理接口。如果必须添加acpi = off使ubuntu成功启动，则表示计算机上的ACPI与该版本的ubuntu不兼容。

再次启动时，仍然会黑屏，进不去。
需要[禁用nouveau](https://blog.51cto.com/u_15075507/4517000) ，然后再进入系统中安装显卡驱动。
```shell
vi /etc/modprobe.d/blacklist.conf
```
最后一行加上`blacklist nouveau`。
使之生效：
```shell
update-initramfs -u
```
然后输入`reboot`重启。

### 编译
```shell
bin/clang" is not able to compile a simple test program.
```
```shell
clang++ --version
```
确定clang没有安装，执行：
```shell
sudo apt-get install clang
```
还是找不到！！！




### Carla
切换到`ue5-dev`分支：
```shell
git checkout -b ue5-dev origin/ue5-dev
```

`bash -x`中的参数`-x`会列出shell指定语句的顺序并将其打印。


* 报错：`error: externally-managed-environment`

> 原因：表示当前Python环境是由系统外部管理的，通常在某些Linux发行版中（尤其是Debian和基于Debian的系统，比如Ubuntu），系统会强烈建议不要直接使用pip来安装包，以避免与系统包管理器（如apt）的潜在冲突。
> 
> 解决：新建Python虚拟环境。

* cmake时候报错：`cannot find /lib64/ld-linux-x86-64.so.2`
