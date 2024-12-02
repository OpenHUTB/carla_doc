# Linux 系统上的构建

本指南详细介绍了如何在 Linux 上从源代码构建 Carla。有两个部分。第一部分详细介绍系统要求和所需软件的安装，第二部分详细介绍如何实际构建和运行 Carla。

构建过程很长（4小时或更长时间）并且涉及多种软件。强烈建议在开始之前完整阅读该指南。

- [__第一部分：先决条件__](#part-one-prerequisites)
    - [系统要求](#system-requirements)
    - [软件要求](#software-requirements)
        - [虚幻引擎](#unreal-engine)
- [__第二部分：构建 Carla__](#part-two-build-carla)
    - [克隆 Carla 仓库](#clone-the-carla-repository)
    - [获取资产](#get-assets)
    - [设置虚幻引擎环境变量](#set-unreal-engine-environment-variable)
    - [构建 Carla](#build-carla)
    - [其他 make 命令](#other-make-commands)
    - [其他](#other)

---
## 第一部分：先决条件 <span id="part-one-prerequisites"></span>

### 系统要求 <span id="system-requirements"></span>

* __Ubuntu 18.04.__ Carla 提供对 Ubuntu 16.04 之前版本的支持。**然而**，虚幻引擎需要适当的编译器才能正常工作。下面单独列出了 Ubuntu 18.04 及之前版本的依赖项。确保安装与您的系统相对应的软件。
* __130 GB 磁盘空间。__ Carla 将占用大约 31 GB，虚幻引擎将占用大约 91 GB，因此需要大约 130 GB 的可用空间来容纳这两个空间以及额外的小型软件安装。
* __足够的 GPU。__ Carla 的目标是进行真实的模拟，因此服务器至少需要 6 GB GPU，但建议使用 8 GB。强烈建议使用专用 GPU 进行机器学习。 
* __两个 TCP 端口和良好的互联网连接。__ 默认为 2000 年和 2001 年。确保这些端口未被防火墙或任何其他应用程序阻止。

!!! 警告
    __如果您要从 Carla 0.9.12 升级到 0.9.13__: 您必须首先将虚幻引擎 4 的 Carla 分支升级到最新版本。有关升级虚幻引擎 4 的详细信息，请参阅 [__虚幻引擎__](#unreal-engine) 部分。



### 软件要求 <span id="software-requirements"></span>

Carla 需要运行多种不同类型的软件。有些是在 Carla 构建过程本身期间构建的，例如*Boost.Python*。其他是应该在开始构建之前安装的二进制文件（*cmake*、*clang*、不同版本的*Python*等）。要安装这些要求，请运行以下命令：

```sh
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
sudo apt-get update
```

!!! 警告
    以下命令取决于您的 Ubuntu 版本。请务必做出相应的选择。

__Ubuntu 24.04__
```shell
# 显卡驱动使用nvidia-driver-470-server（转有）
# 已经安装了clang-14
# sudo apt install g++-10
# sudo apt-get install clang++-10

# 源代码编译安装 llvm-10
sudo apt install gcc-multilib
git clone --branch release/10.x --recursive https://github.com/llvm/llvm-project
cd llvm-project/llvm
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DLLVM_ENABLE_PROJECTS=lld -DCMAKE_INSTALL_PREFIX=/usr/local/llvm-10 ..
make -j8
sudo make install
# /usr/local/llvm-10/bin
# 编译clang-10（默认LLVM编译完没有clang）
# 回到llvm-project目录
cd llvm-project/clang/
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/clang-10 ..
make -j8
sudo make install
# 创建软链接
sudo ln -s /usr/local/clang-10/bin/clang++ /usr/bin/clang++
sudo ln -s /usr/local/clang-10/bin/clang /usr/bin/clang
# 检查版本
clang --version

# 源代码编译安装 gcc-7
wget http://ftp.gnu.org/gnu/gcc/gcc-7.5.0/gcc-7.5.0.tar.gz
tar -zxvf gcc-7.5.0.tar.gz
cd gcc-7.5.0
mkdir build
cd build
../configure --prefix=/usr/local/gcc-7  # 开始配置安装
# 如果安装缺少的库，则运行下面这一条命令
sudo apt-get install libgmp-dev libmpfr-dev libmpc-dev gcc-multilib g++-multilib
# 再次使用下面命令构建安装环境
../configure --enable-multilib
# 构建完成后得到一个Makefile文件。开始编译
make -j8  # -j选项后的数字取决于机器的逻辑处理器数量
sudo make install
# 创建软链接
sudo ln -s /usr/local/clang-10/bin/clang++ /usr/bin/g++


sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-14 180
sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-14 180
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 180
```

!!! 警告
    构建时对`Util/BuildTools/Setup.sh`所做的修改：1. 构建boost时，执行`./bootstrap.sh`的命令中去除了`--with-toolset=clang`选项。2. 对rcplib执行cmake时，去除了`-stdlib=libc++`选项。

__Ubuntu 22.04__
```shell
sudo apt-add-repository "deb http://archive.ubuntu.com/ubuntu focal main universe"
sudo apt-get update
sudo apt-get install build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python3 python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git git-lfs
# update-alternatives的第一个参数--install表示向update-alternatives注册服务名。
# 第二个参数是注册最终地址，成功后将会把命令在这个固定的目的地址做真实命令的软链，以后管理就是管理这个软链；
# 第三个参数：服务名，以后管理时以它为关联依据。
# 第四个参数，被管理的命令绝对路径。
# 第五个参数，优先级，数字越大优先级越高。
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180 &&
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 180
```

__Ubuntu 20.04__
```shell
sudo apt-add-repository "deb http://apt.llvm.org/focal/ llvm-toolchain-focal main"
sudo apt-get update
sudo apt-get install build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180
```


__Ubuntu 18.04__.

```sh
sudo apt-add-repository "deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic main"
sudo apt-get update
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

__Ubuntu 16.04__.

```sh
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
sudo apt-get update
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng16-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

__以前的 Ubuntu__ 版本。

```sh
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng16-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
```

__所有 Ubuntu 系统__。

为了避免虚幻引擎和 Carla 依赖项之间的兼容性问题，请使用相同的编译器版本和 C++ 运行时库来编译所有内容。Carla 团队使用 clang-8 和 LLVM 的 libc++。更改默认的 clang 版本以编译虚幻引擎和 Carla 依赖项。

```sh
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

从 Carla 0.9.12 开始，用户可以选择使用`pip`或`pip3`安装 Carla Python API。需要 20.3 或更高版本。要检查您是否有合适的版本，请运行以下命令：

```sh
# 对于 Python 3
pip3 -V

# 对于 Python 2
pip -V
```

如果您需要升级：

```sh
# 对于 Python 3
pip3 install --upgrade pip

# 对于 Python 2
pip install --upgrade pip
```

您必须安装以下 Python 依赖项：

```sh
pip install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip install --user distro &&
pip3 install --user distro &&
pip install --user wheel &&
pip3 install --user wheel auditwheel
```

---

#### 虚幻引擎 <span id="unreal-engine"></span>

从版本 0.9.12 开始，Carla 使用虚幻引擎 4.26 的修改版。该分支包含 Carla 特有的补丁。

__1.__ 将 Carla 的虚幻引擎 4.26 分支的内容克隆到本地计算机：

```sh
git clone https://github.com/OpenHUTB/UnrealEngine.git ~/UnrealEngine_4.26
```

__2.__ 导航到克隆存储库的目录：
```sh
cd ~/UnrealEngine_4.26
```

__3.__ 进行构建。这可能需要一两个小时，具体取决于您的系统。
```sh
./Setup.sh && ./GenerateProjectFiles.sh && make
```
如果出现
```text
bash: ./Setup.sh: Permission denied
./Setup.sh: line 40: Engine/Build/BatchFiles/Linux/GitDependencies.sh: Permission denied
Engine/Build/BatchFiles/Linux/GitDependencies.sh: line 25: mono: command not found

Setting up Mono
Corlib not in sync with this runtime: expected corlib version 149, found 1051600014.
```

```shell
chmod +x Setup.sh
chmod +x Engine/Build/BatchFiles/Linux/GitDependencies.sh
sudo apt-get install mono-complete
git clone https://github.com/OpenHUTB/UnrealEngine.git ~/UnrealEngine_4.26
```

__4.__ 打开编辑器检查虚幻引擎是否已正确安装。
```sh
cd ~/UnrealEngine_4.26/Engine/Binaries/Linux && ./UE4Editor
```

电脑配置达不到Vulkan的标准会出现（OpenGL的API也废弃了）：
```text
Cannot find a compatible Yulkan driver (ICD)
```

---

## 第二部分：构建 Carla <span id="part-two-build-carla"></span>

!!! 笔记
    使用 `sudo apt-get install aria2` 下载 aria2 将加快以下命令的速度。

### 克隆 Carla 仓库 <span id="clone-the-carla-repository"></span>

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla" target="_blank" class="btn btn-neutral" title="跳转至 Carla 仓库">
<span class="icon icon-github"></span> Carla 仓库</a>
</p>
</div>
上面的按钮将带您进入该项目的官方存储库。从那里下载并在本地提取它或使用以下命令克隆它：

```shell
git clone https://github.com/carla-simulator/carla.git
```

!!! 笔记
    The `master` 分支包含 Carla 的当前版本以及最新的修复和功能。以前的 Carla 版本标有版本名称。永远记住使用命令`git branch`检查 git 中的当前分支。 


### 获取资产 <span id="get-assets"></span>

您需要下载 __最新的__ 资源才能使用当前版本的 Carla。我们提供了一个脚本来自动执行此过程。要使用该脚本，请在 Carla 根文件夹中运行以下命令：

```sh
./Update.sh
```

资源将被下载并提取到适当的位置。

!!! 重要
    要下载当前正在开发的资产，请访问 [更新 Carla](build_update.md#get-development-assets) 并阅读 __获取开发资产__。

要下载 __特定版本__ 的 Carla 资源：

1. 从 Carla 根目录，导航到 `\Util\ContentVersions.txt`。本文档包含所有 Carla 版本的资源链接。
2. 提取 `Unreal\CarlaUE4\Content\Carla` 中的资产。如果该路径不存在，请创建它。
3. 使用类似以下内容的命令提取文件：

```sh
tar -xvzf <assets_file_name>.tar.gz.tar -C C:\path\to\carla\Unreal\CarlaUE4\Content\Carla
```

### 设置虚幻引擎环境变量 <span id="set-unreal-engine-environment-variable"></span>

为了让 Carla 找到虚幻引擎的正确安装，我们需要设置 Carla 环境变量。

仅为此会话设置变量：

```sh
export UE4_ROOT=~/UnrealEngine_4.26
```

要设置变量以使其在会话中保持不变：

__1.__ 打开 `~/.bashrc` 或 `./profile`。  
```sh
gedit ~/.bashrc
# 或 
gedit ~/.profile
```

__2.__ 将以下行添加到文件底部：

```sh
export UE4_ROOT=~/UnrealEngine_4.26 
```

__3.__ 保存文件并重置终端。


### 构建 Carla <span id="build-carla"></span>
本节概述了构建 Carla 的命令。__所有命令都应在根 Carla 文件夹中运行。__

Carla 的构建过程分为两个部分，编译客户端和编译服务器。

!!! 警告
    确保运行 `make PythonAPI` 以准备客户端和运行 `make launch` 准备服务端。
    或者 `make LibCarla` 将准备 Carla 库以导入到任何地方。

__1.__ __编译 Python API 客户端：__：

Python API 客户端授予对模拟的控制权。第一次构建 Carla 时需要编译 Python API 客户端，并且在执行任何更新后需要再次编译。客户端编译完成后，您将能够运行脚本与模拟进行交互。

以下命令编译 Python API 客户端：

```sh
make PythonAPI
```

或者，要为特定版本的 Python 编译 PythonAPI，请在根 Carla 目录中运行以下命令。

```sh
# Delete versions as required
make PythonAPI ARGS="--python-version=2.7, 3.6, 3.7, 3.8"
```

Carla 客户端库将以两种截然不同、互斥的形式构建。这使用户可以自由选择他们喜欢的形式来运行 Carla 客户端代码。两种形式包括`.egg`文件和`.whl`文件。选择以下选项之一来使用客户端库：

__A. `.egg` 文件__

>`.egg` 文件不需要安装。导入 Carla 时，Carla 的所有示例脚本都会自动 [查找此文件](build_system.md#versions-prior-to-0912) 。

>如果您之前安装了 Carla  `.whl`，`.whl` 将优先于 `.egg` 文件。

__B. `.whl` 文件__

>`.whl` 文件应使用 `pip` 或 `pip3` 安装：

```sh
# Python 3
pip3 install <path/to/wheel>.whl

# Python 2
pip install <path/to/wheel>.whl
```

>`.whl` 文件无法分发，因为它是专门为您的操作系统构建的。

!!! 警告
    使用不同方法安装 Carla 客户端库以及系统上安装不同版本的 Carla 可能会出现问题。建议在安装时使用虚拟环境，并在安装`.whl`新客户端库之前卸载任何以前安装的客户端库。


__2.__ __编译服务器__：

以下命令编译并启动虚幻引擎。每次您想要启动服务器或使用虚幻引擎编辑器时运行此命令：

```sh
    make launch
```

该项目可能会要求构建其他实例，例如`UE4Editor-Carla.dll`第一次。同意才能打开项目。在首次启动期间，编辑器可能会显示有关着色器和网格距离场的警告。这些需要一些时间来加载，在此之前地图将无法正确显示。


__3.__ __开始模拟__：

按“运行”开始服务器模拟。可以使用`WASD`键移动相机，并通过在移动鼠标的同时单击场景来旋转相机。

使用`PythonAPI\examples`里面的示例脚本测试模拟器。在模拟器运行的情况下，为每个脚本打开一个新终端并运行以下命令以在城镇中产生一些生命并创建天气循环：

```sh
# 终端 A 
cd PythonAPI/examples
python3 -m pip install -r requirements.txt
python3 generate_traffic.py  

# 终端 B
cd PythonAPI/examples
python3 dynamic_weather.py 
```

!!! 重要
    如果模拟以非常低的 FPS 速率运行，请转至虚幻引擎编辑器中的`Edit -> Editor preferences -> Performance`并禁用 `Use less CPU when in background`。



### 其他 make 命令 <span id="other-make-commands"></span>

您可能会发现还有更多有用的 `make` 命令。在下表中找到它们：

| 命令               | 描述                                      |
|------------------|-----------------------------------------|
| `make help`      | 打印所有可用的命令。                              |
| `make launch`    | 在编辑器窗口中启动 Carla 服务器。                    |
| `make PythonAPI` | 构建 Carla 客户端。                           |
| `make LibCarla`  | 准备将 Carla 库导入到任何地方。                     |
| `make package`   | 构建 Carla 并创建用于分发的打包版本。                  |
| `make clean`     | 删除构建系统生成的所有二进制文件和临时文件。                  |
| `make rebuild`   | `make clean` 和 `make launch` 两者都在一个命令中。 |
| `make check`     | 进行单元测试                                  |

---

### 其他 <span id="other"></span>
v2ray 的版本不能太高，比如可以使用 [v4.45.2](https://github.com/v2fly/v2ray-core/releases/tag/v4.45.2) ，图形界面使用[Qv2ray 2.7.0](https://github.com/Qv2ray/Qv2ray/releases) 。


## 问题

- 执行`make Python`时报错：

`A C++11 capable compiler is required for building the B2 engine.
Toolset 'clang' does not appear to support C++11.`


---

有关本指南的任何问题， 请阅读 **[常见问题解答](build_faq.md)** 页面或[Carla 论坛](https://github.com/carla-simulator/carla/discussions) 中的帖子。

接下来，学习如何更新 Carla 构建或在模拟中迈出第一步，并学习一些核心概念。
<div class="build-buttons">

<p>
<a href="../build_update" target="_blank" class="btn btn-neutral" title="Learn how to update the build">
更新 Carla</a>
</p>

<p>
<a href="../core_concepts" target="_blank" class="btn btn-neutral" title="Learn about CARLA core concepts">
第一步</a>
</p>

</div>
