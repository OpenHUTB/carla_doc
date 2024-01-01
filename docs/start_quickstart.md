# [快速启动包安装](https://carla.readthedocs.io/en/latest/start_quickstart/) 

本指南介绍如何下载和安装 CARLA 的打包版本。该软件包包括 CARLA 服务器和两个客户端库选项。还可以下载其他资源并将其导入到包中。需要使用虚幻引擎编辑器的高级自定义和开发选项不可用，但可以通过使用适用于 [Windows](build_windows.md) 或 [Linux](build_linux.md) 的 CARLA 构建版本来访问这些选项。

* __[在你开始之前](#before-you-begin)__  
* __[CARLA 安装](#carla-installation)__  
	* [A. Debian CARLA 安装](#a-debian-carla-installation)  
	* [B. 包安装](#b-package-installation)  
* __[导入额外资产](#import-additional-assets)__  
* __[安装客户端库](#install-client-library)__
    * [0.9.12 之前的 CARLA 版本](#carla-versions-prior-to-0912)
    * [CARLA 0.9.12+](#carla-0912)
* __[运行 CARLA](#running-carla)__  
	* [命令行选项](#command-line-options)  
* __[更新 CARLA](#updating-carla)__    
* __[跟进](#follow-up)__ 
---
## 在你开始之前

安装 CARLA 之前应满足以下要求：

* __系统要求。__ CARLA 是为 Windows 和 Linux 系统构建的。
* __足够的 GPU。__ CARLA 旨在实现真实仿真，因此服务器至少需要 6 GB GPU，但我们建议使用 8 GB。强烈建议使用专用 GPU 进行机器学习。
* __磁盘空间。__ CARLA 将使用大约 20 GB 的空间。
* __Python.__ [Python]((https://www.python.org/downloads/)) 是 CARLA 中的主要脚本语言。CARLA 在 Linux 上支持 Python 2.7 和 Python 3，在 Windows 上支持 Python 3。
* __Pip.__ CARLA 客户端库的某些安装方法需要 __pip__ 或 __pip3__ （取决于您的 Python 版本）版本 20.3 或更高版本。要检查您的 __pip__ 版本：

>>      # For Python 3
>>      pip3 -V

>>      # For Python 2
>>      pip -V

>如果您需要升级：

>>      # For Python 3
>>      pip3 install --upgrade pip

>>      # For Python 2
>>      pip install --upgrade pip

* __两个 TCP 端口和良好的互联网连接。__ 默认为 2000 年和 2001 年。确保这些端口未被防火墙或任何其他应用程序阻止。
* __其他需求。__  CARLA 需要一些 Python 依赖项。根据您的操作系统安装依赖项：

### Windows

```sh
pip3 install --user pygame numpy
```

### Linux

```sh
pip install --user pygame numpy &&
pip3 install --user pygame numpy
```

---
## CARLA 安装

有两种方法可以下载并安装 CARLA 作为软件包：

__A)__ [下载  Debian 软件包。](#a-debian-carla-installation)

__B)__ [从 GitHub 下载软件包。](#b-package-installation) 

### A. Debian CARLA 安装

Debain 软件包适用于 Ubuntu 18.04 和 Ubuntu 20.04，但 __官方支持的平台是 Ubuntu 18.04__ 。

__1.__ 在系统中设置Debian存储库：
```sh
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
    sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
```

__2.__ 安装 CARLA 并检查 `/opt/` 文件夹中是否有安装：
```sh
    sudo apt-get update # Update the Debian package index
    sudo apt-get install carla-simulator # Install the latest CARLA version, or update the current installation
    cd /opt/carla-simulator # Open the folder where CARLA is installed
```

该存储库包含 CARLA 0.9.10 及更高版本。要安装特定版本，请将版本标签添加到安装命令中：
```sh
    apt-cache madison carla-simulator # List the available versions of Carla
    sudo apt-get install carla-simulator=0.9.10-1 # In this case, "0.9.10" refers to a CARLA version, and "1" to the Debian revision
```

!!! 重要
    要安装 0.9.10 之前的 CARLA 版本，请使用窗口右下角的面板更改为文档的早期版本，然后按照旧的说明进行操作。

### B. 包安装

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/blob/master/Docs/download.md" target="_blank" class="btn btn-neutral" title="Go to the latest CARLA release">
<span class="icon icon-github"></span> CARLA 仓库</a>
</p>
</div>

该存储库包含不同版本的 CARLA。您将找到下载包含所有最新修复和功能的 __当前版本__ 、以前的版本以及包含所有开发修复和功能的 __夜间构建__ （夜间构建是 CARLA 最不稳定的版本）的选项。

该包是一个名为 __CARLA_version.number__ 的压缩文件。下载并解压发布文件。它包含模拟器的预编译版本、Python API 模块和一些用作示例的脚本。

---
## 导入额外资产

每个版本都有其自己的附加资产和地图包。此附加包包括地图 __Town06__ 、__Town07__ 和 __Town10__ 。它们单独存储以减少构建的大小，因此只能在安装主包后才能导入它们。

__1.__ [下载](https://github.com/carla-simulator/carla/blob/master/Docs/download.md) 适合您所需版本的 CARLA 的软件包。

__2.__ 解压包：

- __在 Linux 上__:

    - 将包移动到 _Import_ 文件夹并运行以下脚本以提取内容：

```sh
        cd path/to/carla/root

        ./ImportAssets.sh
```

- __在 Windows 上__:

    - 直接将内容提取到根文件夹中。

---

## 安装客户端库

### 0.9.12 之前的 CARLA 版本

以前版本的 CARLA 不需要安装 Python 库，它们附带了一个 `.egg` 开箱即用的文件。__CARLA 版本 0.9.12+ 显着改变了这种行为；有多个选项可用于安装客户端库__ 。如果您使用的是 0.9.12 之前的 CARLA 版本，请在屏幕右下角选择该版本以查看相关文档。否则，请阅读下面有关 CARLA 0.9.12+ 中可用选项的信息。

### CARLA 0.9.12+

有多个选项可用于安装和使用 CARLA 客户端库：

- __.egg__ 文件
- __.whl__ 文件
- __可下载的Python包__

在决定使用哪种方法之前，请阅读下面有关每种方法的要求和限制的更多信息。请注意，混合使用不同的方法可能会导致不兼容，因此请尽可能使用虚拟环境，或者在使用新库之前 [卸载](build_faq.md#how-do-i-uninstall-the-carla-client-library) 以前安装的库。

>__A. .egg 文件__

>>CARLA 提供了在 `PythonAPI/carla/dist/` 文件夹下的不同 Python 版本 `.egg` 文件，可以开箱即用。`PythonAPI/examples` 中的每个示例脚本都包含一个自动查找此文件的 [代码片段](build_system.md#versions-prior-to-0912) 。在 Linux 中，您可能需要将此文件添加到您的 `PYTHONPATH`。.在 [此处](build_faq.md#importerror-no-module-named-carla) 阅读有关  CARLA 中 `.egg` 文件的更多信息。

>>__如果您之前已经安装了客户端库 `pip`，则该库将优先于该 `.egg` 文件__。您需要先 [卸载](build_faq.md#how-do-i-uninstall-the-carla-client-library) 以前的库。

>__B. .whl 文件__

>>CARLA 提供了不同 Python 版本的 `.whl` 文件。您将需要安装该.whl文件。该`.whl`文件位于`PythonAPI/carla/dist/`。每个受支持的 Python 版本都有一个文件，由文件名指示（例如，carla-0.9.12-__cp36__-cp36m-manylinux_2_27_x86_64.whl 表示 Python 3.6）。

>>__建议在虚拟环境中安装 CARLA 客户端库，以避免在使用多个版本时发生冲突。__

>>要安装 CARLA 客户端库，请运行以下命令，选择适合您所需 Python 版本的文件。您将需要 __pip/pip3__ 版本 20.3 或更高版本。有关如何检查版本和升级pip/pip3的信息，请参阅 [__开始之前__](#before-you-begin) 部分：

>>      # Python 3
>>      pip3 install <wheel-file-name>.whl 

>>      # Python 2
>>      pip install <wheel-file-name>.whl

>>如果您之前安装了客户端库，则应先 [卸载](build_faq.md#how-do-i-uninstall-the-carla-client-library) 旧客户端库，然后再安装新客户端库。

>__C. 可下载的 Python 包__

>>CARLA 客户端库可以从 [PyPi](https://pypi.org/project/carla/) 下载。该库与 Python 版本 2.7、3.6、3.7 和 3.8 兼容。要安装它，您需要 __pip/pip3__ 版本 20.3 或更高版本。有关如何检查版本和升级pip/pip3的信息，请参阅 [__开始之前__](#before-you-begin) 部分。

>>__建议在虚拟环境中安装 CARLA 客户端库，以避免在使用多个版本时发生冲突。__

>>要从 PyPi 安装客户端库，请运行以下命令：

>>      # Python 3
>>      pip3 install carla

>>      # Python 2
>>      pip install carla

>>PyPi 下载仅适合与 CARLA 包一起使用（即不适用于从源代码构建的版本）。由于 PyPi 下载 __仅包含客户端库__，因此在您将与远程 CARLA 服务器通信且不需要下载完整的 CARLA 包的情况下，它非常有用。

---
## 运行 CARLA

启动 CARLA 服务器的方法取决于您使用的安装方法和操作系统：

- Debian 安装:

```sh
    cd /opt/carla-simulator/bin/

    ./CarlaUE4.sh
```

- Linux 软件包安装：

```sh
    cd path/to/carla/root

    ./CarlaUE4.sh
```

- Windows 包安装：

```sh
    cd path/to/carla/root

    CarlaUE4.exe
```

将弹出一个包含城市景观的窗口。这是 __观察者__ 的看法。要在城市中飞行，请使用鼠标和WASD按键，按住鼠标右键控制方向。

这是服务器仿真器，它现在正在运行并等待客户端连接并与世界交互。您可以尝试一些示例脚本来在城市中产生生命并驾驶汽车：

```sh
        # 终端 A 
        cd PythonAPI\examples

        python3 -m pip install -r requirements.txt # Support for Python2 is provided in the CARLA release packages

        python3 generate_traffic.py  

        # Terminal B
        cd PythonAPI\examples

        python3 manual_control.py 
```

#### 命令行选项

启动 CARLA 时有一些可用的配置选项，可以按如下方式使用它们：

```sh
    ./CarlaUE4.sh -carla-rpc-port=3000
```

* `-carla-rpc-port=N` 侦听端口 `N` 上的客户端连接。流媒体端口默认设置为 `N+1` 。
* `-carla-streaming-port=N` 指定传感器数据流的端口。使用 0 获取随机未使用的端口。第二个端口将自动设置为`N+1`。
* `-quality-level={Low,Epic}` 更改图形质量级别。 在 [渲染选项](adv_rendering_options.md) 中了解更多信息。
* __[虚幻引擎 4 命令行参数列表][ue4clilink]。__ 。虚幻引擎提供了很多选项，但并非所有这些选项在 CARLA 中都可用。

[ue4clilink]: https://docs.unrealengine.com/en-US/Programming/Basics/CommandLineArguments

该脚本 [`PythonAPI/util/config.py`][config] 提供了更多配置选项，应在服务器启动时运行：

[config]: https://github.com/carla-simulator/carla/blob/master/PythonAPI/util/config.py

```sh
    ./config.py --no-rendering      # Disable rendering
    ./config.py --map Town05        # Change map
    ./config.py --weather ClearNoon # Change weather

    ./config.py --help # Check all the available configuration options
```

---
## 更新 CARLA

无法更新 CARLA 的打包版本。当新版本发布时，存储库会更新，您需要删除以前的版本并安装新版本。

如果您使用 __pip/pip3__ 安装了客户端库，则应通过运行以下命令将其卸载：

```sh
# Python 3
pip3 uninstall carla

# Python 2
pip uninstall carla
```

---
## 跟进

现在您应该已经启动并运行了 CARLA 的打包版本。如果您在安装过程中遇到任何困难，请随时在 [CARLA 论坛](https://github.com/carla-simulator/carla/discussions/) 或 [Discord](https://discord.gg/8kqACuC) 频道中发帖。

下一步是详细了解 CARLA 的核心概念。阅读 __“第一步”__ 部分开始学习。您还可以在Python API 参考中找到有关 Python API 类和方法的所有信息。


<div class="build-buttons">
<p>
<a href="../core_concepts" target="_blank" class="btn btn-neutral" title="Go to first steps">
前往：第一步</a>
</p>
</div>
