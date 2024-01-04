# Windows 下的构建

本指南详细介绍了如何在 Windows 上从源代码构建 Carla。有两个部分。第一部分详细介绍系统要求和所需软件的安装，第二部分详细介绍如何实际构建和运行 Carla。

构建过程很长（4小时或更长时间）并且涉及多种软件。强烈建议在开始之前完整阅读该指南。

如果您遇到错误或困难，请查看**[常见问题解答](build_faq.md)**页面，其中提供了最常见问题的解决方案。或者，使用 [Carla 论坛](https://github.com/carla-simulator/carla/discussions) 发布您可能有的任何疑问。

- [__第一部分：先决条件__](#part-one-prerequisites)
    - [系统要求](#system-requirements)
    - [软件要求](#software-requirements)
        - [基础软件安装](#minor-installations)
        - [Python 依赖项](#python-dependencies)
        - [主要软件安装](#major-installations)
            - [Visual Studio 2019](#visual-studio-2019)
            - [虚幻引擎](#unreal-engine)
- [__第二部分：构建 Carla__](#part-two-build-carla)
    - [克隆 Carla 仓库](#clone-the-carla-repository)
    - [获取资产](#get-assets)
    - [设置虚幻引擎环境变量](#set-unreal-engine-environment-variable)
    - [构建 Carla](#build-carla)
    - [其他 make 命令](#other-make-commands)


---
## 第一部分：先决条件

在本节中，您将找到开始构建 Carla 之前所需的系统要求、次要和主要软件安装以及 Python 依赖项的详细信息。

### 系统要求

* __64 位操作系统。__ 仿真器应在任何 64 位 Windows 系统中运行。 
* __165 GB 磁盘空间。__ Carla 本身将占用大约 32 GB，相关的主要软件安装（包括虚幻引擎）将占用大约 133 GB。
* __足够的 GPU。__ Carla 旨在实现真实仿真，因此服务器至少需要 6 GB GPU，但建议使用 8 GB。强烈建议使用专用 GPU 进行机器学习。
* __两个 TCP 端口和良好的互联网连接。__ 默认端口为 2000 和 2001 。 确保这些端口未被防火墙或任何其他应用程序阻止。 

..警告::
    __如果您要从 Carla 0.9.12 升级到 0.9.13__: 您必须首先将虚幻引擎 4 的 Carla 分支升级到最新版本。有关升级虚幻引擎 4 的详细信息，请参阅 [__虚幻引擎__](#unreal-engine) 部分。

### 软件要求

#### 基础软件安装

* [__CMake__](https://cmake.org/download/) 从简单的配置文件生成标准构建文件。  
* [__Git__](https://git-scm.com/downloads) 是一个用于管理 Carla 存储库的版本控制系统。
* [__Make__](http://gnuwin32.sourceforge.net/packages/make.htm) 生成可执行文件。必须使用 __Make 的 3.81 版本__，否则构建可能会失败。如果安装了多个版本的 Make，请检查构建 Carla 时在 PATH 中使用的版本是否为 3.81。您可以通过运行来检查默认的 Make 版本 `make --version`。
* [__7Zip__](https://www.7-zip.org/) 一款文件压缩软件。这是自动解压缩资产文件所必需的，并防止在构建期间由于错误或部分提取大文件而出现错误。
* [__Python3 x64__](https://www.python.org/downloads/) 是 Carla 中的主要脚本语言。安装 x32 版本可能会导致冲突，因此强烈建议卸载它。

!!! 重要
    确保将上述程序添加到 [环境路径](https://www.java.com/en/download/help/path.xml) 中。请记住，添加的路径应与程序的 `bin` 目录相对应。
#### Python 依赖项

从 Carla 0.9.12 开始，用于可以选择使用 `pip3` 来安装 Carla 的 Python API。要检查您是否有合适的版本，请运行以下命令：

```sh
pip3 -V
```

如果您需要升级：

```sh
pip3 install --upgrade pip
```

您必须安装以下 Python 依赖项：

```sh
pip3 install --user setuptools
pip3 install --user wheel
```

#### 主要软件安装
##### Visual Studio 2019

从 [此处](https://developerinsider.co/download-visual-studio-2019-web-installer-iso-community-professional-enterprise/) 获取 2019 版 Visual Studio 。选择 __社区__ 作为免费版本。使用 __Visual Studio 安装程序__ 安装三个附加元素： 

* __Windows 8.1 SDK.__ Select it in the _Installation details_ section on the right or go to the _Indivdual Components_ tab and look under the _SDKs, libraries, and frameworks_ heading.
* __x64 Visual C++ Toolset.__ In the _Workloads_ section, choose __Desktop development with C++__. This will enable a x64 command prompt that will be used for the build. Check that it has been installed correctly by pressing the `Windows` button and searching for `x64`. Be careful __not to open a `x86_x64` prompt__.  
* __.NET framework 4.6.2__. In the _Workloads_ section, choose __.NET desktop development__ and then in the _Installation details_ panel on the right, select `.NET Framework 4.6.2 development tools`. This is required to build Unreal Engine. 

!!! 重要
    Other Visual Studio versions may cause conflict. Even if these have been uninstalled, some registers may persist. To completely clean Visual Studio from the computer, go to `Program Files (x86)\Microsoft Visual Studio\Installer\resources\app\layout` and run `.\InstallCleanup.exe -full`  

[命令行参数安装](https://learn.microsoft.com/zh-cn/visualstudio/install/use-command-line-parameters-to-install-visual-studio?view=vs-2019) 。

##### 虚幻引擎

从版本 0.9.12 开始，Carla 使用虚幻引擎 4.26 的修改版。该分支包含 Carla 特定的补丁。

Be aware that to download this fork of Unreal Engine, __you need to have a GitHub account linked to Unreal Engine's account__. If you don't have this set up, please follow [this guide](https://www.unrealengine.com/en-US/ue4-on-github) before going any further.

To build the modified version of Unreal Engine:

__1.__ In a terminal, navigate to the location you want to save Unreal Engine and clone the _carla_ branch:

```sh
    git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git .
```

!!! 笔记 
    虚幻引擎文件夹尽可能靠近`C:\\`，因为如果路径超过一定长度，`Setup.bat`则会在步骤 3 中返回错误。

__2.__ 运行配置脚本：

```sh
    Setup.bat
    GenerateProjectFiles.bat
```

__3.__ 编译修改后的引擎：

>1. Open the `UE4.sln` file inside the source folder with Visual Studio 2019.

>2. In the build bar ensure that you have selected 'Development Editor', 'Win64' and 'UnrealBuildTool' options. Check [this guide](https://docs.unrealengine.com/en-US/ProductionPipelines/DevelopmentSetup/BuildingUnrealEngine/index.html) if you need any help. 
        
>3. In the solution explorer, right-click `UE4` and select `Build`.

__4.__ Once the solution is compiled you can open the engine to check that everything was installed correctly by launching the executable `Engine\Binaries\Win64\UE4Editor.exe`.

!!! Note
    If the installation was successful, this should be recognised by Unreal Engine's version selector. You can check this by right-clicking on any `.uproject` file and selecting `Switch Unreal Engine version`. You should see a pop-up showing `Source Build at PATH` where PATH is the installation path that you have chosen. If you can not see this selector or the `Generate Visual Studio project files` when you right-click on `.uproject` files, something went wrong with the Unreal Engine installation and you will likely need to reinstall it correctly.

!!! Important
    A lot has happened so far. It is highly advisable to restart the computer before continuing.

---
## 第二部分：构建 Carla
 
### 克隆 Carla 存储库

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla" target="_blank" class="btn btn-neutral" title="Go to the CARLA repository">
<span class="icon icon-github"></span> CARLA repository</a>
</p>
</div>

The button above will take you to the official repository of the project. Either download from there and extract it locally or clone it using the following command:

```sh
    git clone https://github.com/carla-simulator/carla
```

!!! Note
    The `master` branch contains the current release of Carla with the latest fixes and features. Previous Carla versions are tagged with the version name. Always remember to check the current branch in git with the command `git branch`. 

### 获取资产

Download the __latest__ assets to work with the current version of CARLA by running the following command in the CARLA root folder:

```sh
    Update.bat
```

The assets will be downloaded and extracted to the appropriate location if have 7zip installed. If you do not have this software installed, you will need to manually extract the file contents to `Unreal\CarlaUE4\Content\Carla`.

To download the assets for a __specific version__ of CARLA:

1. From the root Carla directory, navigate to `\Util\ContentVersions.txt`. This document contains the links to the assets for all CARLA releases. 
2. Extract the assets in `Unreal\CarlaUE4\Content\Carla`. If the path doesn't exist, create it.  
3. Extract the file with a command similar to the following:

```sh
    tar -xvzf <assets_file_name>.tar.gz.tar -C C:\path\to\carla\Unreal\CarlaUE4\Content\Carla
```

### 设置虚幻引擎变量

It is necessary to set an environment variable so that Carla can find the Unreal Engine installation folder. This allows users to choose which specific version of Unreal Engine is to be used. If no environment variable is specified, then Carla will search for Unreal Engine in the windows registry and use the first version it finds there.  

To set the environment variable:

1. Open Windows Control Panel and go to `Advanced System Settings` or search for `Advanced System Settings` in the Windows search bar.  
2. On the `Advanced` panel open `Environment Variables...`.  
3. Click `New...` to create the variable.  
4. Name the variable `UE4_ROOT` and choose the path to the installation folder of the desired Unreal Engine installation.  


### 构建 Carla

This section outlines the commands to build Carla. 

- All commands should be run in the root Carla folder. 
- Commands should be executed via the __x64 Native Tools Command Prompt for VS 2019__. Open this by clicking the `Windows` key and searching for `x64`.

There are two parts to the build process for Carla, compiling the client and compiling the server.

__1.__ __Compile the Python API client__:

The Python API client grants control over the simulation. Compilation of the Python API client is required the first time you build Carla and again after you perform any updates. After the client is compiled, you will be able to run scripts to interact with the simulation.

The following command compiles the Python API client:

```sh
    make PythonAPI
```

The Carla client library will be built in two distinct, mutually exclusive forms. This gives users the freedom to choose which form they prefer to run the Carla client code. The two forms include `.egg` files and `.whl` files. Choose __one__ of the following options below to use the client library:

__A. `.egg` file__

>The `.egg` file does not need to be installed. All of CARLA's example scripts automatically [look for this file](build_system.md#versions-prior-to-0912) when importing CARLA.

>If you previously installed a Carla `.whl`, the `.whl` will take precedence over an `.egg` file.

__B. `.whl` file__

>The `.whl` file should be installed using `pip3`:

```sh
pip3 install <path/to/wheel>.whl
```

>This `.whl` file cannot be distributed as it is built specifically for your OS.

!!! Warning
    Issues can arise through the use of different methods to install the Carla client library and having different versions of Carla on your system. It is recommended to use virtual environments when installing the `.whl` and to [uninstall](build_faq.md#how-do-i-uninstall-the-carla-client-library) any previously installed client libraries before installing new ones.

__2.__ __Compile the server__:

The following command compiles and launches Unreal Engine. Run this command each time you want to launch the server or use the Unreal Engine editor:

```sh
    make launch
```

The project may ask to build other instances such as `UE4Editor-Carla.dll` the first time. Agree in order to open the project. During the first launch, the editor may show warnings regarding shaders and mesh distance fields. These take some time to be loaded and the map will not show properly until then.

__3.__ __Start the simulation__:

Press **Play** to start the server simulation. The camera can be moved with `WASD` keys and rotated by clicking the scene while moving the mouse around.  

Test the simulator using the example scripts inside `PythonAPI\examples`.  With the simulator running, open a new terminal for each script and run the following commands to spawn some life into the town and create a weather cycle:

```sh
        # Terminal A 
        cd PythonAPI\examples
        pip3 install -r requirements.txt
        python3 generate_traffic.py  

        # Terminal B
        cd PythonAPI\examples
        python3 dynamic_weather.py 
```

!!! Important
    If the simulation is running at a very low FPS rate, go to `Edit -> Editor preferences -> Performance` in the Unreal Engine editor and disable `Use less CPU when in background`.

### 其他构建命令

There are more `make` commands that you may find useful. Find them in the table below:  

| Command | Description |
| ------- | ------- |
| `make help`                                                           | Prints all available commands.                                        |
| `make launch`                                                         | Launches Carla server in Editor window.                               |
| `make PythonAPI`                                                      | Builds the Carla client.                                              |
| `make LibCarla`                                                       | Prepares the Carla library to be imported anywhere.                   |
| `make package`                                                        | Builds Carla and creates a packaged version for distribution.         |
| `make clean`                                                          | Deletes all the binaries and temporals generated by the build system. |
| `make rebuild`                                                        | `make clean` and `make launch` both in one command.                   |


---

Read the **[F.A.Q.](build_faq.md)** page or post in the [CARLA forum](https://github.com/carla-simulator/carla/discussions) for any issues regarding this guide.  

Now that you have built CARLA, learn how to update the CARLA build or take your first steps in the simulation, and learn some core concepts.

<div class="build-buttons">

<p>
<a href="../build_update" target="_blank" class="btn btn-neutral" title="Learn how to update the build">
Update Carla</a>
</p>

<p>
<a href="../core_concepts" target="_blank" class="btn btn-neutral" title="Learn about CARLA core concepts">
First steps</a>
</p>

</div>
