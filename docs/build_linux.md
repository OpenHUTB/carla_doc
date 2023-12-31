# Linux 系统上的构建

本指南详细介绍了如何在 Linux 上从源代码构建 Carla。有两个部分。第一部分详细介绍系统要求和所需软件的安装，第二部分详细介绍如何实际构建和运行 Carla。

构建过程很长（4小时或更长时间）并且涉及多种软件。强烈建议在开始之前完整阅读该指南。

If you come across errors or difficulties then have a look at the **[F.A.Q.](build_faq.md)** page which offers solutions for the most common complications. Alternatively, use the [Carla forum](https://github.com/carla-simulator/carla/discussions) to post any queries you may have.

- [__Part One: Prerequisites__](#part-one-prerequisites)
    - [System requirements](#system-requirements)
    - [Software requirements](#software-requirements)
        - [Unreal Engine](#unreal-engine)
- [__Part Two: Build Carla__](#part-two-build-carla)
    - [Clone the Carla repository](#clone-the-carla-repository)
    - [Get assets](#get-assets)
    - [Set Unreal Engine environment variable](#set-unreal-engine-environment-variable)
    - [Build Carla](#build-carla)
    - [Other make commands](#other-make-commands)

---
## Part One: Prerequisites

### System requirements

* __Ubuntu 18.04.__ Carla provides support for previous Ubuntu versions up to 16.04. **However** proper compilers are needed for Unreal Engine to work properly. Dependencies for Ubuntu 18.04 and previous versions are listed separatedly below. Make sure to install the ones corresponding to your system.
* __130 GB disk space.__ Carla will take around 31 GB and Unreal Engine will take around 91 GB so have about 130 GB free to account for both of these plus additional minor software installations. 
* __An adequate GPU.__ Carla aims for realistic simulations, so the server needs at least a 6 GB GPU although 8 GB is recommended. A dedicated GPU is highly recommended for machine learning. 
* __Two TCP ports and good internet connection.__ 2000 and 2001 by default. Make sure that these ports are not blocked by firewalls or any other applications. 

..warning::
    __If you are upgrading from Carla 0.9.12 to 0.9.13__: you must first upgrade the Carla fork of the UE4 engine to the latest version. See the [__Unreal Engine__](#unreal-engine) section for details on upgrading UE4



### Software requirements

Carla requires many different kinds of software to run. Some are built during the Carla build process itself, such as *Boost.Python*. Others are binaries that should be installed before starting the build (*cmake*, *clang*, different versions of *Python*, etc.). To install these requirements, run the following commands:

```sh
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
sudo apt-get update
```

!!! Warning
    The following commands depend on your Ubuntu version. Make sure to choose accordingly. 

__Ubuntu 18.04__.

```sh
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
```
__Previous Ubuntu__ versions.

```sh
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng16-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git
```

__All Ubuntu systems__.

To avoid compatibility issues between Unreal Engine and the Carla dependencies, use the same compiler version and C++ runtime library to compile everything. The Carla team uses clang-8 and LLVM's libc++. Change the default clang version to compile Unreal Engine and the Carla dependencies.

```sh
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```

Starting with Carla 0.9.12, users have the option to install the Carla Python API using `pip` or `pip3`. Version 20.3 or higher is required. To check if you have a suitable version, run the following command:

```sh
# For Python 3
pip3 -V

# For Python 2
pip -V
```

If you need to upgrade:

```sh
# For Python 3
pip3 install --upgrade pip

# For Python 2
pip install --upgrade pip
```

You must install the following Python dependencies:

```sh
pip install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip install --user distro &&
pip3 install --user distro &&
pip install --user wheel &&
pip3 install --user wheel auditwheel
```

---

## Unreal Engine

Starting with version 0.9.12, Carla uses a modified fork of Unreal Engine 4.26. This fork contains patches specific to Carla.

Be aware that to download this fork of Unreal Engine, __you need to have a GitHub account linked to Unreal Engine's account__. If you don't have this set up, please follow [this guide](https://www.unrealengine.com/en-US/ue4-on-github) before going any further.

__1.__ Clone the content for Carla's fork of Unreal Engine 4.26 to your local computer:

```sh
    git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
```
__2.__ Navigate into the directory where you cloned the repository:
```sh
    cd ~/UnrealEngine_4.26
```

__3.__ Make the build. This may take an hour or two depending on your system. 
```sh
    ./Setup.sh && ./GenerateProjectFiles.sh && make
```

__4.__ Open the Editor to check that Unreal Engine has been installed properly.
```sh
    cd ~/UnrealEngine_4.26/Engine/Binaries/Linux && ./UE4Editor
```

---

## Part Two: Build Carla 

!!! Note
    Downloading aria2 with `sudo apt-get install aria2` will speed up the following commands.

### Clone the Carla repository

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


### Get assets

You will need to download the __latest__ assets to work with the current version of Carla. We provide a script to automate this process. To use the script, run the following command in the Carla root folder:

```sh
        ./Update.sh
```

The assets will be downloaded and extracted to the appropriate location.

!!! Important
    To download the assets currently in development, visit [Update Carla](build_update.md#get-development-assets) and read __Get development assets__.

To download the assets for a __specific version__ of CARLA:

1. From the root Carla directory, navigate to `\Util\ContentVersions.txt`. This document contains the links to the assets for all Carla releases. 
2. Extract the assets in `Unreal\CarlaUE4\Content\Carla`. If the path doesn't exist, create it.  
3. Extract the file with a command similar to the following:

```sh
        tar -xvzf <assets_file_name>.tar.gz.tar -C C:\path\to\carla\Unreal\CarlaUE4\Content\Carla
```

### Set Unreal Engine environment variable

For Carla to find the correct installation of Unreal Engine, we need to set the Carla environment variable.

To set the variable for this session only:

```sh
    export UE4_ROOT=~/UnrealEngine_4.26
```

To set the variable so it persists across sessions:

__1.__ Open `~/.bashrc` or `./profile`.  
```sh
    gedit ~/.bashrc

    # or 

    gedit ~/.profile
```

__2.__ Add the following line to the bottom of the file: 

```sh
    export UE4_ROOT=~/UnrealEngine_4.26 
```

__3.__ Save the file and reset the terminal.  


### Build Carla
This section outlines the commands to build Carla. __All commands should be run in the root Carla folder.__

There are two parts to the build process for Carla, compiling the client and compiling the server.

!!! Warning
    Make sure to run `make PythonAPI` to prepare the client and `make launch` for the server.
    Alternatively `make LibCarla` will prepare the Carla library to be imported anywhere.

__1.__ __Compile the Python API client__:

The Python API client grants control over the simulation. Compilation of the Python API client is required the first time you build Carla and again after you perform any updates. After the client is compiled, you will be able to run scripts to interact with the simulation.

The following command compiles the Python API client:

```sh
    make PythonAPI
```

Optionally, to compile the PythonAPI for a specific version of Python, run the below command in the root Carla directory.

```sh
    # Delete versions as required
    make PythonAPI ARGS="--python-version=2.7, 3.6, 3.7, 3.8"
```

The Carla client library will be built in two distinct, mutually exclusive forms. This gives users the freedom to choose which form they prefer to run the Carla client code. The two forms include `.egg` files and `.whl` files. Choose __one__ of the following options below to use the client library:

__A. `.egg` file__

>The `.egg` file does not need to be installed. All of Carla's example scripts automatically [look for this file](build_system.md#versions-prior-to-0912) when importing CARLA.

>If you previously installed a Carla `.whl`, the `.whl` will take precedence over an `.egg` file.

__B. `.whl` file__

>The `.whl` file should be installed using `pip` or `pip3`:

```sh
# Python 3
pip3 install <path/to/wheel>.whl

# Python 2
pip install <path/to/wheel>.whl
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
        cd PythonAPI/examples
        python3 -m pip install -r requirements.txt
        python3 generate_traffic.py  

        # Terminal B
        cd PythonAPI/examples
        python3 dynamic_weather.py 
```

!!! Important
    If the simulation is running at a very low FPS rate, go to `Edit -> Editor preferences -> Performance` in the Unreal Engine editor and disable `Use less CPU when in background`.



### Other make commands

There are more `make` commands that you may find useful. Find them in the table below:  

| Command | Description |
| ------- | ------- |
| `make help`                                                           | Prints all available commands.                                        |
| `make launch`                                                         | Launches Carla server in Editor window.                               |
| `make PythonAPI`                                                      | Builds the CARLA client.                                              |
| `make LibCarla`                                                       | Prepares the Carla library to be imported anywhere.                   |
| `make package`                                                        | Builds Carla and creates a packaged version for distribution.         |
| `make clean`                                                          | Deletes all the binaries and temporals generated by the build system. |
| `make rebuild`                                                        | `make clean` and `make launch` both in one command.                   |

---

Read the **[F.A.Q.](build_faq.md)** page or post in the [Carla forum](https://github.com/carla-simulator/carla/discussions) for any issues regarding this guide.  

Up next, learn how to update the Carla build or take your first steps in the simulation, and learn some core concepts.  
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
