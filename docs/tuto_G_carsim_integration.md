# CarSim 集成

CARLA 与 CarSim 的集成允许将 CARLA 中的车辆控制转发到 CarSim。CarSim 将对车辆进行所有必需的物理计算，并将新状态返回给 CARLA。

本页向您展示如何生成`.sim`文件，解释 CARLA 和 CarSim 之间的车辆尺寸如何关联，以及如何使用 CarSim 集成在 CARLA 上运行仿真。

*   [__在你开始之前__](#before-you-begin)  
*   [__设置 CarSim__](#set-up-carsim)  
	*   [__生成 .sim 文件__](#generate-the-sim-file)  
        * [__在 Windows 上__](#on-windows)
        * [__在 Ubuntu 上__](#on-ubuntu)
	*   [__Vehicle sizes__](#vehicle-sizes)  
*   [__Run the simulation__](#run-the-simulation)  

---
## 在你开始之前

1. 您需要 CarSim 许可证才能启动并运行该软件。如果您目前没有 CarSim 许可证，您可以在 [此处](https://www.carsim.com/forms/additional_information.php) 联系团队以获取信息。
2. 要允许与虚幻引擎通信，您需要安装适用于虚幻引擎 4.24 的 VehicleSim Dynamics 插件（版本 2020.0）。有关查找插件特定版本的信息，请检查此 [链接](https://www.carsim.com/products/supporting/unreal/index.php) 。插件的安装取决于您的操作系统： 
   
    __对于 Windows__:

    Get the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).

    __对于 Ubuntu__:

    1. 在 [这里](https://www.carsim.com/users/unreal_plugin/unreal_plugin_2020_0.php) 下载插件。
    2. 将文件 `CarSim.Build.cs` 替换为 [此处](https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/CarSim.Build.cs) 找到的文件，以便为 Ubuntu 使用正确的求解器。

    3. 如果您使用的是 CARLA 的打包版本，则可以跳过此步骤。打包版本已使用此标志编译，但如果您从源代码构建 CARLA，则需要使用`--carsim`标志编译服务器。

    如果您从源代码构建 CARLA，请在根文件夹中运行以下命令以使用`--carsim`标志编译服务器：

```sh
    make launch ARGS="--carsim"
```

## 设置 CarSim

以下部分详细介绍了如何生成运行仿真所需的 `.sim` 文件。还有关于 CARLA 和 CarSim 之间车辆尺寸关系的详细重要信息。

#### 生成.sim 文件

该`.sim` 文件描述了要在 CARLA 和 CarSim 中运行的仿真。插件需要此文件才能运行仿真。目前无法在 Ubuntu 上生成此文件，但是我们将在下面介绍如何使用之前生成的文件在 Ubuntu 上运行仿真。

##### 在 Windows 上

在 CarSim 上配置完所有参数后，使用 GUI 生成 `.sim` 文件，如下所示：

![generate .sim file](img/carsim_generate.jpg)

生成的`.sim`文件应如下所示：

```
SIMFILE

SET_MACRO $(ROOT_FILE_NAME)$ Run_dd7a828d-4b14-4c77-9d09-1974401d6b25
SET_MACRO $(OUTPUT_PATH)$ D:\carsim\Data\Results
SET_MACRO $(WORK_DIR)$ D:\carsim\Data\
SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\Run_dd7a828d-4b14-4c77-9d09-1974401d6b25\LastRun

FILEBASE $(OUTPUT_FILE_PREFIX)$
INPUT $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\Run_all.par
INPUTARCHIVE $(OUTPUT_FILE_PREFIX)$_all.par
ECHO $(OUTPUT_FILE_PREFIX)$_echo.par
FINAL $(OUTPUT_FILE_PREFIX)$_end.par
LOGFILE $(OUTPUT_FILE_PREFIX)$_log.txt
ERDFILE $(OUTPUT_FILE_PREFIX)$.vs
PROGDIR D:\carsim\
DATADIR D:\carsim\Data\
GUI_REFRESH_V CarSim_RefreshEvent_7760
RESOURCEDIR D:\carsim\\Resources\
PRODUCT_ID CarSim
PRODUCT_VER 2020.0
ANIFILE D:\carsim\Data\runs\animator.par
VEHICLE_CODE i_i
EXT_MODEL_STEP 0.00050000
PORTS_IMP 0
PORTS_EXP 0

DLLFILE D:\carsim\Programs\solvers\carsim_64.dll
END
```
##### 在Ubuntu上

无法在 Ubuntu 上通过 GUI 创建 `.sim` 文件。为了继续，您需要执行以下步骤：

1. 在 Windows 中生成 `.sim` 文件或使用下面的文件模板。
2. 修改该 `.sim` 文件，使变量 `INPUT`, `INPUTARCHIVE`, `LOGFILE` 等指向 Ubuntu 系统中的相应文件。
3. 替换 `DLLFILE` 行以指向 CarSim 解算器，在默认安装中，该解算器将为 `SOFILE /opt/carsim_2020.0/lib64/libcarsim.so.2020.0`. 

生成的文件应与此类似：

```
SIMFILE

FILEBASE /path/to/LastRun
INPUT /path/to/Run_all.par
INPUTARCHIVE /path/to/LastRun_all.par
ECHO /path/to/LastRun_echo.par
FINAL /path/to/LastRun_end.par
LOGFILE /path/to/LastRun_log.txt
ERDFILE /path/to/LastRun.vs
PROGDIR /opt/carsim_2020.0/lib64/
DATADIR .
PRODUCT_ID CarSim
PRODUCT_VER 2020.0
VEHICLE_CODE i_i

SOFILE /opt/carsim_2020.0/lib64/libcarsim.so.2020.0
END
```
#### 车辆尺寸

尽管 CarSim 允许您指定在模拟中使用的车辆尺寸，但目前 CarSim 车辆和 CARLA 车辆之间没有关联。这意味着两个项目中的车辆将具有不同的尺寸。CARLA 车辆的作用只是在仿真过程中充当占位符。

![carsim vehicle sizes](img/carsim_vehicle_sizes.jpg)

!!! 笔记
    CARLA 和 CarSim 中的车辆尺寸之间没有相关性。CARLA 车辆只是一个模拟占位符。

## 运行仿真

运行仿真时所需要做的就是在生成车辆时启用 CarSim。这可以通过将`.sim` 文件路径传递给Python API 的 [方法](https://carla.readthedocs.io/en/latest/python_api/#carla.Vehicle.enable_carsim) 来完成：

```sh
vehicle.enable_carsim(<path_to_ue4simfile.sim>)
```

发送到车辆的所有输入控件都将转发到 CarSim。CarSim 将更新物理并将车辆状态（变换）发送回 CARLA 车辆。

仿真完成后，您可以像往常一样分析 CarSim 中的所有数据。

![carsim analysis](img/carsim_analysis.jpg)


