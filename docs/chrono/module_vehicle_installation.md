# 安装 VEHICLE 模块

这是一个可选模块，可在 Chrono 内实现基于模板的地面车辆建模和模拟。

阅读 [模块介绍](https://api.projectchrono.org/modularity.html) ，了解 Chrono 项目模块化的技术背景。

## 特点

VEHICLE 模块允许用户对车辆进行建模和模拟。

有关更多详细信息，请阅读参考手册的 [Chrono::Vehicle](https://api.projectchrono.org/manual_vehicle.html) 部分。

## 依赖项
- 此模块没有其他依赖项
- 使用 [CRGTerrain](https://api.projectchrono.org/vehicle_terrain.html#vehicle_terrain_crg) 功能需要 [OpenCRG](https://www.asam.net/standards/detail/opencrg/) 库（定义了一种用于描述路面的文件格式。它最初是为了存储路面扫描的高精度高程数据而开发的。这些数据主要用于轮胎、振动或驾驶模拟。精确的高程数据可以对车辆部件或整个车辆进行逼真的耐久性模拟。）。

对于运行时可视化，建议启用并安装 [Chrono::VSG](https://api.projectchrono.org/module_vsg_installation.html) 模块和/或 [Chrono::Irrlicht](https://api.projectchrono.org/module_irrlicht_installation.html) 模块。

## 构建和安装先决条件
Chrono::Vehicle 包含使用 OpenCRG 规范指定地形的选项。支持此可选功能需要 [OpenCRG 库](https://www.asam.net/index.php?eID=dumpFile&t=f&f=3950&token=21a7ae456ec0eb0f9ec3aee5bae3e8c9ebaea140) 。

构建和安装 OpenCRG 库的最简单方法是使用 Chrono 发行版提供的实用程序脚本。这些脚本（`buildOpenCRG.bat`分别`buildOpenCRG.sh`用于 Windows 和 Linux）位于 [Chrono 存储库](https://github.com/projectchrono/chrono/tree/main/contrib/build-scripts/opencrg) `contrib/build-scripts/opencrg`目录中。

1. 复制适当的脚本并放置在任意临时目录中。
2. 按照脚本顶部注释中的说明编辑脚本。
3. 从脚本副本的位置运行脚本（`.\buildOpenCRG.bat`或`sh buildOpenCRG.sh`，视情况而定）。这将创建一个临时目录，所有源存储库都将被克隆到该目录，以及一组目录，各个 URDF 依赖项将在其中构建。
4. 安装目录将包含配置 Chrono::Vehicle 与 OpenCRG 支持所需的必要头文件和库文件。

## 搭建说明

1. 重复 [完整安装](https://api.projectchrono.org/tutorial_install_chrono.html) 的说明，但是当您看到 CMake 窗口时，必须添加以下步骤：
2. 将其设置为`ENABLE_MODULE_VEHICLE`“开”，然后按“配置”（刷新变量列表）
3. 再次按“配置”，然后按“生成”，并按照安装说明进行操作。

如果启用 OpenCRG 支持 (`ENABLE_OPENCRG`)，系统将提示您提供 OpenCRG 本地安装的位置，包括标头、库的位置以及（仅限 Windows）OpenCRG DLL 的位置。

## 如何使用它

- 查阅 [参考手册](https://api.projectchrono.org/manual_vehicle.html) 。
- 查看此模块的 [API 部分](https://api.projectchrono.org/group__vehicle.html) 以获取有关类和函数的文档。
- 查看 [演示](https://api.projectchrono.org/tutorial_table_of_content_chrono_vehicle.html) 的 C++源代码来了解如何使用该模块的功能。


