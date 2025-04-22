# [添加新地图](https://carla.readthedocs.io/en/latest/tuto_M_custom_map_overview/)

Carla 的用户可以创建自定义地图并使用它们来运行模拟。在 Carla 中导入自定义地图的方法有多种。要使用的方法取决于您使用的是 Carla 的打包版本还是从源代码构建的版本。本节概述了启动该流程所需的内容、可用于导入的不同选项以及可用的自定义和行人导航工具。

- [__概述__](#overview)
- [__生成__](#generation)
- [__导入__](#importation)
- [__定制__](#customization)
- [__生成行人导航__](#generate-pedestrian-navigation)

---

## 概述 <span id="overview"></span>

在 Carla 中使用自定义地图涉及四个主要过程：

1. 生成
2. 导入
3. 定制
4. 行人导航

请继续阅读，了解有关每个流程的其他一般信息。

---

## 生成 <span id="generation"></span>

Carla 需要 `.fbx` 格式的地图几何信息和 `.xodr` 格式的 OpenDRIVE 信息。目前推荐的生成这些文件的软件是 RoadRunner。

__[本指南](tuto_M_generate_map.md) 说明如何使用 RoadRunner 生成地图信息。__

---

## 导入 <span id="importation"></span>

有多种方法可以将地图导入 Carla。 

如果您使用的是 Carla __软件包版本__ ，您将使用 Docker 导入地图。此选项仅在 Linux 中可用，并且您将无法使用虚幻编辑器自定义地图。 __您可以在 [此处](tuto_M_add_map_package.md)找到指南。__

如果您使用的是 Carla __源代码构建__ 版本，可以使用三种方法导入地图：

1. 使用自动 `make import` 流程（推荐）。  __您可以在 [此处](tuto_M_add_map_source.md)找到指南。__
2. 使用 RoadRunner 插件。 __您可以在 [此处](tuto_M_add_map_alternative.md#roadrunner-plugin-import)找到指南。__
3. 手动将地图导入到虚幻引擎中。 __您可以在 [此处](tuto_M_add_map_alternative.md#manual-import)找到指南。__

以下视频介绍了将地图导入 Carla 的一些可用方法：

<iframe width="560" height="315" src="https://www.youtube.com/embed/mHiUUZ4xC9o" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<br>

---

## 定制 <span id="customization"></span>

除了数百个准备添加到景观中的静态网格物体外，Carla 还提供了多种工具和指南来帮助您自定义地图：

- __添加子关卡：__ 子关卡将允许多人同时在同一张地图上工作。它们还允许您使用 Python API 切换地图的图层，就像 Carla 分层地图一样。 __您可以在 [此处](tuto_M_custom_layers.md)找到指南。__
- __设置默认天气：__ 尝试不同的天气预设，找到正确的组合后，为您的地图设置默认天气。 __您可以在 [此处](tuto_M_custom_weather_landscape.md#weather-customization)找到指南。__
- __填充景观：__ 使用蓝图通过重复网格（例如路灯、电线和墙壁）填充景观。 __您可以在 [此处](tuto_M_custom_weather_landscape.md#add-serial-meshes)找到指南。__
- __绘制道路：__ 使用混合不同纹理的主材质绘制道路。添加贴花和网格，例如落叶、裂缝或沙井。 __您可以在 [此处](tuto_M_custom_road_painter.md)找到指南。__
- __添加程序化建筑物：__ 使用程序化建筑蓝图添加具有自定义尺寸、楼层数量和可变网格组合的建筑物。  __您可以在 [此处](tuto_M_custom_buildings.md)找到指南。__
- __添加交通灯和交通标志：__ 添加交通灯和标志并配置其影响区域。路口的红绿灯组。 __您可以在 [此处](tuto_M_custom_add_tl.md)找到指南。__

---

## 生成行人导航 <span id="generate-pedestrian-navigation"></span>

为了生成行人并在地图上导航，您需要使用 Carla 提供的工具生成行人导航信息。行人导航应在完成地图自定义后生成，以便不会在导航路径上方创建障碍物。 __您可以在 [此处](tuto_M_generate_pedestrian_navigation.md)找到指南。__

---

如果您对上述过程有任何疑问，请随时在 [讨论页面](https://github.com/OpenHUTB/carla_doc/issues) 中发布。
