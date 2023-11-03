title: 主页

# CARLA 文档

欢迎使用 CARLA 文档。

此主页包含一个索引，其中简要说明了文档中的不同部分。随意按喜欢的顺序阅读。无论如何，这里有一些给新手的建议。

* __安装 CARLA.__ 要么按照[快速开始安装](start_quickstart.md) 获得CARLA 版本，要么按照所需平台[进行构建](build_linux.md)。
* __开始使用 CARLA.__ 标题为 [第一步](core_concepts.md) 的部分介绍了最重要的概念。
* __检查 API.__ 有一个方便的 [Python API 参考](python_api.md) 来查找可用的类和方法。

CARLA 论坛可以发布在阅读过程中可能出现的任何疑问或建议。
<div class="build-buttons">
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="转到最新的 CARLA 版本">
CARLA论坛</a>
</div>

<br>

！！！警告
    __更改文档版本以适合您使用的 CARLA 版本__。使用此窗口右下角的面板更改为以前的版本。 __最新版本指向 `dev` 分支__ 中的文档，这可能是指当前正在开发的功能，并且在任何打包版本的 CARLA 中__不可用__，以及一般文档改进。 ![docs_version_panel](img/docs_version_panel.jpg)

---

## 入门

[__介绍__](start_introduction.md) — 对 CARLA 的期望。

[__快速启动包安装__](start_quickstart.md) — 获取 CARLA 版本。


## 构建CARLA

[__Linux build__](build_linux.md) — 在 Linux 上进行构建。

[__Windows build__](build_windows.md) — 在 Windows 上进行构建。

[__Update CARLA__](build_update.md) — 了解最新内容。

[__Build system__](build_system.md) — 了解构建及其制作方式。

[__CARLA in Docker__](build_docker.md) — 使用容器解决方案运行 CARLA。

[__F.A.Q.__](build_faq.md) — 一些最常见的安装问题。

## 第一步

[__核心概念__](core_concepts.md) — CARLA 中基本概念的概述。

[__第一、 世界和客户端__](core_world.md) — 管理和访问模拟。

[__第二、 角色和蓝图__](core_actors.md) — 了解角色以及如何处理它们。

[__第三、地图和导航__](core_map.md) — 发现不同的地图以及车辆如何移动。

[__第四、 传感器和数据__](core_sensors.md) — 使用传感器检索模拟数据。

## 高级概念

[__OpenDRIVE 独立模式__](adv_opendrive.md) — 使用任何 OpenDRIVE 文件作为 CARLA 地图。

[__PTV-Vissim 联合仿真__](adv_ptv.md) - 在 CARLA 和 PTV-Vissim 之间运行同步仿真。

[__Recorder__](adv_recorder.md) — 在模拟中录制事件并再次播放。

[__渲染选项__](adv_rendering_options.md) — 从质量设置到不渲染或离屏模式。

[__RSS__](adv_rss.md) — CARLA 客户端库中的 RSS 实现。

[__同步和时间步长__](adv_synchrony_timestep.md) — 客户端-服务器通信和模拟时间。

[__基准性能__](adv_benchmarking.md) — 使用我们准备好的脚本执行基准测试。

[__CARLA 代理__](adv_agents.md) — 代理脚本允许单个车辆在地图上漫游或开车到设定的目的地。

## 交通模拟

[__交通模拟概述__](ts_traffic_simulation_overview.md) — 可用于使用交通填充场景的不同选项的概述

[__Traffic Manager__](adv_traffic_manager.md) — 通过将车辆设置为自动驾驶模式来模拟城市交通。

[__SUMO 联合仿真__](adv_sumo.md) — 在 CARLA 和 SUMO 之间运行同步仿真。

[__Scenic__](tuto_G_scenic.md) — 遵循使用 Scenic 库定义不同场景的示例。

## 参考

[__Python API 参考__](python_api.md) — Python API 中的类和方法。

[__Blueprint library__](bp_library.md) — 提供用于生成 actor 的蓝图。

[__C++ 参考__](ref_cpp.md) — CARLA C++ 中的类和方法。

[__Recorder 二进制文件格式__](ref_recorder_binary_file_format.md) — 记录器文件格式的详细说明。

[__Sensors reference__](ref_sensors.md) — 关于传感器及其检索数据的一切。

## 插件

[__carlaviz — web 可视化器__](plugins_carlaviz.md) — 侦听模拟并在网络浏览器中显示场景和一些模拟数据的插件。

## ROS 桥接器

[__ROS 桥文档__](ros_documentation.md) — ROS 桥的简要概述和完整文档的链接

## 自定义地图

[__CARLA 中自定义地图的概述__](tuto_M_custom_map_overview.md) — 添加自定义标准尺寸地图所涉及的过程和选项的概述

[__在 RoadRunner 中创建地图__](tuto_M_generate_map.md) — 如何在 RoadRunner 中生成海关、标准尺寸的地图

[__ 在CARLA包导入地图__](tuto_M_add_map_package.md) 如何在CARLA包中导入地图

[__在 CARLA 源构建中导入地图__](tuto_M_add_map_source.md) — 如何在 CARLA 中导入从源构建的地图

[__导入地图的替代方法__](tuto_M_add_map_alternative.md) — 导入地图的替代方法

[__手动准备地图包__](tuto_M_manual_map_package.md) — 如何准备地图以供手动导入

[__自定义地图：分层地图__](tuto_M_custom_layers.md) — 如何在自定义地图中创建子图层

[__自定义地图：红绿灯和标志__](tuto_M_custom_add_tl.md) — 如何将红绿灯和标志添加到您的自定义地图

[__自定义地图：Road painter__](tuto_M_custom_road_painter.md) — 如何使用road painter工具改变道路的外观

[__自定义地图：程序建筑__](tuto_M_custom_buildings.md) — 用建筑物填充您的自定义地图

[__自定义地图：天气和景观__](tuto_M_custom_weather_landscape.md) — 为您的自定义地图创建天气配置文件并填充景观

[__生成行人导航__](tuto_M_generate_pedestrian_navigation.md) — 获取行人四处走动所需的信息。

## 大型地图

[__大型地图概述__](large_map_overview.md) — CARLA 中大型地图工作原理的说明

[__在 RoadRunner 中创建大地图__](large_map_roadrunner.md) — 如何在 RoadRunner 中创建大地图

[__导入/打包大地图__](large_map_import.md) — 如何导入大地图

## 教程 — 通用

[__添加摩擦触发器__](tuto_G_add_friction_triggers.md) - 定义车轮的动态框触发器

[__控制车辆物理模型__](tuto_G_control_vehicle_physics.md) - 设置车辆物理的运行时的变化

[__控制行人骨骼__](tuto_G_control_walker_skeletons.md) — 使用骨骼为步行者设置动画

[__使用 OpenStreetMap 生成地图__](tuto_G_openstreetmap.md) — 使用 OpenStreetMap 生成用于模拟的地图。

[__检索模拟数据__](tuto_G_retrieve_data.md) — 使用记录器正确收集数据的分步指南

[__CarSim 集成__](tuto_G_carsim_integration.md) - 关于如何使用 CarSim 车辆动力学引擎运行模拟的教程

[__RLlib 集成__](tuto_G_rllib_integration.md) — 了解如何使用 RLlib 库运行您自己的实验

[__Chrono 集成__](tuto_G_chrono.md) — 使用 Chrono 积分来模拟物理

[__在 Docker 中构建虚幻引擎UE和 CARLA__](build_docker_unreal.md) — 在 Docker 中构建虚幻引擎UE和 CARLA

## 教程 — 资产

[__添加新车辆__](tuto_A_add_vehicle.md) — 准备要在 CARLA 中使用的车辆

[__添加新道具__](tuto_A_add_props.md) — 将其他道具导入 CARLA

[__创建独立包__](tuto_A_create_standalone.md) — 为资产生成和处理独立包

[__材料定制__](tuto_A_material_customization.md) - 编辑车辆和建筑材料


## 教程 — 开发人员

[__如何升级内容__](tuto_D_contribute_assets.md) — 向 CARLA 添加新内容

[__创建一个传感器__](tuto_D_create_sensor.md) — 开发一个用于 CARLA 的新传感器

[__创建语义标签__](tuto_D_create_semantic_tags.md) — 为语义分割定义自定义标签

[__自定义车辆悬架__](tuto_D_customize_vehicle_suspension.md) — 修改车辆的悬架系统

[__生成详细碰撞__](tuto_D_generate_colliders.md) — 为车辆创建详细的对撞

[__发布版本__](tuto_D_make_release.md) — 如何发布 CARLA

## CARLA 生态系统

[__Ansys 实时雷达模型__](ecosys_ansys.md) — 有关 Ansys RTR 网络研讨会的详细信息

## 贡献

[__贡献指南__](cont_contribution_guidelines.md) — 为 CARLA 做出贡献的不同方式。

[__行为准则__](cont_code_of_conduct.md) — 贡献者的标准权利和义务。

[__编码标准__](cont_coding_standard.md) — 编写正确代码的指南。

[__文档标准__](cont_doc_standard.md) — 编写适当文档的指南。