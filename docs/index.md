title: 主页

# [Carla 文档](https://carla.readthedocs.io/en/latest/)

欢迎使用 Carla 文档。

此主页包含一个索引，其中简要说明了文档中的不同部分。随意按喜欢的顺序阅读。无论如何，这里有一些给新手的建议。

* __安装 Carla.__ 要么按照[快速开始安装](start_quickstart.md) 获得 Carla 版本，要么按照所需平台[进行构建](build_linux.md)。
* __开始使用 Carla.__ 标题为 [第一步](core_concepts.md) 的部分介绍了最重要的概念。
* __检查 API.__ 有一个方便的 [Python API 参考](python_api.md) 来查找可用的类和方法。

Carla 论坛可以发布在阅读过程中可能出现的任何疑问或建议。
<div class="build-buttons">
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="转到最新的 Carla 版本">
Carla 论坛</a>
</div>

<br>

!!! 警告
    __更改文档版本以适合您使用的 Carla 版本__。使用此窗口右下角的面板更改为以前的版本。 __最新版本指向 `dev` 分支__ 中的文档，这可能是指当前正在开发的功能，并且在任何打包版本的 Carla 中 __不可用__ ，以及一般文档改进。 ![docs_version_panel](img/docs_version_panel.jpg)

---

## 入门

[__介绍__](start_introduction.md) — 对 Carla 的期望。

[__快速启动包安装__](start_quickstart.md) — 获取 Carla 版本。

[__第一步__](tuto_first_steps.md) — 开始进行 Carla 操作。

[__教程__](tutorials.md) — Carla 详细教程。


## 构建 Carla

[__构建 Carla__](build_carla.md) — 进行 Carla 编译。

[__Linux 上编译__](build_linux.md) — 在 Linux 上进行编译。

[__Windows 上编译__](build_windows.md) — 在 Windows 上进行构建。

[__Carla 更新__](build_update.md) — 了解最新内容。

[__构建系统__](build_system.md) — 了解构建及其制作方式。

[__Docker 中的 Carla__](build_docker.md) — 使用容器解决方案运行 Carla。

[__常见问题__](build_faq.md) — 一些最常见的安装问题。

## Carla 主题

[__基础__](foundations.md) — Carla 基础。

[__核心概念__](core_concepts.md) — Carla 中基本概念的概述。

[__第一、 世界和客户端__](core_world.md) — 管理和访问仿真。

[__第二、 参与者和蓝图__](core_actors.md) — 了解角色以及如何处理它们。

[__第三、地图和导航__](core_map.md) — 发现不同的地图以及车辆如何移动。

[__第四、 传感器和数据__](core_sensors.md) — 使用传感器检索仿真数据。

[__开发__](ts_traffic_simulation_overview.md) — 基于 Carla 的开发。

[__自定义资产__](custom_assets_tutorials.md) — Carla 中资产的的开发。

[__内容创作 - 地图__](ts_traffic_simulation_overview.md) — 自定义地图的创作。

[__内容创作 - 车辆__](tuto_content_authoring_vehicles.md) — 自定义车辆的创作。


## 交通仿真

[__交通仿真概述__](ts_traffic_simulation_overview.md) — 可用于使用交通填充场景的不同选项的概述

[__交通管理器__](adv_traffic_manager.md) — 通过将车辆设置为自动驾驶模式来仿真城市交通。

[__和 SUMO 联合仿真__](adv_sumo.md) — 在 Carla 和 SUMO 之间运行同步仿真。

[__Scenic__](tuto_G_scenic.md) — 遵循使用 Scenic 库定义不同场景的示例。


## 高级概念

[__OpenDRIVE 独立模式__](adv_opendrive.md) — 使用任何 OpenDRIVE 文件作为 Carla 地图。

[__记录器__](adv_recorder.md) — 在仿真中录制事件并再次播放。

[__渲染选项__](adv_rendering_options.md) — 从质量设置到不渲染或离屏模式。

[__责任敏感安全__](adv_rss.md) — Carla 客户端库中的责任敏感安全实现。

[__同步和时间步长__](adv_synchrony_timestep.md) — 客户端-服务器通信和仿真时间。

[__基准性能__](adv_benchmarking.md) — 使用我们准备好的脚本执行基准测试。

[__Carla 智能体__](adv_agents.md) — 智能体脚本允许单个车辆在地图上漫游或开车到设定的目的地。



## 参考

[__Python API 参考__](python_api.md) — Python API 中的类和方法。

[__C++ 参考__](ref_cpp.md) — Carla C++ 中的类和方法。

[__蓝图库__](bp_library.md) — 提供用于生成参与者的蓝图。

[__Carla 目录__](catalogue.md) — Carla 中资源的目录列表。

[__记录器二进制文件格式__](ref_recorder_binary_file_format.md) — 记录器文件格式的详细说明。

[__传感器参考__](ref_sensors.md) — 关于传感器及其检索数据的一切。

[__扩展文档__](ext_docs.md) — 其他。


## 自定义地图

[__Carla 中自定义地图的概述__](tuto_M_custom_map_overview.md) — 添加自定义标准尺寸地图所涉及的过程和选项的概述

[__在 RoadRunner 中创建地图__](tuto_M_generate_map.md) — 如何在 RoadRunner 中生成海关、标准尺寸的地图

[__在 Carla 包导入地图__](tuto_M_add_map_package.md) 如何在 Carla 包中导入地图

[__在 Carla 源构建中导入地图__](tuto_M_add_map_source.md) — 如何在 Carla 中导入从源构建的地图

[__导入地图的替代方法__](tuto_M_add_map_alternative.md) — 导入地图的替代方法

[__手动准备地图包__](tuto_M_manual_map_package.md) — 如何准备地图以供手动导入

[__自定义地图：分层地图__](tuto_M_custom_layers.md) — 如何在自定义地图中创建子图层

[__自定义地图：红绿灯和标志__](tuto_M_custom_add_tl.md) — 如何将红绿灯和标志添加到您的自定义地图

[__自定义地图：道路画家__](tuto_M_custom_road_painter.md) — 如何使用道路画家工具改变道路的外观

[__自定义地图：程序建筑__](tuto_M_custom_buildings.md) — 用建筑物填充您的自定义地图

[__自定义地图：天气和景观__](tuto_M_custom_weather_landscape.md) — 为您的自定义地图创建天气配置文件并填充景观

[__生成行人导航__](tuto_M_generate_pedestrian_navigation.md) — 获取行人四处走动所需的信息。

## 大地图

[__大地图概述__](large_map_overview.md) — Carla 中大地图工作原理的说明

[__在 RoadRunner 中创建大地图__](large_map_roadrunner.md) — 如何在 RoadRunner 中创建大地图

[__导入/打包大地图__](large_map_import.md) — 如何导入大地图


## 教程（通用）

[__添加摩擦触发器__](tuto_G_add_friction_triggers.md) - 定义车轮的动态框触发器

[__控制车辆物理模型__](tuto_G_control_vehicle_physics.md) - 设置车辆物理的运行时的变化

[__控制行人骨骼__](tuto_G_control_walker_skeletons.md) — 使用骨骼为步行者设置动画

[__使用 OpenStreetMap 生成地图__](tuto_G_openstreetmap.md) — 使用 OpenStreetMap 生成用于仿真的地图。

[__检索仿真数据__](tuto_G_retrieve_data.md) — 使用记录器正确收集数据的分步指南。

[__在 Docker 中构建虚幻引擎和 Carla__](build_docker_unreal.md) — 在 Docker 中构建虚幻引擎和 Carla


## 教程（资产）

[__添加新车辆__](tuto_A_add_vehicle.md) — 准备要在 Carla 中使用的车辆

[__添加新道具__](tuto_A_add_props.md) — 将其他道具导入 Carla

[__创建独立包__](tuto_A_create_standalone.md) — 为资产生成和处理独立包

[__材料定制__](tuto_A_material_customization.md) - 编辑车辆和建筑材料


## 教程（开发人员）

[__如何升级内容__](tuto_D_contribute_assets.md) — 向 Carla 添加新内容

[__创建一个传感器__](tuto_D_create_sensor.md) — 开发一个用于 Carla 的新传感器

[__创建语义标签__](tuto_D_create_semantic_tags.md) — 为语义分割定义自定义标签

[__自定义车辆悬架__](tuto_D_customize_vehicle_suspension.md) — 修改车辆的悬架系统

[__生成详细碰撞__](tuto_D_generate_colliders.md) — 为车辆创建详细的对撞

[__发布版本__](tuto_D_make_release.md) — 如何发布 Carla

## 示例

[__交通灯的配置和使用__](tuto_G_traffic_light.md) - 红绿灯的配置和使用

[__流量预测__](tuto_G_traffic_prediction.md) - 车流量预测


## Carla 生态系统

[__ISS 智能驾驶系统__](ecosys_iss.md) — 包含用于自动驾驶相关任务的传统和深度学习算法。

[__Web 可视化器__](plugins_carlaviz.md) — 侦听仿真并在网络浏览器中显示场景和一些仿真数据的插件。

[__ROS__](ros_documentation.md) — 机器人操作系统桥接器的简要概述和完整文档的链接。

[__有限元变形材料虚幻插件__](ecosys_femfx.md) — 用[有限元法用于变形材料物理的多线程 CPU 库](https://github.com/GPUOpen-Effects/FEMFX) 。实体对象表示为四面体单元的网格，每个单元都具有控制刚度、体积如何随变形变化以及发生断裂或塑性（永久）变形的应力限制的材料参数。该模型支持多种材料以及材料之间的相互作用。

[__CarSim 集成__](tuto_G_carsim_integration.md) - 关于如何使用 CarSim 车辆动力学引擎运行仿真的教程

[__RLlib 集成__](tuto_G_rllib_integration.md) — 了解如何使用 RLlib 库运行您自己的实验

[__Chrono 集成__](tuto_G_chrono.md) — 使用 Chrono 积分来仿真物理

[__OpenRadios 有限元仿真__](OpenRadioss/use_OpenRadioss_windows.md) — 针对动态负载下的高度非线性问题评估和优化产品性能。

[__PTV-Vissim 联合仿真__](adv_ptv.md) - 在 Carla 和 PTV-Vissim 之间运行同步仿真。

[__Ansys 实时雷达模型__](ecosys_ansys.md) — 有关 Ansys RTR 网络研讨会的详细信息。

[__Matlab 接口__](https://github.com/darkscyla/MATLAB-Carla-Interface) 使用 Python 和 ROS 在 Matlab 中调用 Python。

[__罗技 G29 方向盘__](tuto_G_G29_windows.md) — 使用罗技方向盘进行车辆控制和仿真对方向盘的反向控制。

## 贡献

[__贡献指南__](cont_contribution_guidelines.md) — 为 Carla 做出贡献的不同方式。

[__行为准则__](cont_code_of_conduct.md) — 贡献者的标准权利和义务。

[__编码标准__](cont_coding_standard.md) — 编写正确代码的指南。

[__文档标准__](cont_doc_standard.md) — 编写适当文档的指南。

## 其他
[RoadRunner](https://ww2.mathworks.cn/help/roadrunner/index.html) - 用于构建静态场景

[RoadRunner Scenario](https://ww2.mathworks.cn/help/roadrunner-scenario/index.html) - 用于构建动态交通场景。

[__Carla 中文站__](https://bbs.carla.org.cn/) — 中文交流社区。

[__虚幻引擎文档__](https://github.com/OpenHUTB/engine_doc) — 和虚幻引擎相关的操作教程。

[__待做列表__](待做列表) - 待完成的功能。

