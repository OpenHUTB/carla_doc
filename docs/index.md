title: 主页

# [代理模拟器](https://carla.readthedocs.io/en/latest/)

欢迎使用代理模拟器文档，该页面包含所有内容的索引。运行 [入门](#primary) 示例后可以按自己喜欢的顺序随意阅读（可使用 [DeepSeek大模型](software/llm.md) 来辅助开发）。默认为打开 UE 4.26 的 Carla 文档，支持 UE 5.5 的 Carla 文档请查看 [链接](ue5/index.md) 。

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/carla_doc) 中的 [Issues](https://github.com/OpenHUTB/carla_doc/issues) 页面进行提问或者提交 [Pull Request](https://zhuanlan.zhihu.com/p/153381521) 直接修改文档。

* [__入门__](#primary)  
    * [简介](#introduction)
* [__代理__](#agent)  
    *   [车辆](#vehicle)  
    *   [行人](#pedestrian)  
* [__模拟器__](#simulator)
    *   [概念](#concept)
    *   [开发](#development) 
    *   [模块](#modules) 
* [__现实场景__](#scene)
    * [虚幻场景](#unreal_scene)
    * [地图孪生](#map_twin)
    * [资产孪生](#assert_twin) 
* [__其他__](#other)
    * [参考目录](#ref)
    * [生态系统](#ecology)
    * [基准测试](#test)
    * [贡献指南](#contribution)
    

---

## 入门 <span id="primary"></span>
1. 下载 [链接](https://pan.baidu.com/s/1n2fJvWff4pbtMe97GOqtvQ?pwd=hutb) 中的`software/carla_0.9.15`中的所有文件并解压；
2. 运行`WindowsNoEditor`文件夹下的`CarlaUE4.exe`启动场景，进入场景后按`W`、`S`、`A`、`D`移动场景视角；
3. 使用`Python37/python.exe`运行脚本 [generate_traffic.py](https://github.com/OpenHUTB/carla_doc/blob/master/src/examples/generate_traffic.py) 在场景中生成交通流和运行 [python manual_control.py](https://github.com/OpenHUTB/carla_doc/blob/master/src/examples/manual_control.py) 生成一辆键盘控制的车。


!!! 注意
    当机器性能一般时启动报错：`Out of video memory...`，可以通过命令来降低画质启动：`CarlaUE4.exe -quality-level=Low`，以获得更流畅的效果；甚至使用`CarlaUE4.exe -nullrhi`禁用所有渲染（无需 GPU）。如果运行CarlaUE4.exe时候报错：缺少Microsoft Visual C++ Runtime、DirectX Runtime，则需要安装 [vs_community__2019.exe](https://visualstudio.microsoft.com/zh-hans/vs/older-downloads/) （勾选`.NET桌面开发`和`使用C++的桌面开发`）和 [directx_Jun2010_redist.zip](https://www.microsoft.com/zh-CN/download/details.aspx?id=8109)  （解压后运行`DXSETUP.bat`）。



### 简介 <span id="introduction"></span>

[__介绍__](start_introduction.md) — 对 Carla 的期望

[__快速启动__](start_quickstart.md) — 获取 Carla 版本

[__第一步__](tuto_first_steps.md) — 开始进行 Carla 操作，介绍最重要的概念

[__示例__](tuto_E_gallery.md) — Carla 经典示例

[__教程__](tutorials.md) — Carla 详细教程


<!-- ## 主题 -->

[__基础__](foundations.md) — Carla 服务器和客户端进行操作和通信所需的基本概念

[__第一、 世界和客户端__](core_world.md) — 管理和访问模拟

[__第二、 参与者和蓝图__](core_actors.md) — 了解参与者以及如何处理它们

[__第三、地图和导航__](core_map.md) — 发现不同的地图以及车辆如何移动

[__第四、传感器和数据__](core_sensors.md) — 使用传感器检索模拟数据

[__检索模拟数据__](tuto_G_retrieve_data.md) — 使用记录器正确收集数据的分步指南

[__边界框__](tuto_G_bounding_boxes.md) — 将  Carla 对象的边界框投影到相机中

---
## 代理 <span id="agent"></span>

### 车辆 <span id="vehicle"></span>
<!-- 车辆代理：单个 -> 多个 -->

[__手动控制车辆__](tuto_G_pygame.md) — 使用 PyGame 显示相机传感器的输出

[__车辆代理__](adv_agents.md) — 代理脚本允许单个车辆在地图上漫游或开车到设定的目的地

[__智能驾驶__](ecosys_iss.md) — 包含用于自动驾驶相关任务的传统和深度学习算法

[__车辆实现__](vehicle/wheeled_vehicle.md) — 车辆模型

[__交通模拟概述__](ts_traffic_simulation_overview.md) — 可用于使用交通填充场景的不同选项的概述

[__交通管理器__](adv_traffic_manager.md) — 通过将车辆设置为自动驾驶模式来模拟城市交通

[__交通场景__](scenario_runner.md) — 基于 Carla 交通场景定义和执行引擎

[__交通场景定义__](tuto_G_scenic.md) — 遵循使用 Scenic 库定义不同场景的示例

[__和 SUMO 协同模拟__](adv_sumo.md) — 在 Carla 和 SUMO 之间运行协同模拟



### 行人 <span id="pedestrian"></span>
<!-- 行人代理 -->

[__行人导航__](tuto_G_pedestrian_navigation.md) — 使用骨骼为行人设置动画

[__控制行人骨架__](tuto_G_control_walker_skeletons.md) — 使用骨骼为行人设置动画

[__行人视觉__](tuto_G_pedestrian_visual.md) — 使用相机进行人眼的模拟

[__自行车__](pedestrian/cycle.md) - 可提供最高程度的游戏内控制和反馈的自行车模拟器

[__生成行人导航__](tuto_M_generate_pedestrian_navigation.md) — 获取行人四处走动所需的信息

[__行人建模__](tuto_content_authoring_pedestrians.md) — 在三维建模软件中对人进行建模

[__行人物理场模拟__](pedestrian/tuto_content_chrono_opensim.md) — Chrono OpenSim 解析器

---


## 模拟器 <span id="simulator"></span>

### 概念 <span id="concept"></span>
<!-- Carla 原理 -->

[__OpenDRIVE 独立模式__](adv_opendrive.md) — 使用任何 OpenDRIVE 文件作为 Carla 地图

[__记录器__](adv_recorder.md) — 在模拟中记录事件并再次播放

[__实例分割相机__](tuto_G_instance_segmentation_sensor.md) — 使用实例分割相机来区分同一类的对象

[__渲染选项__](adv_rendering_options.md) — 从质量设置到不渲染或离屏模式

[__通过 API 更改纹理__](tuto_G_texture_streaming.md) — 实时修改地图对象的纹理以添加变化

[__责任敏感安全__](adv_rss.md) — Carla 客户端库中的责任敏感安全实现

[__同步和时间步长__](adv_synchrony_timestep.md) — 客户端-服务器通信和模拟时间

[__多 GPU__](adv_multigpu.md) — 设置 Carla 模拟器以使用多个 GPU 进行处理

[__Carla 配置__](carla_settings.md) — Carla 参数配置设置

[__Carla 模拟器键盘输入__](simulator_keyboard_input.md) — 在模拟器窗口中玩游戏时使用的按键绑定

[__无显示屏运行 Carla 并选择 GPU__](carla_headless.md) — 使用未插在屏幕上的GPU来渲染 Carla 的桌面



### 开发 <span id="development"></span>
<!-- Carla 开发 -->

[__编译 Carla__](build_carla.md) — 进行 Carla 编译

[__Linux 上编译__](build_linux.md) — 在 Linux 上进行编译

[__Windows 上编译__](build_windows.md) — 在 Windows 上进行构建

[__Carla 更新__](build_update.md) — 了解最新内容

[__构建系统__](build_system.md) — 了解构建及其制作方式

[__Docker 中的 Carla__](build_docker.md) — 使用容器解决方案运行 Carla

[__在 Docker 中构建虚幻引擎和 Carla__](build_docker_unreal.md) — 在 Docker 中构建虚幻引擎和 Carla

[__常见问题__](build_faq.md) — 一些最常见的安装问题

[__调试程序__](tuto_D_windows_debug.md) — 进行C++程序的调试


### 模块 <span id="modules"></span>

[__文件说明__](file_specification.md) — 说明源代码中各个模块、文件的作用

[__行为交互__](interbehavior.md) — 针对行为和交互研究的虚拟现实驾驶仿真器

[__罗技 G29 方向盘__](tuto_G_G29_windows.md) — 使用罗技方向盘进行车辆控制和模拟对方向盘的反向控制

[__地理空间生态系统__](adv_cesium.md) — 将 3D 地理空间生态系统引入虚幻引擎

[__Chrono 集成__](tuto_G_chrono.md) — 整合 Chrono 来进行物理模拟

---


## 现实场景  <span id="scene"></span>

### 虚幻场景 <span id="unreal_scene"></span>

[__虚幻引擎入门__](ue/ue_faq.md) - 包括学习流程、概念说明、常见问题和回答等

[__支持的虚幻场景__](core_map.md#non-layered-maps) - 目前支持的所有虚幻地图

[__Carla 插件__](ue/plugin.md) - 虚幻引擎 Carla 插件的相关介绍

[__虚幻引擎高级特性__](tuto_G_unreal.md) — 虚幻引擎相关内容

[__第三方资产__](ue/scene_related.md) - 虚幻引擎相关的场景和资产


### 地图孪生 <span id="map_twin"></span>
<!-- 地图、建筑等静态资产 -->

[__数字孪生工具__](adv_digital_twin.md) — 通过 OpenStreetMap 自动生成 Carla 中的道路和建筑

[__Carla 中自定义地图的概述__](tuto_M_custom_map_overview.md) — 添加自定义标准尺寸地图所涉及的过程和选项的概述

[__内容创作-地图__](tuto_content_authoring_maps.md) — 自定义地图的创作

[__在 RoadRunner 中创建地图__](tuto_M_generate_map.md) — 如何在 RoadRunner 中生成海关、标准尺寸的地图

[__在 Carla 包导入地图__](tuto_M_add_map_package.md) 如何在 Carla 包中导入地图

[__在 Carla 源构建中导入地图__](tuto_M_add_map_source.md) — 如何在 Carla 中导入从源构建的地图

[__导入地图的替代方法__](tuto_M_add_map_alternative.md) — 导入地图的替代方法

[__手动准备地图包__](tuto_M_manual_map_package.md) — 如何准备地图以供手动导入

[__自定义地图：分层地图__](tuto_M_custom_layers.md) — 如何在自定义地图中创建子图层

[__自定义地图：交通灯和标志__](tuto_M_custom_add_tl.md) — 如何将交通灯和标志添加到您的自定义地图

[__自定义地图：道路画家__](tuto_M_custom_road_painter.md) — 如何使用道路画家工具改变道路的外观

[__自定义地图：天气和景观__](tuto_M_custom_weather_landscape.md) — 为您的自定义地图创建天气配置文件并填充景观

[__使用 OpenStreetMap 生成地图__](tuto_G_openstreetmap.md) — 使用 OpenStreetMap 生成用于模拟的地图

[__自定义地图：程序建筑__](tuto_M_custom_buildings.md) — 用建筑物填充您的自定义地图

[__程序化建筑工具__](adv_procedural_building_tool.md) 有助于生成虚拟三维建筑，可以通过简单的界面对其尺寸和装饰风格进行调制，以创建近乎无限的变化

[__大地图概述__](large_map_overview.md) — Carla 中大地图工作原理的说明

[__在 RoadRunner 中创建大地图__](large_map_roadrunner.md) — 如何在 RoadRunner 中创建大地图

[__导入/打包大地图__](large_map_import.md) — 如何导入大地图


### 资产孪生 <span id="assert_twin"></span>
<!-- 开发资产 -->

[__开发__](development_tutorials.md) — 创建自定义功能或内容

[__自定义资产__](custom_assets_tutorials.md) — Carla 中资产的的开发

[__添加摩擦触发器__](tuto_G_add_friction_triggers.md) — 定义车轮的动态框触发器

[__控制车辆物理模型__](tuto_G_control_vehicle_physics.md) — 设置车辆物理的运行时的变化

[__添加新车辆__](tuto_A_add_vehicle.md) — 准备要在 Carla 中使用的车辆

[__内容创作-车辆__](tuto_content_authoring_vehicles.md) — 自定义车辆的创作

[__添加新道具__](tuto_A_add_props.md) — 将其他道具导入 Carla

[__创建独立包__](tuto_A_create_standalone.md) — 为资产生成和处理独立包

[__材料定制__](tuto_A_material_customization.md) — 编辑车辆和建筑材料

[__如何升级内容__](tuto_D_contribute_assets.md) — 向 Carla 添加新内容

[__创建一个传感器__](tuto_D_create_sensor.md) — 开发一个用于 Carla 的新传感器

[__创建语义标签__](tuto_D_create_semantic_tags.md) — 为语义分割定义自定义标签

[__自定义车辆悬架__](tuto_D_customize_vehicle_suspension.md) — 修改车辆的悬架系统

[__生成详细碰撞__](tuto_D_generate_colliders.md) — 为车辆创建详细的对撞

[__链接 Epic 的汽车材质__](epic_automotive_materials.md) — 下载材质并将其链接到我们的车辆，以获得更逼真的车漆。

---

## 其他 <span id="other"></span>

<!-- 其他 --> 

### 参考目录 <span id="ref"></span>

[__Python API 参考__](python_api.md) — Python API 中的类和方法

[__C++ 参考__](ref_cpp.md) — Carla C++ 中的类和方法

[__资源目录__](catalogue.md) — Carla 中资源的目录列表

[__传感器__](ref_sensors.md) — 关于传感器及其检索数据的一切

[__蓝图库__](bp_library.md) — 提供用于生成参与者的蓝图

[__记录器二进制文件格式__](ref_recorder_binary_file_format.md) — 记录器文件格式的详细说明

[__测量和数据包__](measurements.md) - 描述服务器每帧向客户端发送一个包含测量值和收集到图像数据包的详细信息


### 生态系统 <span id="ecology"></span>

[__Web 可视化器__](plugins_carlaviz.md) — 监听模拟并在网络浏览器中显示场景和一些模拟数据的插件

[__ROS__](ros_documentation.md) — 机器人操作系统桥接器的简要概述和完整文档的链接

[__CarSim 集成__](tuto_G_carsim_integration.md) — 关于如何使用 CarSim 车辆动力学引擎运行模拟的教程

[__RLlib 集成__](tuto_G_rllib_integration.md) — 了解如何使用 RLlib 库运行您自己的实验

[__PTV-Vissim 联合模拟__](adv_ptv.md) — 在 Carla 和 PTV-Vissim 之间运行协同模拟

[__Ansys 实时雷达模型__](ecosys_ansys.md) — 有关 Ansys RTR 网络研讨会的详细信息

[__NVIDIA Omniverse 的 SimReady__](ecosys_simready.md) - 为模拟而构建的三维内容，简化模拟三维虚拟环境的内容创建管道

[__Blender 手册__](https://docs.blender.org/manual/zh-hans/latest/) — 三维建模软件

[__Carla 相关的第三方链接__](./third.md) — 中文交流社区

[__待做列表__](todo.md) — 待完成的功能

[__应用Carla的仓库列表__](used_by.md) - 收集引用了Carla的相关库


### 基准测试 <span id="test"></span>

[__自动驾驶排行榜__](leaderboard.md) - 评估自动驾驶人员在现实交通场景中的驾驶熟练程度

[__驾驶基准__](benchmark_start.md) — 用于评估驾驶控制器（代理）并获取有关其性能的指标

[__基准性能__](adv_benchmarking.md) — 分析 Carla 在自己的环境中的性能

[__驾驶基准结构__](benchmark_structure.md) — 驾驶基准模块的总体结构

[__测试代理__](benchmark_creating.md) — 对你的代理进行基准测试

[__计算的基准性能指标__](benchmark_metrics.md) — 用于根据代理在基准测试期间执行的操作计算结果摘要


### 贡献指南 <span id="contribution"></span>

[__贡献方式__](cont_contribution_guidelines.md) — 为 Carla 做出贡献的不同方式

[__行为准则__](cont_code_of_conduct.md) — 贡献者的标准权利和义务

[__编码标准__](cont_coding_standard.md) — 编写正确代码的指南

[__文档标准__](cont_doc_standard.md) — 编写适当文档的指南

[__发布版本__](tuto_D_make_release.md) — 如何发布 Carla


