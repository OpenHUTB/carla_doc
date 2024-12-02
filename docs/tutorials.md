# 教程

在这里您将找到大量教程，帮助您了解如何使用 Carla 的众多功能。


## 一般

### 交通模拟系统

本系统基于开源虚幻引擎框架，构建全道路场景模拟和智能交通系统模拟系统，满足我国特定交通路况多样化需求，为智慧交通算法的教学与应用提供的数据底座。

- [交通数据导入和编辑支持从公开道路地图导入或创建生成道路网络](course/scenario.md#generateMapByOpenMap)，支持添加车辆到道路网络并定义其轨迹以合成驾驶场景，[支持定义车道数量和车道长宽度，支持自定义道路编辑](course/scenario.md#sceneEditing)。
- [车辆运动支持定义车辆运动起点和目标位置](course/navigation.md#definePoint)，[为每辆车指定随机速度值](course/navigation.md#definePoint)，[生成车辆轨迹](course/navigation.md#generateTrajectory)。支持自定义车辆的速度，使其沿轨迹移动时不会发生碰撞或模拟碰撞。支持车辆运动模拟和显示自动生成的轨迹场景，可以看到车辆沿指定轨迹行驶以到达其目标位置。

- 联合模拟支持联合虚拟引擎模拟模拟实验，不仅可以看到[区域宏观](course/regional_macro.md)，还可以看到[路口微观](course/microscopic_Intersection_demo.md)，支持 3D 动画演示运动场景。[支持配置单个或多个摄像头，进行车辆的检测](course/multi-view_camera.md)。[支持计算红绿灯的配时方案，并进行红绿灯的设置](course/auto_signal_control.md)。[支持测试车辆按地图选点进行移动，看到红灯停、绿灯行，以及避让等功能](course/locate_moving.md)。支持加入更多的车辆进行交通拥堵的模拟，统计优化前和优化后的结果。

##### 效果评价

1. [路口评价](course/traffic_indicators.md)：支持高保正的十字路口三维建模，建模[路口交通流量](course/traffic_indicators.md#trafficFlow)、[路口车均延误](course/traffic_indicators.md#aveDelay)、[路口饱和](course/traffic_indicators.md#saturation)、[排队长度](course/traffic_indicators.md#queueLength)等多种路口真实性评价指标分析。
2. [路网评价](course/roadnet_evaluation.md)：支持基于路网数据的路网动态模拟，[预测和分析不同模拟流量和真实流量的相似性](course/roadnet_evaluation.md#predict_flow)。

##### 课程一：交通模拟与场景建模

1. [道路网络导入与编辑学生学习如何使用系统导入公开道路地图或创建生成道路网络](course/scenario.md#generateMapByOpenMap)，以及[编辑道路属性和车道信息，构建具体的交通场景](course/scenario.md#sceneEditing)。
2. [车辆轨迹生成](course/navigation.md#generateTrajectory)与[模拟学生学习如何为车辆指定起点和目标位置](course/navigation.md#definePoint)，[定义车辆的运动轨迹，模拟车辆的行驶过程](course/trajectory_tracking.md)，并观察车辆在模拟环境中的运动行为。
3. [红绿灯配时方案优化学生学习如何配置红绿灯的配时方案](course/signal_control.md)，[并通过模拟实验优化配时参数](course/auto_signal_control.md)，以提高交通流畅度和减少交通拥堵。
4. [交通拥堵模拟与统计分析学生学习如何模拟交通拥堵场景](course/congestion_sim.md)，加入更多的车辆进行模拟，进行拥堵情况的统计分析，并评估优化措施的效果。
5. 路口交通流量评估学生学习如何使用系统提供的路口评价指标，分析[路口交通流量](course/traffic_indicators.md#trafficFlow)、[车均延误](course/traffic_indicators.md#aveDelay)、[饱和度](course/traffic_indicators.md#saturation)、[排队长度](course/traffic_indicators.md#queueLength)等指标，评估路口交通状况。
6. [场景建模与可视化展示学生学习如何使用系统提供的场景建模工具，进行交通场景的建模和可视化展示，以及设计交互式演示界面，呈现模拟结果和实验效果。](adv_digital_twin.md)

##### 课程二：智能交通算法与优化

1. 交通场景感知与分析学生学习如何利用系统进行[目标检测](course/object_detection.md)、[跟踪](course/trajectory_tracking.md)和[再识别](course/recognize.md)，进行交通参与者的分析和统计，对交通进行态势感知，为后面的算法实现提供基础。
2. [交通流量预测模型](tuto_G_traffic_prediction.md)建立学生学习如何使用系统提供的交通数据，建立交通流量预测模型，以预测未来的交通流量情况，并评估模型的准确性和效果。
3. 交通网络优化策略设计学生学习如何使用系统提供的交通网络数据，设计交通网络优化策略，通过调整[路线规划](course/motion_planning.md)、[红绿灯配时](course/signal_control.md)等参数，提升整体交通效率。
4. [智能信号控制](course/signal_control.md)算法实现学生学习如何使用系统提供的信号控制功能，实现智能信号控制算法，优化交通信号的配时方案，提高交通流畅度和减少拥堵。
5. [交通数据可视化](course/visualization.md)与分析学生学习如何使用系统提供的数据可视化工具，对交通数据进行可视化分析，探索交通流量、拥堵状况等趋势和规律。
6. 交通场景优化与模拟实验学生学习如何根据实际交通问题，进行场景优化和模拟实验，评估优化策略的效果，提出改进方案并进行验证。


### Carla 特点

[__检索模拟数据__](tuto_G_retrieve_data.md) — 使用记录器正确收集数据的分步指南。

[__交通管理器__](tuto_G_traffic_manager.md) — 如何使用交通管理器来引导城镇周围的交通。

[__纹理流__](tuto_G_texture_streaming.md) — 实时修改地图对象的纹理以添加变化。

[__实例分割相机__](tuto_G_instance_segmentation_sensor.md) — 使用实例分割相机来区分同一类的对象。

[__边界框__](tuto_G_bounding_boxes.md) — 将  Carla 对象的边界框投影到相机中。  

[__行人导航__](tuto_G_pedestrian_navigation.md) — 将行人骨骼投影到相机平面中。

[__控制行人骨骼__](tuto_G_control_walker_skeletons.md) — 使用骨骼为行人制作动画。


### 构建与集成

[__在 Docker 中构建虚幻引擎和 Carla__](build_docker_unreal.md) — 在 Docker 中构建虚幻引擎和 Carla。

[__CarSim 集成__](tuto_G_carsim_integration.md) — 有关如何使用 CarSim 车辆动力学引擎运行模拟的教程。

[__RLlib 集成__](tuto_G_rllib_integration.md) — 了解如何使用 RLlib 库运行您自己的实验。

[__Chrono 集成__](tuto_G_chrono.md) —  使用 Chrono 集成来模拟物理。

[__PyGame 控制__](tuto_G_pygame.md) — 使用 PyGame 显示相机传感器的输出。


## 资产和地图

[__使用 OpenStreetMap 生成地图__](tuto_G_openstreetmap.md) — 使用 OpenStreetMap 生成用于模拟的地图。 

[__添加新车辆__](tuto_A_add_vehicle.md) — 准备要在 Carla 中使用的车辆。

[__添加新道具__](tuto_A_add_props.md) — 将其他道具导入 Carla。

[__创建独立包__](tuto_A_create_standalone.md) — 生成并处理资产的独立包。 

[__材料定制__](tuto_A_material_customization.md) — 编辑车辆和建筑材料。


## 开发者

[__如何升级内容__](tuto_D_contribute_assets.md) —  向 Carla 添加新内容。

[__创建传感器__](tuto_D_create_sensor.md) — 开发一种用于 Carla 的新传感器。 

[__创建语义标签__](tuto_D_create_semantic_tags.md) — 定义用于语义分割的自定义标签。  

[__定制车辆悬架__](tuto_D_customize_vehicle_suspension.md) —  修改车辆的悬架系统。  

[__生成详细的碰撞体__](tuto_D_generate_colliders.md) — 为车辆创建详细的碰撞体。  

[__发布版本__](tuto_D_make_release.md) — 如何发布 Carla。


## 视频教程

[__基础知识__](https://www.youtube.com/watch?v=pONr1R1dy88) — 了解 Carla 的基本概念并开始您的第一个脚本。 [__代码__](https://carla-releases.s3.eu-west-3.amazonaws.com/Docs/Fundamentals.ipynb)  

[__深入了解 Carla 的传感器__](https://www.youtube.com/watch?v=om8klsBj4rc) — 深入了解 Carla 的传感器及其使用方法。[__代码__](https://carla-releases.s3.eu-west-3.amazonaws.com/Docs/Sensors_code.zip)
