# 教程

在这里您将找到大量教程，帮助您了解如何使用 Carla 的众多功能。

## 一般

### 交通仿真系统

[**目标检测**](course/object_detection.md) — 通过传感器数据来实时识别Carla环境中的物体和车辆。

[**轨迹跟踪**](course/trajectory_tracking.md) — 利用控制算法使车辆沿预定路线准确行驶。

[**交通指标计算**](course/traffic_indicators.md) — 路口交通流量、路口车均延误、路口饱和、排队长度等多种路口真实性评价指标分析。

[**车辆导航**](course/navigation.md) —  自定义车辆速度，从指定起点运动到目标位置，并生成轨迹。

[**多摄像头配置**](course/multi-view_camera.md) — 支持配置单个或多个摄像头，进行车辆的检测。

[**路口微观**](course/microscopic_Intersection_demo.md) — 切换视角观察不同路口的交通状况。

[**车辆选点移动**](course/locate_moving.md) — 点击地图任意可生成车辆位置生成车辆并移动。

[**路径规划**](course/motion_planning.md) — 利用路径规划算法控制车辆避开障碍物，找到安全可行路线的方法。

[**RoadRunner道路场景建模**](course/scenario.md) — 设计用于模拟和测试自动驾驶系统的 3D 场景，创建要导入 Carla 的大地图。

[**智能信号控制**](course/signal_control.md) — 优化交通信号的配时方案，提高交通流畅度和减少拥堵。

### Carla 特点

[__检索仿真数据__](tuto_G_retrieve_data.md) — 使用记录器正确收集数据的分步指南。

[__交通管理器__](tuto_G_traffic_manager.md) — 如何使用交通管理器来引导城镇周围的交通。

[__纹理流__](tuto_G_texture_streaming.md) — 实时修改地图对象的纹理以添加变化。

[__实例分割相机__](tuto_G_instance_segmentation_sensor.md) — 使用实例分割相机来区分同一类的对象。

[__边界框__](tuto_G_bounding_boxes.md) — 将  Carla 对象的边界框投影到相机中。  

[__行人导航__](tuto_G_pedestrian_navigation.md) — 将行人骨骼投影到相机平面中。

[__控制行人骨骼__](tuto_G_control_walker_skeletons.md) — 使用骨骼为行人制作动画。


### 构建与集成

[__在 Docker 中构建虚幻引擎和 Carla__](build_docker_unreal.md) — 在 Docker 中构建虚幻引擎和 Carla。

[__CarSim 集成__](tuto_G_carsim_integration.md) — 有关如何使用 CarSim 车辆动力学引擎运行仿真的教程。

[__RLlib 集成__](tuto_G_rllib_integration.md) — 了解如何使用 RLlib 库运行您自己的实验。

[__Chrono 集成__](tuto_G_chrono.md) —  使用 Chrono 集成来仿真物理。

[__PyGame 控制__](tuto_G_pygame.md) — 使用 PyGame 显示相机传感器的输出。


## 资产和地图

[__使用 OpenStreetMap 生成地图__](tuto_G_openstreetmap.md) — 使用 OpenStreetMap 生成用于仿真的地图。 

[__添加新车辆__](tuto_A_add_vehicle.md) — 准备要在 Carla 中使用的车辆。

[__添加新道具__](tuto_A_add_props.md) — 将其他道具导入 Carla。

[__创建独立包__](tuto_A_create_standalone.md) — 生成并处理资产的独立包。 

[__材料定制__](tuto_A_material_customization.md) — 编辑车辆和建筑材料。


## 开发者

[__如何升级内容__](tuto_D_contribute_assets.md) —  向 Carla 添加新内容。

[__创建传感器__](tuto_D_create_sensor.md) — 开发一种用于 Carla 的新传感器。 

[__创建语义标签__](tuto_D_create_semantic_tags.md) — 定义用于语义分割的自定义标签。  

[__定制车辆悬架__](tuto_D_customize_vehicle_suspension.md) —  修改车辆的悬架系统。  

[__生成详细的碰撞器__](tuto_D_generate_colliders.md) — 为车辆创建详细的碰撞器。  

[__发布版本__](tuto_D_make_release.md) — 如何发布 Carla。


## 视频教程

[__基础知识__](https://www.youtube.com/watch?v=pONr1R1dy88) — 了解 Carla 的基本概念并开始您的第一个脚本。 [__代码__](https://carla-releases.s3.eu-west-3.amazonaws.com/Docs/Fundamentals.ipynb)  

[__深入了解 Carla 的传感器__](https://www.youtube.com/watch?v=om8klsBj4rc) — 深入了解 Carla 的传感器及其使用方法。[__代码__](https://carla-releases.s3.eu-west-3.amazonaws.com/Docs/Sensors_code.zip)
