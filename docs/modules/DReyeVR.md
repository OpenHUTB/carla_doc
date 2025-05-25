# 虚拟现实

## 1. 模块化架构与扩展性
* 工厂模式（DReyeVRFactory）

    通过继承ACarlaActorFactory，动态生成自定义车辆（如TeslaM3、Mustang66）和传感器（如EgoSensor）。工厂类根据配置文件中的参数（如车辆类型、蓝图路径）注册Actor定义，支持热加载和灵活扩展新车辆类型。

* 模块分离

    功能被划分为独立模块：

    * 车辆控制（EgoVehicle）：处理驾驶逻辑、输入映射、物理模拟。
    * 传感器处理（EgoSensor）：集成眼动追踪、焦点计算、帧捕获。
    * 用户界面（FlatHUD）：绘制 HUD 元素（文本、纹理、十字准星）。
    * 游戏模式（DReyeVRGameMode）：管理玩家、输入、重播和音效。

## 2. 配置驱动与灵活性
* 配置文件（ConfigFile类）

    使用INI格式存储参数（如车辆属性、声音设置、眼动追踪配置），支持运行时热加载。例如：
    ```cpp
    GeneralParams.Get<float>("CameraParams", "VignetteIntensity");
    ```
通过ConfigFileData比较录制与回放配置，确保一致性。

* 车辆参数动态加载

    车辆蓝图（如BP_TeslaM3）的参数通过配置文件加载，允许快速调整模型属性（如车轮数量、生成代数）。

## 3. 传感器与数据融合
* 眼动追踪集成（EgoSensor）

    利用SRanipal插件获取实时眼动数据（注视点、瞳孔直径），计算视锥体汇聚距离（Vergence），用于分析驾驶员注意力或生成调试可视化。

* 多传感器支持

    支持深度着色器（DepthShader）、语义分割（SemanticSegmentationShader）等后处理效果，通过CreatePostProcessingEffect动态组合渲染管线。


## 4. 虚拟现实（VR）与输入系统
* SteamVR集成（ADReyeVRPawn）

    初始化VR设备（如HTC Vive），设置跟踪原点（Eye或Floor），并处理头部旋转与位置同步。

  * 多输入设备支持

    * 键盘/手柄：通过SetupPlayerInputComponent绑定输入轴（如油门、转向）。

    * Logitech方向盘：支持力反馈、按钮映射，通过LogitechWheelUpdate处理物理输入。


## 5. 实时数据处理与可视化
* 帧捕获与回放（EgoSensor）

    使用USceneCaptureComponent2D截取高分辨率图像，支持多种后处理效果（如线性Gamma、自定义着色器），并将数据保存至磁盘供后续分析。
* 调试可视化

    在编辑器中绘制调试信息（如视线射线、碰撞点），通过DrawDebugLine和DrawDebugSphere辅助开发。

## 6. 性能优化与稳定性
* 异步线程处理

    眼动追踪数据通过定时器（TickEyeTracker）异步更新，避免阻塞主线程。

* 资源管理

    使用UTexture2D动态生成十字准星、方块纹理，减少内存占用。例如：
    ```
    GenerateSquareImage(ReticleSrc, ReticleSize, FColor::Red);
    ```

## 7. 与Carla模拟器的深度集成
* Actor生命周期管理

    通过ACarlaEpisode和FindEgoVehicleDefinition确保车辆与传感器的正确生成与销毁。

* 重播系统（CarlaReplayer）

    支持录制和回放驾驶数据，通过ReplayTick同步车辆状态与传感器输入。

## 总结
DReyeVR模块通过模块化设计、配置驱动和插件集成，构建了一个高度可定制的虚拟驾驶环境。其核心优势在于：

* 灵活性：通过工厂模式和配置文件快速适配不同车辆与场景。
* 沉浸感：集成VR设备和眼动追踪，提升用户体验。
* 可扩展性：支持自定义传感器和后处理效果，便于算法研究与原型开发。

未来改进方向可能包括优化眼动追踪性能、增加更多车辆物理模型，以及完善多平台输入支持。
