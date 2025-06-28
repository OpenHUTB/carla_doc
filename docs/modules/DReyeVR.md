# 虚拟现实


## 1. 模块化架构与扩展性
* 工厂模式（DReyeVRFactory）

    通过继承ACarlaActorFactory，动态生成自定义主车（如TeslaM3、Mustang66）和主传感器（如EgoSensor）。工厂类根据配置文件中的参数（如车辆类型、蓝图路径）动态注册车辆类型，支持热加载和灵活扩展新车辆类型。处理键盘、手柄等输入设备的信号，传递给生成的 Actor。

* 模块分离

    功能被划分为独立模块：

    * 车辆控制（EgoVehicle）：处理驾驶逻辑、输入映射、物理模拟。
    * 传感器处理（EgoSensor）：集成眼动追踪、焦点计算、帧捕获。
    * 用户界面（FlatHUD）：绘制 HUD 元素（文本、纹理、十字准星）。
    * 游戏模式（ [DReyeVRGameMode](VR/DReyeVRGameMode.md) ）：管理玩家、输入、重播和音效。


* 增加新类型车（默认支持特斯拉Model3、[福特野马](https://baike.baidu.com/item/%E7%A6%8F%E7%89%B9%E9%87%8E%E9%A9%AC/8441335) 、吉普、踏板车 [Vespa](https://baike.baidu.com/item/Vespa/791252) ）的操作步骤

    * 在`DReyeVRFactor.h`的`VehicleTypes`列表中增加新的自定义自车类型；

    * 同步新增加车的配置文件（`Config/EgoVehicle/XYZ.ini`）和蓝图（`Content/DReyeVR/EgoVehicles/XYZ/BP_XYZ.uasset`）


## 2. 配置驱动与灵活性

* 配置文件类（`ConfigFile`类）

    使用 INI格式 存储参数（如车辆属性、声音设置、眼动追踪配置），支持运行时热加载。例如：
    ```cpp
    GeneralParams.Get<float>("CameraParams", "VignetteIntensity");
    ```
通过 ConfigFileData 比较记录与回放配置，确保一致性。

* `DefaultEngine.ini` 配置文件
    VR 模式
    ```shell
    [/Script/EngineSettings.GameMapsSettings]
    EditorStartupMap=/Game/Carla/Maps/Town03.Town03
    LocalMapOptions=
    TransitionMap=/Game/Carla/Maps/Town03.Town03
    bUseSplitscreen=True
    TwoPlayerSplitscreenLayout=Horizontal
    ThreePlayerSplitscreenLayout=FavorTop
    FourPlayerSplitscreenLayout=Grid
    bOffsetPlayerGamepadIds=False
    GameInstanceClass=/Script/Carla.CarlaGameInstance
    GlobalDefaultGameMode=/Script/CarlaUE4.DReyeVRGameMode
    GlobalDefaultServerGameMode=/Script/CarlaUE4.DReyeVRGameMode
    ```

* 车辆参数动态加载

    * 车辆蓝图（如BP_TeslaM3）的参数通过配置文件加载，允许快速调整模型属性（如车轮数量、生成代数）。
    * 特斯拉M3的配置（`Config/EgoVehicle/TeslaM3.ini`）：所加载的蓝图路径、相对于车的相机位置和旋转、车辆中引擎（声音）的位置、仪表板（速度里程、转向信号、档位切换）、反光镜（后向镜、左视镜、右视镜。反光镜非常消耗资源，如果想获得更平滑的FPS，可以将 `XMirrorEnable` 标志设置为`False`。）


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
