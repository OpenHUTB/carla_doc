# CarlaUnreal 传感器模块详细说明文档

# 目录
- [概述](#概述)
- [文件详细说明](#文件详细说明)
  - [1. CollisionSensor.cpp](#1-collisionsensorcpp)
    - [文件路径](#文件路径)
    - [功能](#功能)
    - [代码结构](#代码结构)
    - [依赖关系](#依赖关系)
    - [使用方法](#使用方法)
    - [数据输出格式](#数据输出格式)
    - [性能和优化](#性能和优化)
  - [2. CustomV2XSensor.cpp](#2-customv2xsensorcpp)
    - [文件路径](#文件路径-1)
    - [功能](#功能-1)
    - [代码结构](#代码结构-1)
    - [依赖关系](#依赖关系-1)
    - [使用方法](#使用方法-1)
    - [数据输出格式](#数据输出格式-1)
    - [性能和优化](#性能和优化-1)
  - [3. DepthCamera.cpp](#3-depthcameracpp)
    - [文件路径](#文件路径-2)
    - [功能](#功能-2)
    - [代码结构](#代码结构-2)
    - [依赖关系](#依赖关系-2)
    - [使用方法](#使用方法-2)
    - [数据输出格式](#数据输出格式-2)
    - [性能和优化](#性能和优化-2)
  - [4. DVSCamera.cpp](#4-dvscameracpp)
    - [文件路径](#文件路径-3)
    - [功能](#功能-3)
    - [代码结构](#代码结构-3)
    - [依赖关系](#依赖关系-3)
    - [使用方法](#使用方法-3)
    - [数据输出格式](#数据输出格式-3)
    - [性能和优化](#性能和优化-3)
- [集成和扩展](#集成和扩展)
  - [依赖关系总结](#依赖关系总结)
  - [扩展建议](#扩展建议)

本文档详细介绍CarlaUnreal插件中传感器模块的四个具体实现文件：`CollisionSensor.cpp`、`CustomV2XSensor.cpp`、`DepthCamera.cpp` 和 `DVSCamera.cpp`。这些文件实现了Carla自动驾驶仿真器中不同类型的传感器功能，用于在Unreal Engine环境中模拟自动驾驶车辆的感知系统。文档旨在为开发者提供深入的技术参考，包括功能、实现逻辑、配置方法和使用场景。

## 概述

CarlaUnreal是基于Unreal Engine开发的开源自动驾驶仿真平台，其传感器模块是核心组件之一，负责模拟现实世界中的感知设备。这些传感器通过继承Unreal Engine的`AActor`或`ASceneCaptureSensor`类实现，支持与Carla的客户端-服务器架构集成，允许通过Python API或ROS2接口获取数据。

以下四个文件分别实现不同类型的传感器功能：

1. **CollisionSensor.cpp**：碰撞传感器，检测车辆或行人Actor的碰撞事件，记录碰撞物体、冲量等信息。
2. **CustomV2XSensor.cpp**：V2X通信传感器，模拟车辆间无线通信，支持消息生成和传播仿真。
3. **DepthCamera.cpp**：深度相机，捕获场景的深度信息，生成深度图像，适用于环境感知。
4. **DVSCamera.cpp**：动态视觉传感器（DVS），基于亮度变化生成事件流，适合高速动态场景。

## 文件详细说明

### 1. CollisionSensor.cpp

#### 文件路径
`~/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/CollisionSensor.cpp`

#### 功能
- **类**：`ACollisionSensor`（继承自传感器基类，推测为`ASensor`）。
- **作用**：检测附加到车辆或行人Actor的碰撞事件，记录碰撞的参与者（Actor）、冲量（NormalImpulse）和碰撞位置（HitResult），并将数据序列化后发送到客户端。
- **应用场景**：
  - 自动驾驶安全性测试：检测车辆与环境（如其他车辆、行人、障碍物）的碰撞。
  - 事件记录：为仿真分析提供碰撞数据。
  - ROS2集成：在机器人操作系统中处理碰撞数据。

#### 代码结构
- **构造函数**：
  - 初始化传感器，启用每帧Tick（`PrimaryActorTick.bCanEverTick = true`）。
- **主要方法**：
  - `GetSensorDefinition()`：定义传感器蓝图，类型为`other.collision`。
  - `SetOwner(AActor *NewOwner)`：绑定传感器到所有者（如车辆或行人），动态注册碰撞事件回调：
    - 对于`ACarlaWheeledVehicle`，绑定到车辆网格的`OnComponentHit`事件。
    - 对于`AWalkerBase`，绑定到行人网格的`OnComponentHit`事件。
    - 其他情况，绑定到Actor的`OnActorHit`事件。
  - `PrePhysTick(float DeltaSeconds)`：清理上一帧的碰撞记录，避免重复处理。
  - `OnCollisionEvent(AActor *Actor, AActor *OtherActor, FVector NormalImpulse, const FHitResult &Hit)`：
    - 验证碰撞双方有效性。
    - 检查是否为新碰撞（避免同一帧重复处理）。
    - 将冲量转换为米单位（`NormalImpulse *= 1e-2`）。
    - 序列化碰撞数据（包括双方Actor信息和冲量），通过数据流发送。
    - 记录到Carla的事件记录器（`GetRecorder()->AddCollision`）。
    - 支持ROS2数据传输，包含传感器变换和冲量信息。
  - `OnActorCollisionEvent` 和 `OnComponentCollisionEvent`：分别处理Actor级和组件级碰撞，调用`OnCollisionEvent`。
- **数据结构**：
  - `CollisionRegistry`：存储碰撞记录（帧号、碰撞双方Actor），用于去重。

#### 依赖关系
- **Carla模块**：`CarlaEpisode`（获取仿真帧和Actor信息）、`ActorRegistry`（序列化Actor）、`CarlaEngine`（获取帧计数器）。
- **Unreal Engine**：`AActor`、`UPrimitiveComponent`（碰撞检测）、`FHitResult`（碰撞细节）。
- **外部接口**：`FCarlaServer`（数据流）、ROS2（可选）。

#### 使用方法
1. **蓝图配置**：
   - 在Unreal Editor中，将`CollisionSensor`附加到车辆或行人Actor。
   - 设置传感器ID和位置（通常与所有者Actor对齐）。
2. **Python API示例**：
   ```python
   import carla
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   blueprint_library = world.get_blueprint_library()
   collision_bp = blueprint_library.find('sensor.other.collision')
   collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
   collision_sensor.listen(lambda event: print(f"Collision with {event.other_actor.type_id}, impulse: {event.normal_impulse}"))
   ```
3. **ROS2集成**：
   - 启用ROS2支持后，传感器数据以特定格式（包含变换和冲量）发送到ROS2话题。
   - 示例话题：`/carla/<vehicle_id>/collision`。

#### 数据输出格式
- **序列化数据**：
  ```json
  {
    "actor": {"id": <actor_id>, "type": <actor_type>},
    "other_actor": {"id": <other_actor_id>, "type": <other_actor_type>},
    "normal_impulse": {"x": <float>, "y": <float>, "z": <float>}
  }
  ```
- **ROS2数据**：包含传感器变换（相对于父Actor）和冲量向量。

#### 性能和优化
- **性能瓶颈**：频繁的碰撞检测和数据序列化可能影响性能，尤其在密集场景中。
- **优化建议**：
  - 限制碰撞检测的频率（通过调整Tick间隔）。
  - 使用更高效的数据结构（如`unordered_set`）替代`CollisionRegistry`的线性搜索。
  - 减少ROS2数据传输的频率，仅在关键碰撞时发送。

### 2. CustomV2XSensor.cpp

#### 文件路径
`~/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/CustomV2XSensor.cpp`

#### 功能
- **类**：`ACustomV2XSensor`（继承自传感器基类，推测为`ASensor`）。
- **作用**：模拟V2X（Vehicle-to-Everything）通信，允许车辆间发送和接收自定义消息，基于路径损耗模型（`PathLossModel`）模拟信号传播。
- **应用场景**：
  - 车联网仿真：测试V2X协议（如CAM消息）在不同场景下的性能。
  - 协同驾驶：模拟车辆间的信息共享（如位置、速度）。
  - 通信研究：分析信号衰减、干扰和接收功率。

#### 代码结构
- **构造函数**：
  - 初始化传感器，启用每帧Tick。
  - 创建`URandomEngine`（用于噪声模拟）和`PathLossModel`对象。
- **主要方法**：
  - `SetOwner(AActor *Owner)`：
    - 管理所有者绑定，维护静态`mV2XActorContainer`列表。
    - 设置站ID（基于Carla Actor ID）。
    - 将所有者传递给`PathLossModel`。
  - `GetSensorDefinition()`：返回V2X传感器蓝图定义（`MakeCustomV2XDefinition`）。
  - **参数配置**：
    - `Set(const FActorDescription &ActorDescription)`：从蓝图设置传感器属性。
    - `SetPropagationParams(...)`：配置传播参数（发射功率、接收灵敏度、频率、天线增益等）。
    - `SetPathLossModel(EPathLossModel)`：选择路径损耗模型（如自由空间、城市模型）。
    - `SetScenario(EScenario)`：设置场景（如城市、公路）。
  - **消息处理**：
    - `PrePhysTick(float DeltaSeconds)`：
      - 清除上一周期的消息。
      - 如果消息数据更新（`mMessageDataChanged`），生成新消息并存储到`mActorV2XDataMap`。
    - `CreateCustomV2XMessage()`：生成V2X消息，包含ITS PDU头部（协议版本、消息ID、站ID）和用户定义内容。
    - `PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)`：
      - 构建其他Actor的发送列表（`ActorPowerList`）。
      - 调用`PathLossModel->Simulate`模拟通信，获取接收功率（`ActorPowerMap`）。
      - 从`mActorV2XDataMap`提取接收到的消息，存储到`V2XDataList`。
      - 通过`WriteMessageToV2XData`序列化消息并发送。
  - `Send(const FString message)`：设置用户定义的消息内容，标记为待发送。
- **数据结构**：
  - `mV2XActorContainer`：静态列表，存储所有V2X传感器绑定的Actor。
  - `mActorV2XDataMap`：静态映射，存储Actor与消息/功率对的关联。
  - `mV2XData`：存储待发送的消息集合。

#### 依赖关系
- **Carla模块**：`CarlaEpisode`（获取Actor信息）、`UCarlaStatics`（访问仿真状态）。
- **Unreal Engine**：`AActor`、`URandomEngine`（随机数生成）。
- **自定义模块**：`PathLossModel`（信号传播模拟）。

#### 使用方法
1. **蓝图配置**：
   - 创建V2X传感器，设置传播参数（如`TransmitPower=23.0`、`ReceiverSensitivity=-90.0`）。
   - 附加到车辆Actor，指定场景（如`EScenario::Urban`）。
2. **Python API示例**：
   ```python
   import carla
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   blueprint_library = world.get_blueprint_library()
   v2x_bp = blueprint_library.find('sensor.other.custom_v2x')
   v2x_bp.set_attribute('transmit_power', '23.0')
   v2x_bp.set_attribute('receiver_sensitivity', '-90.0')
   v2x_sensor = world.spawn_actor(v2x_bp, carla.Transform(), attach_to=vehicle)
   v2x_sensor.listen(lambda data: print(f"Received V2X message: {data.message}"))
   ```
3. **发送消息**：
   - 通过蓝图或代码调用`Send(FString)`发送自定义消息。
   - 示例：`v2x_sensor->Send("Hello, V2X!")`。

#### 数据输出格式
- **序列化数据**：
  ```json
  [
    {
      "message": "<user_defined_string>",
      "power": <float> // 接收功率（dBm）
    },
    ...
  ]
  ```
- **消息结构**（`CustomV2XM_t`）：
  ```cpp
  struct CustomV2XM_t {
    ITSContainer::ItsPduHeader_t header; // 协议版本、消息ID、站ID
    char message[/* max_length */];      // 用户定义内容
  };
  ```

#### 性能和优化
- **性能瓶颈**：
  - `PathLossModel->Simulate`涉及复杂计算，可能在大量车辆场景中成为瓶颈。
  - 静态`mActorV2XDataMap`的频繁访问可能导致内存开销。
- **优化建议**：
  - 限制通信范围（通过`filter_distance`参数）以减少模拟的Actor数量。
  - 使用异步任务处理信号传播计算。
  - 定期清理`mV2XActorContainer`以释放无效Actor。

### 3. DepthCamera.cpp

#### 文件路径
`~/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/DepthCamera.cpp`

#### 功能
- **类**：`ADepthCamera`（继承自`ASceneCaptureSensor`，推测为场景捕获基类）。
- **作用**：捕获场景的深度信息，生成深度图像，应用后处理效果（如镜头畸变和深度渲染）。
- **应用场景**：
  - 环境感知：为自动驾驶提供3D场景深度信息。
  - 地图重建：生成点云或深度图。
  - 视觉算法测试：验证深度相关的感知算法。

#### 代码结构
- **构造函数**：
  - 添加后处理材质：
    - `PhysicLensDistortion`：模拟镜头畸变。
    - `DepthEffectMaterial`：渲染深度效果（Linux使用GLSL版本，其他平台使用默认版本）。
- **主要方法**：
  - `GetSensorDefinition()`：返回深度相机蓝图定义（类型为`camera.depth`）。
  - `PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)`：
    - 使用`FPixelReader::SendPixelsInRenderThread`在渲染线程中捕获深度图像。
    - 将图像数据（`FColor`格式）发送到客户端。
- **数据结构**：依赖Unreal Engine的渲染目标（`UTextureRenderTarget2D`）存储深度图像。

#### 依赖关系
- **Carla模块**：`FPixelReader`（像素读取和发送）、`UActorBlueprintFunctionLibrary`（蓝图定义）。
- **Unreal Engine**：`ASceneCaptureSensor`（场景捕获基类）、`UTextureRenderTarget2D`（渲染目标）。
- **后处理材质**：`PhysicLensDistortion`、`DepthEffectMaterial`（及其GLSL变体）。

#### 使用方法
1. **蓝图配置**：
   - 创建深度相机，设置分辨率（如`image_size_x=1280`、`image_size_y=720`）和视场角（`fov=90`）。
   - 附加到车辆或固定位置。
2. **Python API示例**：
   ```python
   import carla
   import numpy as np
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   blueprint_library = world.get_blueprint_library()
   depth_bp = blueprint_library.find('sensor.camera.depth')
   depth_bp.set_attribute('image_size_x', '1280')
   depth_bp.set_attribute('image_size_y', '720')
   depth_camera = world.spawn_actor(depth_bp, carla.Transform(carla.Location(z=2.5)), attach_to=vehicle)
   depth_camera.listen(lambda image: process_depth_image(image))
   def process_depth_image(image):
       array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
       print(f"Depth image shape: {array.shape}")
   ```
3. **数据处理**：
   - 深度图像以`FColor`格式传输，通常需要转换为浮点深度值（参考Carla文档）。

#### 数据输出格式
- **序列化数据**：RGBA图像（`FColor`格式），其中颜色编码深度值。
- **示例**：
  ```python
  # 深度值解码（参考Carla文档）
  def depth_to_array(image):
      array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
      depth = (array[:, :, 0] + array[:, :, 1] * 256 + array[:, :, 2] * 256 * 256) / (256 * 256 * 256 - 1)
      return depth * 1000  # 转换为米
  ```

#### 性能和优化
- **性能瓶颈**：高分辨率深度图像的渲染和传输可能导致延迟。
- **优化建议**：
  - 降低分辨率或视场角以减少渲染开销。
  - 使用异步渲染队列处理深度图像。
  - 仅在必要时启用后处理效果。

### 4. DVSCamera.cpp

#### 文件路径
`~/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/DVSCamera.cpp`

#### 功能
- **类**：`ADVSCamera`（继承自`ASceneCaptureSensor`，推测为场景捕获基类）。
- **作用**：模拟动态视觉传感器（DVS），通过检测像素亮度变化生成事件流，适用于高速运动或低光照场景。
- **应用场景**：
  - 动态场景感知：跟踪快速移动的物体。
  - 低功耗视觉：模拟生物启发的视觉系统。
  - 神经形态计算：为Spiking Neural Networks提供输入数据。

#### 代码结构
- **构造函数**：
  - 启用后处理效果，添加`PhysicLensDistortion`材质。
  - 创建`URandomEngine`用于噪声模拟。
- **主要方法**：
  - `GetSensorDefinition()`：
    - 返回DVS相机蓝图定义，包含可配置参数：
      - `positive_threshold`（正阈值，默认0.3）。
      - `negative_threshold`（负阈值，默认0.3）。
      - `sigma_positive_threshold`（正噪声标准差，默认0.0）。
      - `sigma_negative_threshold`（负噪声标准差，默认0.0）。
      - `refractory_period_ns`（不应期，纳秒，默认0）。
      - `use_log`（是否使用对数变换，默认true）。
      - `log_eps`（对数变换epsilon，默认0.001）。
  - `Set(const FActorDescription &Description)`：从蓝图提取参数，存储到`config`结构。
  - `PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)`：
    - 捕获RGB图像（`ReadPixels`）。
    - 转换为灰度（`ImageToGray`）或对数灰度（`ImageToLogGray`）。
    - 运行DVS仿真（`Simulation`），生成事件流。
    - 序列化事件并通过数据流或ROS2发送。
  - `ImageToGray(const TArray<FColor> &image)`：
    - 将RGB图像转换为灰度（`0.2989*R + 0.5870*G + 0.1140*B`）。
    - 存储到`last_image`。
  - `ImageToLogGray(const TArray<FColor> &image)`：
    - 将RGB图像转换为对数灰度（`log(eps + I/255.0)`）。
    - 存储到`last_image`。
  - `Simulation(float DeltaTime)`：
    - 初始化：设置参考图像（`ref_values`）、前一图像（`prev_image`）和时间戳（`last_event_timestamp`）。
    - 遍历像素，检测亮度变化（阈值`1e-6`）。
    - 根据变化方向（正/负）选择阈值（`Cp`/`Cm`），添加噪声（`sigma_Cp`/`sigma_Cm`）。
    - 生成事件（`DVSEvent`：x、y、时间戳、极性），考虑不应期（`refractory_period_ns`）。
    - 按时间戳排序事件。
- **数据结构**：
  - `last_image`：当前灰度图像。
  - `prev_image`：前一帧灰度图像。
  - `ref_values`：参考亮度值。
  - `last_event_timestamp`：每个像素的最近事件时间戳。
  - `DVSEventArray`：事件列表（x、y、t、polarity）。

#### 依赖关系
- **Carla模块**：`CarlaEpisode`（获取时间）、`UActorBlueprintFunctionLibrary`（蓝图参数）、`ROS2`（可选）。
- **Unreal Engine**：`ASceneCaptureSensor`、`URandomEngine`、`UTextureRenderTarget2D`。
- **外部库**：`carla::Buffer`、`carla::BufferView`（数据序列化）。

#### 使用方法
1. **蓝图配置**：
   - 创建DVS相机，设置分辨率和参数（如`positive_threshold=0.3`、`refractory_period_ns=1000`）。
   - 附加到车辆或固定位置。
2. **Python API示例**：
   ```python
   import carla
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   blueprint_library = world.get_blueprint_library()
   dvs_bp = blueprint_library.find('sensor.camera.dvs')
   dvs_bp.set_attribute('positive_threshold', '0.3')
   dvs_bp.set_attribute('use_log', 'True')
   dvs_camera = world.spawn_actor(dvs_bp, carla.Transform(carla.Location(z=2.5)), attach_to=vehicle)
   dvs_camera.listen(lambda data: process_dvs_events(data))
   def process_dvs_events(data):
       for event in data:
           print(f"DVS event: x={event.x}, y={event.y}, t={event.t}, polarity={event.polarity}")
   ```
3. **ROS2集成**：
   - 事件流以`DVSEvent`格式发送到ROS2话题（如`/carla/<vehicle_id>/dvs`）。
   - 包含相机分辨率、视场角和变换信息。

#### 数据输出格式
- **序列化数据**：
  ```json
  [
    {"x": <int>, "y": <int>, "t": <uint64>, "polarity": <bool>},
    ...
  ]
  ```
- **ROS2数据**：包含事件流、相机参数（分辨率、视场角）和变换。

#### 性能和优化
- **性能瓶颈**：
  - 像素级亮度变化检测和事件生成对高分辨率图像开销大。
  - 事件排序（`std::sort`）在高事件率时可能影响性能。
- **优化建议**：
  - 降低分辨率或限制视场角。
  - 使用并行处理（如多线程）加速像素遍历。
  - 调整`refractory_period_ns`以减少事件生成率。

## 集成和扩展

### 依赖关系总结
- **共同依赖**：
  - Unreal Engine：`AActor`、`UPrimitiveComponent`、`UTextureRenderTarget2D`。
  - Carla框架：`CarlaEpisode`、`ActorRegistry`、`UCarlaStatics`、`FPixelReader`。
- **特定依赖**：
  - `CollisionSensor`：`FCarlaServer`、`ROS2`。
  - `CustomV2XSensor`：`PathLossModel`。
  - `DepthCamera`：后处理材质（`PhysicLensDistortion`、`DepthEffectMaterial`）。
  - `DVSCamera`：`URandomEngine`、`carla::Buffer`。

### 扩展建议
- **自定义传感器**：
  - 继承`ASensor`或`ASceneCaptureSensor`，实现`GetSensorDefinition`和数据处理逻辑。
  - 参考`CustomV2XSensor`的路径损耗模型集成方式。
- **新功能**：
  - 为`CollisionSensor`添加碰撞强度过滤。
  - 为`CustomV2XSensor`支持更多消息类型（如DENM）。
  - 为`DepthCamera`添加自定义后处理效果。
  - 为`DVSCamera`支持彩色事件生成。

