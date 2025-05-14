# CarlaUnreal 插件 Actor 模块说明文档

# 目录
- [概述](#概述)
- [核心类与功能](#核心类与功能)
  - [1. FActorData](#1-factordata)
    - [主要功能](#主要功能)
    - [关键方法](#关键方法)
    - [派生类](#派生类)
  - [2. UActorDispatcher](#2-uactordispatcher)
    - [主要功能](#主要功能-1)
    - [关键方法](#关键方法-1)
  - [3. FActorRegistry](#3-factorregistry)
    - [主要功能](#主要功能-2)
    - [关键方法](#关键方法-2)
- [代码结构与实现细节](#代码结构与实现细节)
  - [数据记录与恢复](#数据记录与恢复)
  - [Actor 生成与管理](#actor-生成与管理)
  - [ROS2 集成](#ros2-集成)
  - [错误处理](#错误处理)
- [使用场景](#使用场景)
  - [仿真场景初始化](#仿真场景初始化)
  - [状态保存与恢复](#状态保存与恢复)
  - [动态管理](#动态管理)
  - [传感器与数据流](#传感器与数据流)
  - [交通模拟](#交通模拟)
- [注意事项](#注意事项)
  - [物理模拟](#物理模拟)
  - [ID 管理](#id-管理)
  - [ROS2 集成](#ros2-集成-1)
  - [边界框与标签](#边界框与标签)
  - [未实现功能](#未实现功能)

## 概述

CarlaUnreal 插件中的 Actor 模块是 CARLA 模拟器中用于管理和操作参与者（Actor）的重要组件。参与者包括车辆、行人、交通信号灯、交通标志和传感器等，用于模拟真实的交通场景和环境交互。该模块提供了 Actor 的生成、注册、数据记录与恢复、状态管理（如休眠与唤醒）以及销毁等功能，广泛应用于 CARLA 的仿真逻辑中。

本模块基于 Unreal Engine 的 Actor 系统，结合 CARLA 的自定义逻辑，实现了对不同类型参与者的统一管理和高效操作。以下文档详细介绍了模块中的核心类、功能以及代码实现的关键点。

---

## 核心类与功能

### 1. FActorData

`FActorData` 是 Actor 模块的基础数据类，用于记录 **记录**和**恢复**通用 Actor 的状态数据（如位置、旋转、速度等）。它是其他特定类型数据类的基类（如车辆、行人等）。

#### 主要功能
- **记录数据**：记录 Actor 的位置、旋转、缩放、速度、角速度和物理模拟状态。
- **恢复数据**：根据记录的数据恢复 Actor 的状态。
- **重生 Actor**：根据存储的变换信息重新生成 Actor。
- **获取局部变换**：计算相对于当前地图原点的变换。

#### 关键方法
- `AActor* RespawnActor(UCarlaEpisode* CarlaEpisode, const FActorInfo& Info)`  
  重生一个 Actor，使用存储的变换信息并向上偏移 15 个单位以避免碰撞。
- `void RecordActorData(FCarlaActor* CarlaActor, UCarlaEpisode* CarlaEpisode)`  
  记录 Actor 的位置、旋转、缩放、速度、角速度和物理状态。
- `void RestoreActorData(FCarlaActor* CarlaActor, UCarlaEpisode* CarlaEpisode)`  
  恢复 Actor 的变换、速度、角速度和物理状态，针对车辆和行人进行特殊处理。
- `FTransform GetLocalTransform(UCarlaEpisode* CarlaEpisode) const`  
  计算相对于当前地图原点的局部变换。

#### 派生类
- **FVehicleData**：扩展了 `FActorData`，用于记录和恢复车辆特定的数据，如物理控制、灯光状态和速度限制。
- **FWalkerData**：扩展了 `FActorData`，用于记录和恢复行人特定的控制数据和存活状态。
- **FTrafficSignData**：扩展了 `FActorData`，用于记录和恢复交通标志的模型和标识信息。
- **FTrafficLightData**：扩展了 `FActorData`，用于记录和恢复交通信号灯的状态和控制器信息。
- **FActorSensorData**：扩展了 `FActorData`，用于记录和恢复传感器的数据流。

---

### 2. UActorDispatcher

`UActorDispatcher` 是 Actor 模块的核心调度类，负责 **生成**、**注册**、**销毁**和**状态管理**（休眠/唤醒）Actor。它通过绑定 Actor 定义和生成函数，管理 Actor 的生命周期。

#### 主要功能
- **绑定 Actor 定义**：将 Actor 定义与生成函数关联，分配唯一 ID。
- **生成 Actor**：根据描述和变换生成 Actor，并注册到系统中。
- **销毁 Actor**：移除 Actor 及其控制器，清理注册信息。
- **状态管理**：将 Actor 设置为休眠或唤醒状态。
- **ROS2 集成**：支持将 Actor 注册到 ROS2 系统中，处理名称映射和回调。

#### 关键方法
- `void Bind(FActorDefinition Definition, SpawnFunctionType Functor)`  
  绑定 Actor 定义和生成函数，分配唯一 ID。
- `void Bind(ACarlaActorFactory &ActorFactory)`  
  绑定 Actor 工厂中的所有定义。
- `TPair<EActorSpawnResultStatus, FCarlaActor*> SpawnActor(const FTransform &Transform, FActorDescription Description, FCarlaActor::IdType DesiredId)`  
  生成 Actor 并注册，返回生成状态和 Actor 指针。
- `AActor* ReSpawnActor(const FTransform &Transform, FActorDescription Description)`  
  重新生成 Actor，不进行注册。
- `bool DestroyActor(FCarlaActor::IdType ActorId)`  
  销毁指定 ID 的 Actor 及其控制器。
- `FCarlaActor* RegisterActor(AActor &Actor, FActorDescription Description, FActorRegistry::IdType DesiredId)`  
  注册 Actor，绑定到指定 ID，并支持 ROS2 集成。
- `void PutActorToSleep(FCarlaActor::IdType Id, UCarlaEpisode* CarlaEpisode)`  
  将 Actor 及其子 Actor 设置为休眠状态。
- `void WakeActorUp(FCarlaActor::IdType Id, UCarlaEpisode* CarlaEpisode)`  
  唤醒指定 ID 的 Actor。
- `void OnActorDestroyed(AActor *Actor)`  
  处理 Actor 销毁事件，清理注册和 ROS2 映射。

---

### 3. FActorRegistry

`FActorRegistry` 是 Actor 模块的注册管理类，负责 **注册**、**注销**和**查找** Actor。它维护 Actor 的唯一 ID 和状态信息，确保数据一致性。

#### 主要功能
- **注册 Actor**：为 Actor 分配唯一 ID，存储其信息并关联到系统中。
- **注销 Actor**：移除 Actor 的注册信息，清理映射。
- **查找 Actor**：根据 ID 或 Actor 指针查找注册的 Actor。
- **状态管理**：支持 Actor 的休眠和唤醒操作。
- **类型判断**：根据 Actor 类型返回对应的枚举值（车辆、行人、传感器等）。

#### 关键方法
- `FCarlaActor* Register(AActor &Actor, FActorDescription Description, IdType DesiredId)`  
  注册 Actor，分配或复用 ID，存储描述和状态信息。
- `void Deregister(IdType Id)`  
  注销指定 ID 的 Actor，清理映射。
- `void Deregister(AActor *Actor)`  
  根据 Actor 指针注销 Actor。
- `TSharedPtr<FCarlaActor> MakeCarlaActor(IdType Id, AActor &Actor, FActorDescription Description, crp::ActorState InState)`  
  创建 `FCarlaActor` 对象，初始化描述、标签和边界框。
- `void PutActorToSleep(FCarlaActor::IdType Id, UCarlaEpisode* CarlaEpisode)`  
  将 Actor 及其子 Actor 设置为休眠状态，移除活跃映射。
- `FCarlaActor* FindCarlaActor(IdType Id)`  
  根据 ID 查找注册的 Actor。
- `static FCarlaActor::ActorType GetActorType(const AActor *Actor)`  
  判断 Actor 类型（车辆、行人、交通信号灯等）。

---

## 代码结构与实现细节

### 数据记录与恢复
- **通用数据**：`FActorData` 及其派生类记录 Actor 的位置、旋转、速度等基本信息，确保状态一致性。
- **特定数据**：针对车辆（`FVehicleData`）、行人（`FWalkerData`）等类型，记录特定控制数据（如车辆灯光、行人控制）。
- **恢复逻辑**：恢复时根据 Actor 类型（如车辆、传感器）应用不同逻辑，确保物理状态和控制参数正确。

### Actor 生成与管理
- **生成流程**：`UActorDispatcher` 通过绑定的生成函数生成 Actor，并通过 `FActorRegistry` 注册到系统中。
- **ID 分配**：`FActorRegistry` 使用全局计数器 `ID_COUNTER` 分配唯一 ID，支持用户指定 ID。
- **状态管理**：休眠状态通过移除活跃映射实现，唤醒时重新注册 Actor。

### ROS2 集成
- **名称映射**：Actor 注册时支持 ROS2 名称映射，处理车辆和非车辆的命名规则。
- **回调支持**：为特定角色（如 `hero` 或 `ego`）添加 ROS2 回调，处理实时数据交互。

### 错误处理
- **日志记录**：使用 Unreal Engine 的日志系统（`UE_LOG`）记录生成、销毁和注册中的错误。
- **状态检查**：生成和注册过程中对 Actor 描述和 ID 进行验证，确保数据有效性。

---

## 使用场景
1. **仿真场景初始化**：
   - 使用 `UActorDispatcher::SpawnActor` 生成车辆、行人等 Actor，设置初始位置和状态，进行场景初始化。
   - 通过 `FActorRegistry::Register` 注册 Actor，分配唯一 ID。
2. **状态保存与恢复**：
   - 使用 `FActorData::RecordActorData` 保存 Actor 状态。
   - 使用 `FActorData::RestoreActorData` 恢复 Actor 状态，用于场景重置或回放。
3. **动态管理**：
   - 使用 `UActorDispatcher::PutActorToSleep` 和 `WakeActorUp` 动态管理 Actor 的活跃状态，优化性能。
   - 使用 `UActorDispatcher::DestroyActor` 清理不再需要的 Actor。
4. **传感器与数据流**：
   - 使用 `FActorSensorData` 管理传感器数据流，支持实时数据传输和处理。
5. **交通模拟**：
   - 使用 `FTrafficLightData` 和 `FTrafficSignData` 管理交通信号灯和标志的状态，模拟真实交通规则。

---

## 注意事项
1. **物理模拟**：
   - 车辆和行人的物理模拟状态通过 `bSimulatePhysics` 控制，需确保正确设置以避免仿真异常。
   - 恢复物理状态时，需针对不同 Actor 类型（如车辆、行人）应用特定逻辑。
2. **ID 管理**：
   - 确保 Actor ID 的唯一性，避免重复注册或冲突。
   - 使用 `DesiredId` 时，需验证其未被占用。
3. **ROS2 集成**：
   - 启用 ROS2 时，需确保正确配置名称映射和回调，避免命名冲突或数据丢失。
4. **边界框与标签**：
   - 边界框通过 `UBoundingBoxCalculator` 计算，需确保 Actor 的几何信息正确。
   - 语义标签通过 `ATagger` 获取，需避免无效标签（如 `None` 或 `Other`）。
5. **未实现功能**：
   - 部分代码（如行人死亡计时器）标记为 `TODO`，需后续完善。
   - 子 Actor 的休眠逻辑可能需要进一步优化。
