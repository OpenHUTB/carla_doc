# CarlaUnreal 灯光模块说明文档

## 概述

CarlaUnreal 插件的灯光（Lights）模块是基于虚幻引擎（Unreal Engine）开发的，用于在 CARLA 模拟器中管理和控制灯光相关功能。该模块通过两个核心类 `UCarlaLight` 和 `UCarlaLightSubsystem` 实现，分别负责单个灯光组件的管理和全局灯光系统的协调。灯光模块支持动态设置灯光的强度、颜色、开关状态和类型，并与天气系统、日夜循环等功能集成，提供逼真的光照效果。

本模块遵循 MIT 许可证，版权归巴塞罗那自治大学（UAB）计算机视觉中心（CVC）所有。

---

## 模块结构

灯光模块主要包含以下两个核心类：

1. **`UCarlaLight`**：表示单个灯光组件，负责管理灯光的属性（如强度、颜色、开关状态等）以及与灯光子系统的交互。
2. **`UCarlaLightSubsystem`**：全局灯光子系统，负责管理所有灯光组件的注册、注销、状态更新以及与客户端的通信。

以下是对这两个类的详细说明。

---

## 1. UCarlaLight 类

### 1.1 类概述

`UCarlaLight` 是一个继承自虚幻引擎组件的类（推测继承自 `UActorComponent` 或类似组件），用于表示游戏世界中的单个灯光。它提供了灯光属性的设置与获取方法，并支持与灯光子系统、天气系统以及事件记录器的交互。

### 1.2 主要功能

- **灯光属性管理**：支持设置和获取灯光的强度、颜色、开关状态和类型。
- **生命周期管理**：处理灯光组件的初始化、销毁和游戏结束逻辑。
- **子系统交互**：将灯光注册到 `UCarlaLightSubsystem` 中，以便进行全局管理。
- **状态记录**：记录灯光状态的变化，用于日志或回放功能。
- **位置获取**：支持将灯光的本地坐标转换为全局坐标（针对大型地图场景）。

### 1.3 关键成员函数

以下是 `UCarlaLight` 类的主要函数及其功能：

#### 构造函数
```cpp
UCarlaLight::UCarlaLight()
```
- 初始化灯光组件，禁用每帧 Tick（`bCanEverTick = false`）以优化性能。

#### 生命周期函数
- **BeginPlay**
  ```cpp
  void BeginPlay()
  ```
  - 在游戏开始时调用，注册灯光到子系统并与天气系统关联。
- **EndPlay**
  ```cpp
  void EndPlay(const EEndPlayReason::Type EndPlayReason)
  ```
  - 在游戏结束时调用，从灯光子系统中注销灯光。
- **OnComponentDestroyed**
  ```cpp
  void OnComponentDestroyed(bool bDestroyingHierarchy)
  ```
  - 在组件销毁时调用，执行父类的销毁逻辑。

#### 灯光属性设置与获取
- **SetLightIntensity / GetLightIntensity**
  ```cpp
  void SetLightIntensity(float Intensity)
  float GetLightIntensity() const
  ```
  - 设置和获取灯光强度，调用 `UpdateLights` 更新灯光效果。
- **SetLightColor / GetLightColor**
  ```cpp
  void SetLightColor(FLinearColor Color)
  FLinearColor GetLightColor() const
  ```
  - 设置和获取灯光颜色，调用 `UpdateLights` 和 `RecordLightChange` 更新效果并记录变化。
- **SetLightOn / GetLightOn**
  ```cpp
  void SetLightOn(bool bOn)
  bool GetLightOn() const
  ```
  - 设置和获取灯光开关状态，调用 `UpdateLights` 和 `RecordLightChange`。
- **SetLightType / GetLightType**
  ```cpp
  void SetLightType(ELightType Type)
  ELightType GetLightType() const
  ```
  - 设置和获取灯光类型（枚举类型 `ELightType`）。

#### 灯光状态管理
- **GetLightState**
  ```cpp
  carla::rpc::LightState GetLightState()
  ```
  - 返回灯光的当前状态（包括位置、强度、类型、颜色、开关状态等），用于 RPC 通信。
- **SetLightState**
  ```cpp
  void SetLightState(carla::rpc::LightState LightState)
  ```
  - 根据传入的 `carla::rpc::LightState` 更新灯光的所有属性，并调用 `UpdateLights` 和 `RecordLightChange`。

#### 注册与注销
- **RegisterLight**
  ```cpp
  void RegisterLight()
  ```
  - 将灯光注册到 `UCarlaLightSubsystem` 并与天气系统关联，避免重复注册。
- **RegisterLightWithWeather**
  ```cpp
  void RegisterLightWithWeather()
  ```
  - 将灯光与天气系统关联（具体实现未在代码中提供，推测与天气动态调整灯光相关）。

#### 其他功能
- **GetLocation**
  ```cpp
  FVector GetLocation() const
  ```
  - 获取灯光的世界位置，支持大型地图的本地到全局坐标转换。
- **GetId / SetId**
  ```cpp
  int GetId() const
  void SetId(int InId)
  ```
  - 获取和设置灯光的唯一标识符。
- **RecordLightChange**
  ```cpp
  void RecordLightChange() const
  ```
  - 记录灯光状态变化到事件记录器，用于日志或回放。

### 1.4 数据成员（推测）

基于函数实现，`UCarlaLight` 可能包含以下成员变量：
- `bool bRegistered`：标记灯光是否已注册。
- `float LightIntensity`：灯光强度。
- `FLinearColor LightColor`：灯光颜色。
- `bool bLightOn`：灯光开关状态。
- `ELightType LightType`：灯光类型（枚举）。
- `int Id`：灯光唯一标识符。

---

## 2. UCarlaLightSubsystem 类

### 2.1 类概述

`UCarlaLightSubsystem` 是虚幻引擎的子系统类（继承自 `USubsystem`），负责管理所有 `UCarlaLight` 实例。它提供灯光的注册、注销、状态查询与更新功能，并支持与客户端的通信以及日夜循环的控制。

### 2.2 主要功能

- **灯光管理**：维护一个灯光集合，支持注册和注销灯光。
- **状态同步**：跟踪客户端的状态变化，确保灯光状态在多客户端环境中同步。
- **批量操作**：支持批量获取和设置灯光状态。
- **日夜循环**：与天气系统集成，控制日夜循环的开关。
- **客户端管理**：支持多客户端场景，标记“脏”状态以触发更新。

### 2.3 关键成员函数

#### 初始化与反初始化
- **Initialize**
  ```cpp
  void Initialize(FSubsystemCollectionBase& Collection)
  ```
  - 子系统初始化，当前为空，注释提到计划订阅地图变化。
- **Deinitialize**
  ```cpp
  void Deinitialize()
  ```
  - 子系统反初始化，当前为空。

#### 灯光管理
- **RegisterLight**
  ```cpp
  void RegisterLight(UCarlaLight* CarlaLight)
  ```
  - 注册一个灯光到子系统，检查 ID 冲突并标记客户端状态为“脏”。
- **UnregisterLight**
  ```cpp
  void UnregisterLight(UCarlaLight* CarlaLight)
  ```
  - 从子系统中注销一个灯光，标记客户端状态为“脏”。
- **GetLight**
  ```cpp
  UCarlaLight* GetLight(int Id)
  ```
  - 根据 ID 获取对应的灯光组件，若不存在返回 `nullptr`。

#### 状态管理
- **IsUpdatePending**
  ```cpp
  bool IsUpdatePending() const
  ```
  - 检查是否有客户端状态待更新（即是否有“脏”状态）。
- **GetLights**
  ```cpp
  std::vector<carla::rpc::LightState> GetLights(FString Client)
  ```
  - 获取所有灯光的状态并返回，清除指定客户端的“脏”状态。
- **SetLights**
  ```cpp
  void SetLights(FString Client, std::vector<carla::rpc::LightState> LightsToSet, bool DiscardClient)
  ```
  - 批量设置灯光状态，更新指定客户端状态并可选丢弃客户端。

#### 日夜循环
- **SetDayNightCycle**
  ```cpp
  void SetDayNightCycle(const bool active)
  ```
  - 设置日夜循环的开关状态，通过天气控制器（`AWeather`）实现。

#### 客户端状态管理
- **SetClientStatesdirty**
  ```cpp
  void SetClientStatesdirty(FString ClientThatUpdate)
  ```
  - 标记除指定客户端外的所有客户端状态为“脏”，触发更新。

### 2.4 数据成员（推测）

基于函数实现，`UCarlaLightSubsystem` 可能包含以下成员变量：
- `TMap<int, UCarlaLight*> Lights`：存储灯光 ID 和对应灯光组件的映射。
- `TMap<FString, bool> ClientStates`：存储客户端标识及其“脏”状态的映射。

---

## 使用场景

### 1. 创建和配置灯光
1. 在虚幻引擎中创建一个 Actor，添加 `UCarlaLight` 组件。
2. 通过蓝图或代码设置灯光的属性（强度、颜色、类型等）。
3. 灯光会自动在 `BeginPlay` 时注册到 `UCarlaLightSubsystem`。

### 2. 动态调整灯光
- 使用 `SetLightIntensity`、`SetLightColor` 等函数动态调整灯光效果。
- 调用 `SetLightState` 批量设置灯光状态（常用于 RPC 通信）。

### 3. 管理日夜循环
- 通过 `UCarlaLightSubsystem::SetDayNightCycle` 启用或禁用日夜循环，与天气系统联动。

### 4. 客户端同步
- 在多客户端场景中，使用 `GetLights` 和 `SetLights` 同步灯光状态。
- 通过 `IsUpdatePending` 检查是否有状态更新需求。

---

## 依赖关系

- **虚幻引擎核心模块**：依赖 `UWorld`、`UActorComponent`、`FLinearColor`、`FVector` 等。
- **Carla 模块**：
  - `CarlaStatics`：提供获取游戏模式、剧情实例等静态工具。
  - `AWeather`：天气系统，用于日夜循环控制。
  - `carla::rpc`：提供 RPC 通信相关的数据结构（如 `LightState`）。
- **Kismet**：使用 `GameplayStatics` 获取天气控制器。

---

## 注意事项

1. **ID 冲突**：注册灯光时需确保 ID 唯一，否则会触发警告日志。
2. **性能优化**：`UCarlaLight` 默认禁用 Tick，适合静态或低频更新的灯光场景。
3. **大型地图支持**：位置获取支持本地到全局坐标转换，需确保 `ALargeMapManager` 正确配置。
4. **客户端状态**：多客户端场景下，需妥善管理 `ClientStates` 以避免状态同步问题。
5. **天气系统集成**：日夜循环依赖 `AWeather` 类的实现，确保天气控制器存在。

---

## 扩展与定制

- **添加新灯光类型**：扩展 `ELightType` 枚举并实现对应的渲染逻辑。
- **增强天气交互**：在 `RegisterLightWithWeather` 中添加天气动态调整灯光的逻辑。
- **优化性能**：根据场景需求启用部分灯光的 Tick 或实现批量更新机制。
- **事件记录**：扩展 `RecordLightChange` 以支持更复杂的事件日志格式。

---
