# 灯光(UCarlaLight)模块说明文档

## 目录

### [1. 模块概述](#1-模块概述)
   - [1.1 UCarlaLight介绍](#11-ucarlalight介绍)
   - [1.2 组件核心功能](#12-组件核心功能)

### [2. 类成员函数说明](#2-核心类说明)
   - [2.1 生命周期函数](#21-生命周期函数)
   - [2.2 灯光注册与注销](#22-灯光注册与注销)
   - [2.3 属性控制函数](#23-属性控制函数)
   - [2.4 辅助功能](#24-辅助功能)

### [3. 关键成员变量](#3-关键成员变量)
### [4. 使用场景](#4-使用场景)
  - [4.1 动态环境光照](#41-动态环境光照)
  - [4.2 车辆信号灯同步](#42-车辆信号灯同步)
  - [4.3 仿真事件回放](#43-仿真事件回放)
### [5. 注意事项](#5-注意事项)
### [6. 扩展建议](#6-扩展建议)

---

## 1 模块概述 <a id="1-模块概述"></a>
### 1.1 UCarlaLight介绍 <a id="11-ucarlalight介绍"></a>
`UCarlaLight` 是 CARLA 仿真平台中用于管理动态灯光的 Unreal Engine 组件类。
### 1.2 组件核心功能 <a id="12-组件核心功能"></a>
组件的核心功能包括：灯光属性控制、子系统集成、状态同步、事件记录。

- **灯光属性控制**：动态调整灯光强度、颜色、开关状态及类型。

- **子系统集成**：与 `CarlaLightSubsystem` 交互实现全局灯光管理。
- **状态同步**：支持与 RPC 协议兼容的灯光状态序列化（`carla::rpc::LightState`）。
- **事件记录**：灯光状态变化时触发事件记录，支持仿真回放与调试。

## 2 类成员函数说明<a id="2-核心类说明"></a>

### 2.1 生命周期函数<a id="21-生命周期函数"></a>

#### BeginPlay()
- **作用**：组件初始化入口，触发灯光注册流程。
- **流程**：
  1. 调用父类 `Super::BeginPlay()`。
  2. 执行 `RegisterLight()` 注册到灯光子系统。

#### EndPlay()
- **作用**：组件销毁时清理资源。
- **流程**：
  1. 从 `CarlaLightSubsystem` 注销当前灯光。
  2. 调用父类 `Super::EndPlay()`。

### 2.2 灯光注册与注销<a id="22-灯光注册与注销"></a>

#### RegisterLight()
- **作用**：将灯光注册到子系统和天气系统。
- **关键逻辑**：
  - 通过 `GetWorld()->GetSubsystem<UCarlaLightSubsystem>()` 获取子系统实例。
  - 调用 `CarlaLightSubsystem->RegisterLight(this)` 完成注册。
  - 标记 `bRegistered = true` 防止重复注册。
- **注销逻辑**：在 `EndPlay()` 中调用 `CarlaLightSubsystem->UnregisterLight(this)`。

### 2.3 属性控制函数<a id="23-属性控制函数"></a>

| 函数名 | 参数 | 功能描述 |
| --- | --- | --- |
| SetLightIntensity() | `float Intensity` | 设置灯光强度并更新渲染效果。 |
| SetLightColor() | `FLinearColor Color` | 设置颜色并记录状态变化事件。 |
| SetLightOn() | `bool bOn` | 切换灯光开关状态并触发更新。 |
| SetLightType() | `ELightType Type` | 设置灯光类型（如车灯、路灯等）。 |

#### GetLightState() 与 SetLightState()
- **作用**：实现与 RPC 协议的状态双向同步。
- **数据结构**：
  ```cpp
  carla::rpc::LightState {
    FVector _location;      // 世界坐标位置
    float _intensity;       // 强度
    LightGroup _group;      // 类型（枚举）
    FLinearColor _color;    // 颜色
    bool _active;           // 是否开启
    int _id;                // 唯一标识符
  }
  ```

### 2.4 辅助功能<a id="24-辅助功能"></a>

#### GetLocation()
- **作用**：获取灯光全局坐标（支持大型地图场景）。
- **流程**：
  1. 获取组件所属 Actor 的本地坐标。
  2. 通过 `ALargeMapManager` 将坐标转换为全局坐标系。

#### RecordLightChange()
- **作用**：记录灯光状态变化事件（用于仿真回放）。
- **触发条件**：调用 `SetLightColor()` 或 `SetLightOn()`。
- **依赖模块**：通过 `UCarlaStatics::GetCurrentEpisode()` 获取记录器实例。

## 3 关键成员变量<a id="3-关键成员变量"></a>

| 变量名 | 类型 | 描述 |
| --- | --- | --- |
| LightIntensity | `float` | 当前灯光强度（默认值由引擎配置） |
| LightColor | `FLinearColor` | RGB 颜色值（线性空间） |
| bLightOn | `bool` | 是否开启 |
| LightType | `ELightType` | 灯光分类（如车辆、环境光源等） |
| Id | `int` | 唯一标识符（由子系统分配） |

## 4 使用场景<a id="4-使用场景"></a>

### 4.1 动态环境光照<a id="41-动态环境光照"></a>
```cpp
// 脚本控制路灯夜间自动开启
LightComponent->SetLightOn(true);
LightComponent->SetLightIntensity(5000.0f);
LightComponent->SetLightColor(FLinearColor(0.9f, 0.9f, 0.8f));
```

### 4.2 车辆信号灯同步<a id="42-车辆信号灯同步"></a>
```cpp
// 同步刹车灯状态到 RPC 协议
carla::rpc::LightState state = LightComponent->GetLightState();
state._color = FLinearColor::Red;
state._active = bIsBraking;
LightComponent->SetLightState(state);
```

### 4.3 仿真事件回放<a id="43-仿真事件回放"></a>
在事件回放上通过调用 `RecordLightChange()` 记录状态变化时间点。

## 5 注意事项<a id="5-注意事项"></a>

- **子系统依赖**：Actor 必须存在于已启用 `CarlaLightSubsystem` 的场景中。
- **线程安全**：避免在非游戏线程（如异步任务）中直接调用 `SetLightXxx` 函数。
- **大型地图支持**：使用 `GetLocation()` 而非直接获取 Actor 坐标以确保跨地图兼容性。

## 6 扩展建议<a id="6-扩展建议"></a>

使用序列化优化重写 `Serialize()` 实现灯光状态的持久化存储（如保存/加载场景）。RPC性能优化上对高频更新的灯光（如转向灯）实现差值同步，减少网络流量。

---

