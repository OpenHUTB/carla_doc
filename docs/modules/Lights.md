
---
# UCarlaLight 说明文档
---
## 概述

`UCarlaLight` 是 CARLA 仿真平台中用于管理动态灯光的 Unreal Engine 组件类，核心功能包括：

- **灯光属性控制**：动态调整灯光强度、颜色、开关状态及类型。
- **子系统集成**：与 `CarlaLightSubsystem` 交互实现全局灯光管理。
- **状态同步**：支持与 RPC 协议兼容的灯光状态序列化（`carla::rpc::LightState`）。
- **事件记录**：灯光状态变化时触发事件记录，支持仿真回放与调试。

## 类成员函数说明

### 1. 生命周期函数

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

### 2. 灯光注册与注销

#### RegisterLight()
- **作用**：将灯光注册到子系统和天气系统。
- **关键逻辑**：
  - 通过 `GetWorld()->GetSubsystem<UCarlaLightSubsystem>()` 获取子系统实例。
  - 调用 `CarlaLightSubsystem->RegisterLight(this)` 完成注册。
  - 标记 `bRegistered = true` 防止重复注册。
- **注销逻辑**：在 `EndPlay()` 中调用 `CarlaLightSubsystem->UnregisterLight(this)`。

### 3. 属性控制函数

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

### 4. 辅助功能

#### GetLocation()
- **作用**：获取灯光全局坐标（支持大型地图场景）。
- **流程**：
  1. 获取组件所属 Actor 的本地坐标。
  2. 通过 `ALargeMapManager` 将坐标转换为全局坐标系。

#### RecordLightChange()
- **作用**：记录灯光状态变化事件（用于仿真回放）。
- **触发条件**：调用 `SetLightColor()` 或 `SetLightOn()`。
- **依赖模块**：通过 `UCarlaStatics::GetCurrentEpisode()` 获取记录器实例。

## 关键成员变量

| 变量名 | 类型 | 描述 |
| --- | --- | --- |
| LightIntensity | `float` | 当前灯光强度（默认值由引擎配置） |
| LightColor | `FLinearColor` | RGB 颜色值（线性空间） |
| bLightOn | `bool` | 是否开启 |
| LightType | `ELightType` | 灯光分类（如车辆、环境光源等） |
| Id | `int` | 唯一标识符（由子系统分配） |

## 使用场景

### 动态环境光照
```cpp
// 脚本控制路灯夜间自动开启
LightComponent->SetLightOn(true);
LightComponent->SetLightIntensity(5000.0f);
LightComponent->SetLightColor(FLinearColor(0.9f, 0.9f, 0.8f));
```

### 车辆信号灯同步
```cpp
// 同步刹车灯状态到 RPC 协议
carla::rpc::LightState state = LightComponent->GetLightState();
state._color = FLinearColor::Red;
state._active = bIsBraking;
LightComponent->SetLightState(state);
```

### 仿真事件回放
通过 `RecordLightChange()` 记录状态变化时间点。

## 注意事项

- **子系统依赖**：Actor 必须存在于已启用 `CarlaLightSubsystem` 的场景中。
- **线程安全**：避免在非游戏线程（如异步任务）中直接调用 `SetLightXxx` 函数。
- **大型地图支持**：使用 `GetLocation()` 而非直接获取 Actor 坐标以确保跨地图兼容性。

## 扩展建议

### 蓝图集成
将 `LightIntensity`、`LightColor` 等属性暴露为 `BlueprintReadWrite`，方便设计师调整。

### 序列化优化
重写 `Serialize()` 实现灯光状态的持久化存储（如保存/加载场景）。

### RPC 性能优化
对高频更新的灯光（如转向灯）实现差值同步，减少网络流量。

---