# Trigger 文档

## 概述

**Trigger** 是 CARLA 仿真环境中的物理规则触发模块，用于根据轮轴与场景的重叠情况，动态调整车辆轮轴的摩擦系数，以实现场景的动态效果。

该触发器适用于多种场景，例如小路、波浪、雨水、冰面，以及根据路段动态选择和更换摩擦系数。它支持动态变化摩擦系数和多种触发距离样式，能够显著增强仿真环境的真实性和复杂度。

**Trigger** 具有良好的模块化特性，便于开发者进行二次开发和自定义扩展。

---

## 结构化设计

**Trigger** 采用结构化设计，将物理操作、生成方法和应用场景分层，通过触发器工厂 (TriggerFactory) 和触发器 (Trigger) 维持低耦合、高内聚的设计原则，确保模块易于维护和扩展。

- **Trigger**: 负责触发区域检测和轮轴摩擦系数调整。
- **TriggerFactory**: 提供触发器模板定义和动态生成。
- **ActorBlueprintFunctionLibrary**: 辅助函数，用于触发器属性配置。
- **CarlaStatics/CarlaGameInstance**: 对接 CARLA 游戏环境，获取相关单元和世界信息。

这种架构支持快速添加更多类型的触发器（如雨天打滑、冰面等）以及自定义摩擦逻辑。

---

## 架构

### 概述

- **World**: 场景检测的起点。
- **TriggerFactory**: 根据配置生成触发器。
- **Trigger**: 实现触发区域和摩擦系数维护。

这种责任划分使得系统能够灵活应对大规模、复杂场景的仿真需求。

---

## 使用 Trigger

### 数据输入与输出

| 概念            | 描述                                   |
|-----------------|----------------------------------------|
| TriggerVolume   | 给定触发区域，检测车辆进入或离开。      |
| Friction        | 设置新的摩擦系数，改变车辆在不同表面的驾驶行为。 |
| OldFriction     | 记录车辆进入前的原有摩擦系数，方便离开后恢复。 |

### 创建 Trigger

触发器通过触发器工厂动态生成，并注入关卡中：

```cpp
auto *Trigger = World->SpawnActorDeferred<ATrigger>(
    Description.Class,
    Transform,
    this,
    nullptr,
    ESpawnActorCollisionHandlingMethod::AlwaysSpawn);

Trigger->SetFriction(3.5f);
Trigger->SetBoxExtent(FVector(100.0f, 100.0f, 100.0f));
UGameplayStatics::FinishSpawningActor(Trigger, Transform);
```

可以在蓝图或 C++ 中动态调整摩擦力大小与触发体积范围，以适配不同场景。

### 触发操作

- **OnTriggerBeginOverlap**: 将轮轴摩擦系数调整到特定值。
- **OnTriggerEndOverlap**: 恢复轮轴的原有摩擦系数。

```cpp
void OnTriggerBeginOverlap(...)
{
    OldFrictionValues = Vehicle->GetWheelsFrictionScale();
    UpdateWheelsFriction(Vehicle, {Friction, Friction, Friction, Friction});
}

void OnTriggerEndOverlap(...)
{
    UpdateWheelsFriction(Vehicle, OldFrictionValues);
}
```

通过这种机制，车辆在不同地形表面（如湿滑地面、结冰路段、起伏道路）能够自动改变驾驶特性。

### 管理触发区

- **Init()**: 绑定开始/结束重叠事件。
- **EndPlay()**: 卸载触发器时进行清理，避免内存泄漏。

```cpp
void Init()
{
    TriggerVolume->OnComponentBeginOverlap.AddDynamic(this, &ATrigger::OnTriggerBeginOverlap);
    TriggerVolume->OnComponentEndOverlap.AddDynamic(this, &ATrigger::OnTriggerEndOverlap);
}

void EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    TriggerVolume->OnComponentBeginOverlap.RemoveDynamic(this, &ATrigger::OnTriggerBeginOverlap);
    TriggerVolume->OnComponentEndOverlap.RemoveDynamic(this, &ATrigger::OnTriggerEndOverlap);
}
```

---

## 运行模式

### Trigger 模块功能

| 功能            | 描述                                   |
|-----------------|----------------------------------------|
| 自动缓存/恢复   | 记录并恢复车辆原有轮轴摩擦系数，确保仿真连贯。 |
| 触发场景选择    | 检测车辆是否进入触发区域，根据场景需求动态调整摩擦力。 |
| 摩擦系数调整    | 在触发区域内自动改变车辆行驶性能，模拟不同地表特性。 |
| 高效清理机制    | 组件卸载时自动解绑事件，避免资源泄漏。     |

### TriggerFactory 工厂功能

| 功能        | 描述                                           |
|-------------|-----------------------------------------------|
| 触发器定义  | 通过 MakeTriggerDefinition 生成 "friction" 类型定义。 |
| 动态创建    | 读取摩擦系数与区域范围参数，动态生成配置触发器。 |

---

## 示例场景应用

| 场景            | 效果描述                               |
|-----------------|---------------------------------------|
| 湿滑路段        | 进入时减小摩擦系数，增加打滑概率，测试控制算法稳定性。 |
| 冰面行驶        | 摩擦系数极低，考验自动驾驶在极端条件下的决策能力。 |
| 泥泞路段        | 中等降低摩擦，考察驱动与避障能力。         |
| 干燥路面（默认摩擦） | 离开特殊区域后恢复正常摩擦系数，确保驾驶逻辑连贯。 |

---

## 总结

- **Trigger** 通过检测车辆与触发区域的重叠状态，在进入时更改轮轴摩擦系数，离开时恢复，实现动态场景效果。
- **TriggerFactory** 提供模块定义与动态生成，便于配置和扩展，与环境添加机制无缝对接。
- **适用场景**: 波浪路面、滑路行驶、道路故障和触发监测等仿真场景。
- **未来拓展**: 可基于此架构快速扩展更多类型触发器，如湿度感知、温度变化、轮胎磨损模拟等。
