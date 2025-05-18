# 灯光蓝图

* 灯光 [BP_Lights](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Lights/BP_Lights.uasset)
* 多灯 [BP_multipleLights](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Lights/BP_multipleLights.uasset)
* 灯ID设置器 [LightIdSetter](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Lights/LightIdSetter.uasset)

# 人车模拟器虚幻场景灯光系统技术文档

## 目录

1. [项目概述](#1项目概述)
2. [核心组件与蓝图结构](#2核心组件与蓝图结构)
    1. [BP_BaseLight](#21bp_baselight)
    2. [BP_SpecificLight_Type](#22bp_specificlight_type)
    3.[BP_MultipleLights](#23bp_multiplelights)
    4.[LightIdSetter](#24lightidsetter)
    5.[SceneLightingParameters 数据资产](#25scenelightingparameters-数据资产)
3. [关键功能实现](#3关键功能实现)
    1.[灯光实例化与初始化](#31灯光实例化与初始化)
    2.[灯光控制接口](#32灯光控制接口)
    3. [ID 分配机制](#33id-分配机制)
    4. [与动态系统的集成](#34与动态系统的集成)
    5. [性能优化考虑](#35性能优化考虑)
4. [工作流程](#4工作流程)
5. [优化与扩展建议](#5优化与扩展建议)
    1. [文件格式支持扩展（不适用）](#51文件格式支持扩展不适用)
    2. [异常处理与日志记录](#52异常处理与日志记录)
    3. [性能优化](#53性能优化)
    4. [单元测试](#54单元测试)
    5. [更多灯光特性](#55更多灯光特性)
6. [总结](#6总结)

---

## 1. 项目概述

本模块以 Unreal Engine 作为渲染核心，通过蓝图化体系对场景中的人工光源（路灯、建筑投光、交通信号灯等）进行结构化管理。所有灯具参数（光强、色温、衰减曲线、IES 曲线等）均以 Data Asset 形式数据化保存，并通过统一的 Blueprint 接口实现对光强、色彩与闪烁效果的编程控制，实时响应昼夜循环、气象系统和交通信号相位等全局事件。借鉴 CARLA 在蓝图组织和资产管理方面的最佳实践，本系统采用 C++ 与蓝图混合开发，兼顾高性能渲染与后续扩展维护的灵活性。在夜间、黄昏、黎明及雨雾等特殊场景下，高精度的人工灯光渲染不仅提升了视觉真实感，也显著增强了车载摄像头和深度传感器的环境感知准确性，为自动驾驶仿真提供了可靠的数据支撑。


### 模块依赖与集成

本灯光系统模块作为一个重要的场景组件，与模拟器的多个系统存在交互：

*   **虚幻引擎渲染管线**: 直接利用 UE 的灯光组件和渲染特性。
*   **场景管理系统**: 关卡设计师通过放置灯光蓝图实例来构建场景。
*   **模拟器控制接口 (例如 Python API)**: 外部系统可能需要通过 ID 等方式控制特定灯光的开关或属性。
*   **动态环境系统 (昼夜循环/天气)**: 灯光系统需要接收来自这些系统的指令，根据时间和天气自动调整状态。


![灯光系统依赖草图](../img/light_BP.png)
该图展示了灯光系统模块如何与虚幻引擎渲染、场景管理以及模拟器的动态系统交互，提供受控的人工照明。

---

## 2. 核心组件与蓝图结构

本灯光系统由以下核心蓝图资产构成，它们组织在虚幻引擎的内容浏览器中，例如位于 `/Game/Blueprints/Scene/Lights/` 目录下：

### 1. BP_BaseLight

*   **功能**: 作为所有具体人工光源蓝图的基类。封装了标准人工光源所需的基本组件和通用逻辑。
*   **设计**:
    *   包含一个或多个 `LightComponent` (如 SpotLightComponent, PointLightComponent)，具体类型可根据需要或在子类中指定。
    *   包含 `StaticMeshComponent` 用于表示灯具模型（可选）。
    *   定义了通用的变量，如 `LightColor`, `LightIntensity`, `AttenuationRadius` 等，这些变量可在编辑器中修改或通过函数调用设置。
    *   提供了基础控制函数，如 `TurnOn()`, `TurnOff()`, `SetIntensity(float NewIntensity)`, `SetColor(FLinearColor NewColor)`。
    *   可以包含一个用于存储唯一 ID 的变量 (如 `LightID`，类型为 String 或 Integer)。
*   **作用**: 提供了灯光蓝图代码的复用基础和统一的控制接口。

### 2. BP_SpecificLight_Type

*   **功能**: 从 `BP_BaseLight` 继承，用于表示特定类型的灯光实例，如 **`BP_Streetlight_Modern`**, **`BP_BuildingLight_Window`**, **`BP_TrafficLight_Simple`** 等。
*   **设计**:
    *   继承 `BP_BaseLight` 的组件和函数。
    *   在 `Construction Script` 中配置特定的灯具网格体资产 (`StaticMeshComponent`)。
    *   可以设置默认的灯光属性值（如特定路灯的默认强度、颜色）。
    *   如果需要，可以添加该特定类型灯光独有的变量或函数（例如，交通信号灯蓝图可能包含一个表示当前状态的枚举变量和用于切换状态的函数）。
    *   可以在 `BeginPlay` 事件中读取数据资产 (`SceneLightingParameters`) 来初始化自身的属性。
*   **作用**: 为不同外观和用途的灯光提供具体的资产，便于关卡设计师在场景中使用和配置。

#### 蓝图实现示例（BeginPlay 初始化）

```cpp
// 这是一个概念性的蓝图伪代码示例
// Blueprint Pseudo-Code: 高级灯光配置加载
Event BeginPlay
    // 1. 加载场景光照参数数据资产
    Const UDataAsset* SceneLightingDA = Cast<UDataAsset>(LoadAsset("/Game/Configs/SceneLighting/SceneLightingParams"));
    If Not IsValid(SceneLightingDA)
        LogError("SceneLightingParams 加载失败，检查资产路径或引用")
        Return
    End If

    // 2. 按 LightTypeIdentifier 查找对应的配置，失败时使用默认
    FLightConfig LightCfg
    If Not SceneLightingDA.GetConfigByType(LightTypeIdentifier, Out LightCfg)
        LogWarning("未找到 LightType '%s' 的配置，采用 DefaultConfig", LightTypeIdentifier.ToString())
        LightCfg = SceneLightingDA.DefaultConfig
    End If

    // 3. 将配置应用到 LightComponent
    If IsValid(LightComponent)
        LightComponent.SetIntensity(LightCfg.Intensity)
        LightComponent.SetLightColor(LightCfg.Color)
        LightComponent.SetAttenuationRadius(LightCfg.AttenuationRadius)
        If LightCfg.IESProfileAsset IsValid
            LightComponent.SetIESProfile(LightCfg.IESProfileAsset)
        End If
    End If

    // 4. 更新灯具外观
    If IsValid(StaticMeshComponent) And LightCfg.MeshAsset IsValid
        StaticMeshComponent.SetStaticMesh(LightCfg.MeshAsset)
        If LightCfg.LampMaterial IsValid
            StaticMeshComponent.SetMaterial(0, LightCfg.LampMaterial)
        End If
    End If
End Event
```

### 3. BP_MultipleLights

*   **功能：** 一个控制器蓝图，用于管理场景中的一组相关的 `BP_BaseLight` 实例。
*   **设计：**
    *   包含一个 `Array` 变量，存储 `BP_BaseLight` 或其子类 Actor 的引用。这些引用可以在编辑器中手动指定，或在运行时通过搜索获取。
    *   提供对该组灯光进行批量操作的函数，如 `TurnGroupOn()`、`TurnGroupOff()`、`SetGroupIntensity(float NewIntensity)`。这些函数内部会遍历数组，并对每个灯光 Actor 调用其相应的控制函数。
*   **作用：** 便于对场景中的一组灯光进行统一管理和控制（例如，同时开启或关闭一条街上的所有路灯），减少外部系统需要直接操作大量单个 Actor 的需求。

### 4. LightIdSetter

*   **功能：** 一个放置在场景中的蓝图 Actor 或一个可添加到 Actor 上的 Actor Component，负责在运行时扫描场景并为特定的灯光 Actor 分配唯一的标识符 (ID)。
*   **设计：**
    *   可以设计为在游戏开始 (`BeginPlay`) 时执行一个函数 `AssignUniqueIDs()`。
    *   该函数使用 `GetAllActorsOfClass` 获取所有需要分配 ID 的灯光 Actor（例如，所有 `BP_BaseLight` 的实例）。
    *   生成唯一的 ID（例如，基于递增计数器、GUID、或基于 Actor 名称/位置的哈希）。
    *   将生成的 ID 存储在每个灯光 Actor 的 `Tags` 数组或其自定义的 `LightID` 变量中。
    *   需要考虑 ID 的持久性（如果场景需要保存/加载）和在运行时动态生成灯光的处理。
*   **作用：** 使得外部系统能够通过唯一的 ID 精准地引用和控制场景中的任何一个或一组灯光 Actor。这是实现外部控制和自动化测试的关键。

**蓝图示例 (AssignUniqueIDs 函数)**

```cpp
// 这是一个概念性的蓝图伪代码示例
// Blueprint Pseudo-Code: 为所有基灯 Actor 分配全局唯一 ID
Function AssignUniqueLightActorIDs()
    // 1. 获取场景中所有继承自 BP_LightBase 的 Actor
    Local Variable LightActors = GetAllActorsOfClass(BP_LightBase)
    If LightActors.IsEmpty()
        LogWarning("AssignUniqueLightActorIDs：未发现任何 BP_LightBase 实例，跳过 ID 分配")
        Return
    End If

    // 2. 遍历并为每个灯光 Actor 生成并赋予唯一 ID
    For Index = 0 To LightActors.Num() - 1
        Local Variable Actor = LightActors[Index]
        If Not IsValid(Actor)
            Continue
        End If

        // 2.1 生成全局唯一 GUID
        Local Variable NewGUID = FGuid.NewGuid()
        Local Variable IDString = "LightID_" + NewGUID.ToString(EGuidFormats.DigitsWithHyphens)

        // 2.2 移除旧的 LightID_ 标签（如有）并添加新标签
        For Each Tag In Actor.Tags
            If Tag.ToString().StartsWith("LightID_")
                Actor.RemoveTag(Tag)
            End If
        End For
        Actor.AddTag(FName(*IDString))

        // 2.3 如果 Actor 包含 LightID 字段，则同步更新
        Local Variable LightBaseRef = Cast<BP_LightBase>(Actor)
        If IsValid(LightBaseRef)
            LightBaseRef.LightID = IDString
        Else
            LogError("AssignUniqueLightActorIDs：Actor {0} 无法转换为 BP_LightBase", Actor.GetName())
        End If
    End For
End Function
```

### 5. SceneLightingParameters 数据资产

*   **功能**: 这是一个 Data Asset 类型的资产，用于集中存储场景中不同类型灯光的配置参数。
*   **设计**:
    *   包含一个或多个结构体 (`Struct`) 定义，每个结构体代表一种灯光类型 (例如，`ST_StreetlightConfig`, `ST_BuildingLightConfig`)。
    *   每个结构体包含该类型灯光的默认属性 (如默认强度、颜色、衰减半径、灯具模型资产引用、是否受昼夜循环影响等)。
    *   蓝图 (如 `BP_SpecificLight_Type`) 可以在初始化时读取这个数据资产，根据自身的类型获取对应的配置参数来设置自身属性。
*   **作用**: 将灯光的配置数据与蓝图代码分离，便于美术和策划人员在不修改蓝图代码的情况下调整灯光参数，提高了数据的管理效率和灵活性。

### 3. 关键功能实现

本节描述灯光蓝图系统的主要功能及其实现方式。

#### 1. 灯光实例化与初始化

**实现过程**:

Level designers 只需将如 BP_Streetlight_Modern 等 BP_SpecificLight_Type 实例拖入关卡，然后在 Details 面板中通过公开的变量（强度、色温、衰减半径等）覆盖蓝图默认值；在 Construction Script 中会根据配置自动设置 StaticMeshComponent 以渲染正确的灯具模型；运行时，BeginPlay 事件首先加载 SceneLightingParameters 数据资产，再通过类名、标签或蓝图专用标识查询出对应的 FLightConfig 条目；最后，脚本将该配置中的光强、颜色、IES 曲线等参数应用到 LightComponent，确保全局数据驱动与关卡设计师的个性化调整协同生效。

#### 2. 灯光控制接口

*   **功能**: 提供蓝图内部和外部系统控制灯光状态和属性的能力。
*   **实现过程**:
    1.  在 `BP_BaseLight` 中，实现 `TurnOn`, `TurnOff`, `SetIntensity(float)`, `SetColor(FLinearColor)` 等自定义事件或函数。这些函数直接操作内部的 `LightComponent` 的 `SetVisibility`, `SetIntensity`, `SetLightColor` 等方法。
    2.  在 `BP_MultipleLights` 中，实现 `TurnGroupOn`, `TurnGroupOff` 等函数。这些函数遍历其拥有的 `BP_BaseLight` 引用数组，并对数组中的每个 Actor 调用其相应的控制函数。
    3.  外部系统 (如模拟控制脚本) 可以通过以下方式控制灯光:
        *   直接获取特定灯光 Actor 的引用 (例如，通过其名称，但不推荐用于大量灯光)。
        *   使用 `GetActorsWithTag` 或自定义查找函数，根据 `LightIdSetter` 分配的唯一 ID 找到目标灯光 Actor，然后调用其公共函数。
        *   查找 `BP_MultipleLights` 实例，并调用其批量控制函数。

#### 3. ID 分配机制

*   **功能**: 为场景中的灯光 Actor 分配唯一的运行时 ID，便于外部系统引用。
*   **实现过程**:
    i. 创建一个蓝图 Actor `LightIdSetter` 并将其放置在场景中。
    ii. 在 `LightIdSetter` 的 `BeginPlay` 事件中，执行 `AssignUniqueIDs()` 函数。
    iii. `AssignUniqueIDs()` 函数使用 `GetAllActorsOfClass(BP_BaseLight)` 或其子类，获取所有需要分配 ID 的灯光 Actor 列表。
    iv. 遍历列表，为一个计数器递增生成 ID，或者使用 GUID。
    v. 将生成的 ID 存储在当前灯光 Actor 的 `Tags` 数组中 (使用 `AddTag(FName)`)。例如，将 ID 格式化为 `"LightID_123"` 这样的标签。
    vi. 外部系统可以通过 `GetAllActorsWithTag(FName("LightID_123"))` 查找具有特定 ID 的灯光。

#### 4. 与动态系统的集成

*   **功能**: 使灯光系统能够响应昼夜循环和天气变化。
*   **实现过程**:
    i. 模拟器的 TimeOfDay/Weather Manager 蓝图 (假设存在) 在时间和天气变化时触发事件或调用函数。
    ii. TimeOfDay/Weather Manager 获取相关的灯光 Actor 或 `BP_MultipleLights` 实例的引用。这可以通过直接引用、按标签查找 ID、或查找特定的灯光组控制器来实现。
    iii. 例如，当时间到达黄昏时，Manager 调用所有路灯组 (如由一个 `BP_MultipleLights` 管理的组，或者通过遍历所有带有 `"Streetlight"` 标签的 Actor) 的 `TurnGroupOn()` 函数。
    iv. Manager 也可以根据天气强度 (如雨量、雾密度) 调整灯光的强度，调用 `SetGroupIntensity()` 或单个灯光的 `SetIntensity()` 函数。

### 5. 性能优化考虑

*   **功能**: 确保场景中大量灯光不会导致性能下降。
*   **实现过程**:
    i. 在 `BP_BaseLight` 的 `LightComponent` 设置中，配置适当的 `Attenuation Radius` (衰减半径)，使灯光只影响局部区域。
    ii. 根据需要调整灯光的 `Cast Shadows` 设置。动态阴影对性能影响较大，对于不关键的灯光可以禁用。
    iii. 对于外观相同的灯具网格体，考虑在 `BP_SpecificLight_Type` 中使用 `Hierarchical Instanced Static Mesh` (HISM) 组件来代替多个 `StaticMeshComponent`，减少 Draw Call。
    iv. 利用虚幻引擎的 Level of Detail (LOD) 和剔除机制。

### 4. 工作流程

本灯光系统模块的典型使用工作流程如下:

1.  **数据资产配置**: 美术或关卡设计师编辑 `SceneLightingParameters` 数据资产，定义不同类型灯光的默认外观和性能参数。
2.  **蓝图资产创建与修改**: 蓝图工程师创建或修改 `BP_BaseLight` 及其子类 `BP_SpecificLight_Type` 蓝图，设置灯具模型、组件，实现基础控制函数，并在 `BeginPlay` 中添加数据资产读取参数的逻辑。
3.  **场景布局**: 关卡设计师在虚幻场景中放置 `BP_SpecificLight_Type` 实例来构建人工照明环境。对于需要成组控制的灯光，将其引用添加到 `BP_MultipleLights` 实例的数组中。
4.  **ID 分配配置**: 在场景中放置 `LightIdSetter` Actor，确保在游戏开始时能够正确执行 ID 分配逻辑。

### 5. 优化与扩展建议

基于当前的设计，可以考虑以下优化和未来扩展方向:

#### 1. 文件格式支持扩展 (不适用)

*   本模块主要处理虚幻引擎内的蓝图资产和运行时 Actor，不涉及外部文件格式的读写，因此此项不适用。

#### 2. 异常处理与日志记录

建议在蓝图与底层 C++ 混合实现中统一引入严谨的校验与日志机制：对所有外部引用（Actor、Component、DataAsset 等）均调用 IsValid() 或 ensure()，一旦发现无效引用立即提前返回或触发断言，避免后续执行空指针操作导致崩溃；同时为灯光子系统定义专属日志分类（例如 DECLARE_LOG_CATEGORY_EXTERN(LogLightSystem, Log, All)），并在关键路径（如 ID 分配失败、数据资产加载异常、引用失效等）使用 UE_LOG(LogLightSystem, Warning, TEXT(“LightID 分配失败：%s 无法转换为 BP_LightBase”), *ActorName) 或 UE_LOG(…, Error, TEXT(“加载 SceneLightingParameters 失败，请检查资产路径”)) 输出带有上下文信息的日志，以便快速定位与追踪问题。

#### 3. 性能优化

*   对复杂场景进行 Lightmap Baking (烘焙静态灯光)，只保留动态灯光 (如交通信号灯) 为动态。
*   进一步优化灯光参数，例如减少不必要的阴影投射、调整光源的移动性和更新频率。
*   对于非常远距离的灯光，考虑使用简单的发光材质平面来代替实际光源，节省渲染开销。

#### 4. 单元测试

*   虽然蓝图的单元测试框架不如 C++ 成熟，但可以创建专门的测试场景，编写测试蓝图来实例化灯光、调用其函数，并检查属性是否按预期改变。例如，测试 `TurnOn()` 后灯光是否可见、`SetIntensity()` 是否生效。

#### 5. 更多灯光特性

*   **IES Profiles**: 支持使用 IES 文件来定义光源的光强分布，提升灯光效果的真实感。
*   **体积光 (Volumetric Lighting)**: 在有雾或灰尘的场景中，通过体积光效果增加光束的可见性。
*   **灯光破坏**: 实现灯光 Actor 在受到碰撞或特定事件触发时闪烁、闪烁或熄灭的效果。
*   **更高级的交通信号灯**: 实现复杂的交通信号灯逻辑，包括多种模式、闪烁、故障等。

### 6. 总结

本技术文档系统地阐述了基于 Unreal Engine 的车载模拟器人工光源蓝图架构，涵盖框架设计、核心组件、关键功能实现及运行流程。系统以 C++ 与蓝图混合开发为基础，采用继承化组件结构承载各类灯具逻辑，依托集中化 Data Asset 管理光照参数，并通过全局唯一 ID 驱动的控制接口实现运行时统一调度与扩展。方案原生支持路灯、建筑投光、交通信号灯等多种光源类型，并能与昼夜循环、气象系统、交通信号等子系统深度联动，为夜间、黄昏、雨雾等复杂场景下的高保真渲染提供坚实基础。未来可沿着物理渲染精度优化、Lightmass 预计算加速、IES 曲线及体积光效增强等方向持续迭代，以进一步提升系统的性能、真实感和功能复杂度。

