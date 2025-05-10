## 道具蓝图

* 道具工厂 [PropFactory](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Props/PropFactory.uasset)


# 道具/Props说明文档

## 目录

1. [项目概述](#1项目概述)
2. [核心组件与蓝图结构](#2核心组件与蓝图结构)
    - [BP_BaseProp](#21bp_baseprop)
    - [BP_SpecificProp_Type](#22bp_specificprop_type)
    - [BP_MultipleProps](#23bp_multipleprops)
    - [PropIdSetter](#24propidsetter)
    - [ScenePropsParameters 数据资产](#25scenepropsparameters-数据资产)
3. [关键功能实现](#3关键功能实现)
    - [道具实例化与初始化](#31道具实例化与初始化)
    - [道具控制接口](#32道具控制接口)
    - [ID 分配机制](#33id-分配机制)
    - [与动态系统的集成](#34与动态系统的集成)
    - [性能优化考虑](#35性能优化考虑)
4. [工作流程](#4工作流程)
5. [优化与扩展建议](#5优化与扩展建议)
    - [文件格式支持扩展（不适用）](#51文件格式支持扩展不适用)
    - [异常处理与日志记录](#52异常处理与日志记录)
    - [性能优化](#53性能优化)
    - [单元测试](#54单元测试)
    - [更多道具特性](#55更多道具特性)
6. [总结](#6总结)

---

## 1. 项目概述

道具系统在虚拟环境中扮演着重要角色，不仅能够丰富场景的细节，还能为仿真环境带来更多的交互性和沉浸感。本模块专注于为场景中的各种道具（如家具、交通标识、街头设备等）设计并实现蓝图系统。通过结构化的蓝图和统一的控制接口，使得道具的配置、管理与控制更加灵活且高效，并能与模拟器的其他动态系统（如环境变化、角色互动）实现无缝集成。

本道具系统模块参考了类似 CARLA 模拟器在物体蓝图组织上的实践，旨在提供一个高效、易扩展且高度可定制的解决方案，既能满足仿真需求，又具有较强的可维护性。

### 模块依赖与集成

道具系统模块作为场景的核心组成部分，与模拟器的多个系统存在紧密交互：

- **虚幻引擎渲染管线**: 利用UE4的物理和渲染特性，确保道具的表现效果符合预期。
- **场景管理系统**: 通过道具蓝图实例的放置，关卡设计师能够创建场景中的各种道具。
- **模拟器控制接口 (例如 Python API)**: 允许外部系统通过 ID 等方式控制特定道具的属性或状态。
- **动态环境系统**: 道具系统需要与时间、天气等动态系统进行联动，确保道具与环境变化的同步性。

---

## 2. 核心组件与蓝图结构

道具系统由多个核心蓝图组件组成，以下是主要的构成部分：

### i. BP_BaseProp

- **功能**: 作为所有具体道具类型蓝图的基类，封装了道具的基本属性和通用功能。
- **设计**:
  - 包含一个或多个 `StaticMeshComponent`，表示道具的物理外观。
  - 定义了通用的变量，如 `PropName`, `PropType`, `IsInteractive`（是否可互动）等，这些变量可以在编辑器中修改或通过函数设置。
  - 提供基础的控制函数，如 `SetState(bool NewState)`, `Interact()`等。
  - 可以包含一个用于存储唯一 ID 的变量（如 `PropID`，类型为 String 或 Integer）。
- **作用**: 提供道具蓝图的复用基础，确保各类型道具能够共享基础逻辑。

### ii. BP_SpecificProp_Type

- **功能**: 从 `BP_BaseProp` 继承，用于表示特定类型的道具，如 **`BP_Table`**, **`BP_TrafficSign`**, **`BP_StreetLamp`** 等。
- **设计**:
  - 继承 `BP_BaseProp` 的组件和函数。
  - 在 `Construction Script` 中配置道具的模型 (`StaticMeshComponent`) 和可变属性。
  - 可以设置道具的交互属性，如点击时是否播放动画、改变状态等。
  - 如果需要，可以为该道具类型添加独有的变量或函数（例如，交通标志可能有表示当前状态的变量以及用于切换状态的函数）。
  - 在 `BeginPlay` 中通过读取 `ScenePropsParameters` 数据资产来初始化自身属性。
- **作用**: 为不同外观和功能的道具提供具体的蓝图资产，便于关卡设计师在场景中使用和配置。

#### 蓝图实现示例（BeginPlay 初始化）

```cpp
// 伪代码示例
Event BeginPlay
    // 加载场景道具参数数据资产
    LocalVariable PropsParams = LoadDataAsset("ScenePropsParametersAssetReference");

    // 根据道具类型查找对应的配置
    LocalVariable MyConfig = PropsParams.FindConfigForPropType(Self.PropTypeIdentifier);

    If MyConfig Is Valid
        // 设置道具的初始状态和属性
        StaticMeshComponent.SetStaticMesh(MyConfig.PropMesh);
        SetState(MyConfig.DefaultState);
    End If
End Event
````

### iii. BP\_MultipleProps

* **功能**: 一个控制器蓝图，用于管理场景中的一组相关道具实例。
* **设计**:

  * 包含一个 `Array` 变量，存储 `BP_BaseProp` 或其子类道具的引用。这些引用可以在编辑器中手动指定，或在运行时动态获取。
  * 提供对该组道具进行批量操作的函数，如 `SetGroupState(bool NewState)`、`InteractWithGroup()` 等，允许同时修改一组道具的状态。
* **作用**: 便于对一组道具进行统一管理和控制（例如，同步开启一组互动道具）。

### iv. PropIdSetter

* **功能**: 用于为场景中的道具分配唯一标识符 (ID) 的蓝图。
* **设计**:

  * 可以在游戏开始时执行 `AssignUniquePropIDs()` 函数。
  * 该函数通过 `GetAllActorsOfClass` 获取所有需要分配 ID 的道具实例。
  * 为每个道具生成唯一 ID，并将其存储在道具的 `Tags` 数组或自定义的 `PropID` 变量中。
* **作用**: 使外部系统能够通过唯一 ID 引用并控制场景中的道具，尤其适用于自动化测试和外部系统集成。

**蓝图示例 (AssignUniquePropIDs 函数)**

```cpp
// 伪代码示例
Function AssignUniquePropIDs

LocalVariable PropActors Array<Actor> = GetAllActorsOfClass(BP_BaseProp);
LocalVariable CurrentID Integer = 0;

For Each Actor In PropActors
    // 生成唯一ID
    LocalVariable NewIDString = "PropID_" + ToString(CurrentID);

    // 将ID作为标签添加到Actor
    Actor.AddTag(FName(NewIDString));

    // 如果 BP_BaseProp 有 PropID 变量，也可以设置它
    // Cast Actor To BP_BaseProp
    // If Cast Successful
    //     BP_BaseProp_Ref.PropID = NewIDString;
    // End If

    CurrentID = CurrentID + 1;
End For Each
End Function
```

### v. ScenePropsParameters 数据资产

* **功能**: 存储场景中不同道具类型的配置参数。
* **设计**:

  * 包含多个结构体，每个结构体代表一种道具类型（例如，`ST_TableConfig`, `ST_TrafficSignConfig`）。
  * 每个结构体包含该类型道具的默认属性（如模型、交互状态、默认材质等）。
  * 蓝图（如 `BP_SpecificProp_Type`）可以在初始化时读取该数据资产，根据自身类型选择相应的配置。
* **作用**: 将道具的配置数据与蓝图代码分离，方便美术和策划人员调整道具属性。

---

## 3. 关键功能实现

### i. 道具实例化与初始化

1. 关卡设计师将 `BP_SpecificProp_Type`（如 `BP_Table`）拖拽到场景中。
2. 在蓝图实例的详情面


板中，关卡设计师可以调整道具的基本属性（如模型、材质、交互功能等）。
3\. 运行时，`BeginPlay` 会触发道具的初始化，蓝图从 `ScenePropsParameters` 数据资产中读取相关配置信息，确保道具的属性正确设置。

### ii. 道具控制接口

* 外部系统（如模拟器控制系统、自动化测试工具）可以通过道具的 `PropID` 唯一标识符来控制道具的行为。
* 常见操作包括：

  * 通过 `SetState()` 函数修改道具状态（如从“关闭”到“开启”）。
  * 调用 `Interact()` 函数模拟玩家与道具的交互。

### iii. ID 分配机制

* 使用 `PropIdSetter` 蓝图为每个道具分配唯一 ID，并在蓝图中存储该 ID。
* 该 ID 可以通过标签、变量或其他方式存储，以供外部系统查询和操作。

### iv. 与动态系统的集成

道具系统能够与模拟器的动态环境系统（如天气、时间）进行联动。例如，某些道具（如灯具）可能需要根据时间或天气自动开关。

### v. 性能优化考虑

* 对于大量道具的管理，使用 `BP_MultipleProps` 控制一组道具的行为，避免对每个道具单独进行操作。
* 道具的物理和视觉效果尽量采用低成本的优化方法（如简化碰撞体积、减少不必要的实时计算）。

---

## 4. 工作流程

1. 关卡设计师在虚幻引擎中使用 `BP_SpecificProp_Type` 将道具拖入场景，并根据需要配置其属性。
2. 当游戏启动时，道具会在 `BeginPlay` 中进行初始化。
3. 外部系统或玩家通过控制接口与道具交互，改变其状态或属性。
4. 每个道具都会有一个唯一的 `PropID`，用于在模拟器中进行唯一标识。

---

## 5. 优化与扩展建议

### i. 文件格式支持扩展（不适用）

当前方案已足够灵活，暂不考虑支持额外的文件格式。

### ii. 异常处理与日志记录

可以在每个蓝图中添加异常处理逻辑，确保在发生错误时进行日志记录，方便后期排查和调试。

### iii. 性能优化

* 使用对象池机制来管理频繁创建和销毁的道具，减少内存的分配和释放。
* 对于非互动道具，尽量避免不必要的物理计算和渲染更新。

### iv. 单元测试

建议为核心功能编写单元测试，确保道具实例化、属性设置和ID分配等功能的准确性。

### v. 更多道具特性

可进一步扩展道具的类型和功能，例如支持更多的动画、状态切换、或与玩家互动的方式。

---

## 6. 总结

道具系统为虚拟场景中的物体提供了灵活的管理方式，允许关卡设计师和开发人员轻松地配置、控制和优化场景中的各类道具。通过高效的蓝图结构和统一的接口，系统不仅能够支持多种道具类型，还能与模拟器的动态环境无缝集成，为仿真场景提供了丰富的交互性。
