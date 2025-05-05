# Vegetation(植被)模块说明文档


# BaseVegetationActor 植被物理模拟模块说明文档

- [1. 模块概述](#1-模块概述)
- [2. 详细设计说明](#2-详细设计说明)
  - [2.1 植被物理参数系统（FSpringBasedVegetationParameters）](#21-植被物理参数系统fspringbasedvegetationparameters)
  - [2.2 骨骼物理建模（FSkeletonBone, FSkeletonJoint）](#22-骨骼物理建模fskeletonbone-fskeletonjoint)
  - [2.3 植被基类（ABaseVegetationActor）](#23-植被基类abasevegetationactor)
  - [2.4 弹簧模拟组件（USpringBasedVegetationComponent）](#24-弹簧模拟组件uspringbasedvegetationcomponent)
- [3. 使用方法详解](#3-使用方法详解)
- [4. 技术机制与注意事项](#4-技术机制与注意事项)

---

## 1. 模块概述

本模块用于 CARLA 模拟器中的植物物理动画控制，结合弹簧力学与骨骼动力学建模，实现植物对象（如草、树枝）的自然运动模拟。

该模块主要由以下四个核心文件组成：

- `BaseVegetationActor.h` / `.cpp`
- `SpringBasedVegetationComponent.h` / `.cpp`

其功能包括植被弹簧参数建模、骨架结构描述、动力学求解、碰撞反馈与编辑器交互支持。

---

## 2. 详细设计说明

### 2.1 植被物理参数系统（FSpringBasedVegetationParameters）

该结构体描述了植被动画模拟中核心的弹簧参数，定义于 `BaseVegetationActor.h`。主要字段包括：

- `BaseSpringStrength`：基础弹簧强度。
- `MinSpringStrength`：最小弹簧强度，用于限制。
- `Alpha` / `Beta`：角速度积分参数，控制阻尼效果。
- `Gravity`：植被受力方向。
- `HorizontalFalloff` / `VerticalFalloff`：弹簧力随距离的衰减。
- `CollisionForceParameter` / `CollisionForceMinVel`：碰撞触发机制参数。
- `Skeleton`：包含骨骼结构的数组，用于建立物理模拟结构。

参数可以通过编辑器或运行时蓝图接口设置。

### 2.2 骨骼物理建模（FSkeletonBone, FSkeletonJoint）

定义于 `SpringBasedVegetationComponent.h`，构成物理弹簧网络的基础单位。

- `FSkeletonBone`：表示每节骨骼的物理属性（质量、长度、质心位置）。
- `FSkeletonJoint`：描述骨骼连接关系（JointId、ParentId、位置、旋转等）。

这些结构在组件初始化时用于构建拓扑结构与刚体质量矩阵。

### 2.3 植被基类（ABaseVegetationActor）

该类继承自 UE 的 `AActor`，主要职责：

- 维护并提供 `FSpringBasedVegetationParameters` 的管理接口。
- 支持蓝图接口，如 `SetParametersToComponent()`、`GetParametersFromComponent()`。
- 在 `BeginPlay()` 时自动初始化组件与参数同步。
- 通过绑定的 `USpringBasedVegetationComponent` 实现动态控制。

### 2.4 弹簧模拟组件（USpringBasedVegetationComponent）

该组件定义了完整的物理动画系统，核心职责包括：

- 通过旋转矩阵与惯性张量推进骨骼角动量与姿态。
- 实现角速度更新、姿态积分（如角动量守恒、阻尼恢复力等）。
- 处理外部碰撞信息（Capsule、车辆等），并添加外力反应。
- 通过 Tick 每帧推进物理求解，处理多个骨骼点联动。
- 使用 Eigen 实现矩阵变换、力矩求解、高效模拟。

---

## 3. 使用方法详解

1. 在场景中放置继承自 `ABaseVegetationActor` 的蓝图或 C++ 对象。
2. 通过编辑器或代码设置 `SpringParameters`。
3. 保证其包含 `USpringBasedVegetationComponent`，用于弹簧求解。
4. 可在运行时动态设置参数并实时刷新模拟效果：

```cpp
SpringActor->GetParametersFromComponent();
SpringActor->SpringParameters.BaseSpringStrength = 1200.0f;
SpringActor->SetParametersToComponent();
```

---

## 4. 技术机制与注意事项

- 所有力学运算基于 Eigen 三维向量矩阵库完成。
- 姿态更新采用角速度积分方式，避免欧拉角爆炸。
- 植被组件默认每帧自动调用 `UpdateSkeleton()`。
- 模拟粒度可控：可通过调整骨骼数量、质量矩阵降低性能开销。
- 建议限制 `ForceMaxDistance` 以避免远程对象影响植被模拟。

---

## 1.植被物理参数系统（FSpringBasedVegetationParameters）

该结构体定义在 `BaseVegetationActor.h`，其作用是统一存储所有控制植被物理行为的可调参数，便于在运行时从组件同步参数、在编辑器中配置或蓝图控制中调整。

### 参数说明：

| 参数名 | 类型 | 描述 |
|--------|------|------|
| `BaseSpringStrength` | float | 弹簧的基础恢复力强度，控制弹性摆动的主力度。 |
| `MinSpringStrength` | float | 弹簧恢复力的最小阈值，用于避免数值过小导致的抖动或零弹性。 |
| `SpringStrengthMulFactor` | float | 基于骨骼长度的缩放因子。 |
| `Gravity` | FVector | 指定的重力方向（默认为向下）。 |
| `HorizontalFalloff` / `VerticalFalloff` | float | 水平与垂直方向的衰减参数，用于控制弹力作用范围。 |
| `CollisionForceParameter` | float | 与动态物体碰撞后，附加外力的计算因子。 |
| `CollisionForceMinVel` | float | 达到该速度后才会触发碰撞外力反应。 |
| `ForceMaxDistance` | float | 最大力作用距离，避免远距离对象干扰。 |
| `DeltaTimeOverride` | float | 可用于替代 Tick 中的 DeltaTime。 |
| `Skeleton` | TArray<FSkeletonJoint> | 包含所有骨骼节点的定义。 |

这些参数通过组件接口与 Actor 接口双向同步，确保模拟中数值一致。

---

## 2.弹簧模拟组件（USpringBasedVegetationComponent）

定义于 `SpringBasedVegetationComponent.h` 与 `.cpp` 中。是实现动态物理模拟的核心模块。继承自 `UActorComponent`。

### 核心成员：

- `std::vector<SkeletonNode>`：封装每个骨骼点当前的姿态、角速度、力状态。
- `std::vector<FSkeletonJoint>` / `FSkeletonBone`：构建实际的骨架数据。
- `std::vector<Eigen::Vector3d>`：模拟每一节点的惯性行为。
- `std::unordered_map`：用于管理关节之间的父子拓扑索引。

### Tick 中主要物理流程：

```cpp
for each SkeletonNode {
  ComputeExternalForces();
  IntegrateAngularVelocity();     // 欧拉法积分旋转
  UpdateRotationMatrix();         // 使用 Eigen 计算旋转矩阵
  ApplyDampingAndElasticity();    // 弹性恢复+阻尼力
  HandleCollisionIfNeeded();      // 与车辆等碰撞体交互
}
```

### 关键算法逻辑：

- **角速度积分**：利用公式 `ω += I⁻¹ * τ * Δt` 更新角速度（`I` 为惯性张量，`τ` 为力矩）。
- **旋转更新**：采用矩阵指数或四元数近似实现角度更新，避免万向锁问题。
- **碰撞检测**：使用 Unreal 的 Capsule 组件与 Actor 距离判断是否交互，若触发则添加附加力矩。

---

## 3.使用方法详解

### 蓝图接口说明

- `GetParametersFromComponent()`：将组件中当前参数拉取至 Actor 的 SpringParameters 属性。
- `SetParametersToComponent()`：将 Actor 中编辑的 SpringParameters 推送至组件，刷新模拟参数。

在蓝图中可直接调用如下流程：

```text
事件 BeginPlay
→ 调用 SetParametersToComponent
→ 修改 BaseSpringStrength / Gravity 等参数
→ 调用 GetParametersFromComponent 查看反馈结果
```


