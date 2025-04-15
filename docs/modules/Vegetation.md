## 1. 组件概述
`SpringBasedVegetationComponent` 组件负责模拟植被的弹性运动行为，以提升仿真环境的真实感。

## 2. 代码结构
```
CARLA
│── Unreal
│   └── CarlaUE4
│       └── Plugins
│           └── Carla
│               └── Source
│                   └── Carla
│                       └── Vegetation
```
该组件的主要实现位于 `SpringBasedVegetationComponent.cpp`。

## 3. 依赖项
### 3.1 头文件依赖
`SpringBasedVegetationComponent.cpp` 依赖以下头文件：
- `SpringBasedVegetationComponent.h`
- `Carla/Walker/WalkerAnim.h`
- `Math/Matrix.h`
- `Components/CapsuleComponent.h`
- `DrawDebugHelpers.h`
- `Kismet/KismetMathLibrary.h`
- `BaseVegetationActor.h`
- `Vehicle/CarlaWheeledVehicle.h`
- `<unordered_set>` `<vector>` `<cmath>` `<sstream>`
- `compiler/disable-ue4-macros.h`
- `carla/rpc/String.h`
- `compiler/enable-ue4-macros.h`

## 4. 宏定义
组件内定义了多个用于日志控制的宏：

| 宏定义 | 说明 | 所在行 |
|--------|------|--------|
| `ACC_LOG(...)` | 累积日志 | 44 |
| `ACCUMULATIONLOGS` | 控制 `ACC_LOG` 是否启用（0 关闭） | 27 |
| `COLLISION_LOG(...)` | 碰撞日志 | 39 |
| `COLLISIONLOGS` | 控制 `COLLISION_LOG` 是否启用 | 26 |
| `FICT_LOG(...)` | 虚拟力日志 | 49 |
| `FICTITIOUSFORCELOGS` | 控制 `FICT_LOG` 是否启用 | 28 |
| `OTHER_LOG(...)` | 其他日志 | 54 |
| `OTHERLOGS` | 控制 `OTHER_LOG` 是否启用 | 29 |
| `SOLVER_LOG(...)` | 计算求解日志 | 34 |
| `SOLVERLOGS` | 控制 `SOLVER_LOG` 是否启用 | 25 |
| `SPRINGVEGETATIONLOGS` | 控制植被弹簧系统日志 | 24 |

## 5. 主要函数
### 5.1 数值计算与转换
| 函数 | 说明 | 所在行 |
|------|------|--------|
| `EigenToFString<T>(T &t)` | 将 Eigen 矩阵转换为 FString | 89 |
| `EigenVectorToRotator(const Eigen::Vector3d &Vector)` | 将 Eigen 3D 向量转换为 FRotator | 134 |
| `GetDeltaRotator(const FRotator &Rotator1, const FRotator &Rotator2)` | 计算两个旋转器的角度差 | 58 |
| `GetSign<T>(T n)` | 获取数值的符号 | 84 |
| `OuterProduct(const Eigen::Vector3d &V1)` | 计算向量外积（单输入版本） | 99 |
| `OuterProduct(const Eigen::Vector3d &V1, const Eigen::Vector3d &V2)` | 计算向量外积（双输入版本） | 95 |
| `RotatorToEigenVector(const FRotator &Rotator)` | 将 FRotator 转换为 Eigen 3D 向量 | 127 |
| `ToEigenMatrix(const FMatrix &Matrix)` | 将 Unreal FMatrix 转换为 Eigen 矩阵 | 112 |
| `ToEigenMatrix(const FTransform &Transform)` | 将 Unreal FTransform 转换为 Eigen 矩阵 | 121 |
| `ToEigenVector(const FVector &V1)` | 将 FVector 转换为 Eigen 3D 向量 | 104 |
| `ToUnrealVector(const Eigen::Vector3d &V1)` | 将 Eigen 3D 向量转换为 FVector | 108 |

