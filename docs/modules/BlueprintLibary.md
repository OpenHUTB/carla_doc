# CARLA BlueprintLibary 模块技术文档 (MapGenFunctionLibrary)

---

## 目录  
1. [模块概述](#模块概述)  
2. [核心功能](#核心功能)  
3. [类与方法详解](#类与方法详解)  
4. [数据结构与配置](#数据结构与配置)  
5. [关键流程](#关键流程)  
6. [配置参数](#配置参数)  
7. [注意事项](#注意事项)  
8. [示例配置](#示例配置)  
9. [附录](#附录)  

---

## 模块概述  
`MapGenFunctionLibrary` 是 CARLA 中用于地图生成和几何处理的蓝图函数库，提供以下核心能力：  
- **静态网格生成**：从程序化数据创建符合 UE 规范的静态网格资产  
- **坐标投影转换**：实现横向墨卡托投影，支持 OSM 坐标到 UE 坐标系的转换  
- **异步控制**：提供线程休眠与渲染命令刷新接口  
- **资源管理**：自动化资产创建/保存流程及引擎资源清理  

---

## 核心功能  
### 1. 静态网格构建  
- **几何数据转换**  
  ```cpp
  FMeshDescription BuildMeshDescriptionFromData(...);
  ```
  - 将顶点/三角形数据转换为 UE 的 FMeshDescription 结构  
  - 支持法线、切线、UV 等多通道属性配置  

- **资产创建与保存**  
  ```cpp
  static UStaticMesh* CreateMesh(...);
  ```
  - 自动生成 StaticMesh 资产并保存到指定包路径  
  - 内置碰撞体生成（CTF_UseComplexAsSimple）  

### 2. 地理坐标转换  
- **横向墨卡托投影**  
  ```cpp
  static FVector2D GetTransversemercProjection(...);
  ```
  - 实现经纬度到平面坐标的精确转换  
  - 支持自定义参考原点（lat0, lon0）  
  - 自动应用 OSM 到厘米的缩放因子（100x）  

### 3. 系统级控制  
- **渲染管线同步**  
  ```cpp
  static void FlushRenderingCommandsInBlueprint();
  ```
  - 强制刷新渲染线程命令队列  
  - 确保资源修改实时生效  
```mermaid
   sequenceDiagram
  participant GameThread
  participant RenderThread
  GameThread->>RenderThread: 发送渲染命令
  RenderThread->>GameThread: 确认命令接收
  GameThread->>RenderThread: 执行Flush操作
  RenderThread->>GameThread: 返回完成状态
   ```
- **内存管理**  
  ```cpp
  static void CleanupGEngine();
  ```
  - 执行全量垃圾回收  
  - 清理未引用资产及 Actor 实例  
 
---

## 类与方法详解  
### `UMapGenFunctionLibrary` 类  
继承自 `UBlueprintFunctionLibrary`，提供地图生成相关的静态工具方法，所有方法均支持蓝图调用。
### 类结构图
 ```mermaid
classDiagram
  class UMapGenFunctionLibrary{
    +CreateMesh() UStaticMesh*
    +BuildMeshDescriptionFromData() FMeshDescription
    +GetTransversemercProjection() FVector2D
    +FlushRenderingCommandsInBlueprint()
    +CleanupGEngine()
  }
   ```
#### 关键方法  
| 方法                          | 功能描述                                                                 |
|-------------------------------|-------------------------------------------------------------------------|
| `CreateMesh()`                | 入口方法，创建并保存静态网格资产，返回 UStaticMesh 指针                |
| `BuildMeshDescriptionFromData()` | 核心几何处理，将程序化网格数据转换为 UE 网格描述结构                   |
| `GetTransversemercProjection()` | 地理坐标转换，返回投影后的 2D 坐标（单位：厘米）                       |
| `FlushRenderingCommandsInBlueprint()` | 强制同步渲染线程，确保图形资源更新                                   |
| `CleanupGEngine()`            | 执行全局资源清理，释放内存                                           |

#### 参数详解（CreateMesh）  
| 参数              | 类型                      | 必填 | 描述                                                                 |
|-------------------|---------------------------|------|----------------------------------------------------------------------|
| Data              | FProceduralCustomMesh     | 是   | 包含顶点、三角形等几何数据的结构体                                   |
| ParamTangents     | TArray<FProcMeshTangent>  | 否   | 顶点切线数组，为空时使用默认计算                                     |
| MaterialInstance  | UMaterialInstance*        | 否   | 主材质实例，应用于生成网格的所有表面                                 |
| MapName           | FString                   | 是   | 目标地图名称，用于构建资产存储路径                                   |
| FolderName        | FString                   | 是   | 存储目录名称，通常表示资产分类（如 Roads、Buildings）               |
| MeshName          | FName                     | 是   | 生成的静态网格资产名称，需符合 UE 命名规范                           |

---

## 数据结构与配置  
### 1. FProceduralCustomMesh 结构体  
定义程序化生成的网格数据：

| 字段          | 类型                  | 描述                             |
|---------------|-----------------------|----------------------------------|
| Vertices      | TArray<FVector>       | 顶点坐标数组（UE 坐标系，单位：厘米） |
| Triangles     | TArray<int32>         | 三角形索引数组（按顺时针顺序）     |
| Normals       | TArray<FVector>       | 顶点法线向量数组                  |
| UV0           | TArray<FVector2D>     | 基础 UV 通道坐标                  |
| VertexInstances| TArray<FVector>       | 顶点实例数据（预留扩展）          |

### 2. 全局常量  
```cpp
static const float OSMToCentimetersScaleFactor = 100.0f;
```
- OSM 单位到厘米的转换系数（1 OSM单位 = 100厘米）  
- 应用于 `GetTransversemercProjection()` 输出坐标  

---

## 关键流程  
### 静态网格生成流程  
1. **数据准备**  
   - 填充 FProceduralCustomMesh 结构体  
   - 准备材质实例（需提前加载到内存）  

2. **网格描述构建**  
   ```mermaid
   graph LR
     A[顶点数据] --> B(创建VertexID)
     C[三角形索引] --> D(生成VertexInstance)
     D --> E[设置法线/UV/切线]
     E --> F(构建Polygon)
     F --> G[生成FMeshDescription]
   ```

3. **资产序列化**  
   - 自动创建 UPackage 并保存至 `/Game/CustomMaps/[MapName]/Static/[FolderName]/`  
   - 资产命名规范：`MeshName` 参数指定  

### 坐标转换算法  
横向墨卡托投影公式：  
```
x = R * asinh(sin(Δλ) / sqrt(tan²φ + cos²Δλ))  
y = R * atan(tanφ / cosΔλ)  
Result = (x, y_ref - y) * OSMToCentimetersScaleFactor  
```
其中：  
- φ, λ：目标点的纬度/经度（弧度）  
- φ0, λ0：参考原点纬度/经度（弧度）  
- R：地球半径（6373000 米）  

---

## 配置参数  
### CreateMesh 参数约束  
| 参数              | 有效范围              | 默认值    |
|-------------------|-----------------------|-----------|
| Data.Vertices.Num() | ≥ 3                  | 无        |
| Data.Triangles.Num() | 必须为 3 的倍数      | 无        |
| MaterialInstance  | 必须为已加载的材质    | nullptr   |
| MeshName          | 符合 UE 命名规则      | 无        |

### 投影参数预设  
| 参数     | 建议值          | 说明                     |
|----------|-----------------|--------------------------|
| lat0/lon0| 地图中心点坐标  | 通常取整个地图区域的中心 |
| R        | 6373000.0       | 地球平均半径，不可修改   |

---

## 注意事项  
1. **线程安全**  
   - `SetThreadToSleep` 会阻塞当前线程，避免在游戏线程长时间休眠  
   - `FlushRenderingCommandsInBlueprint` 需在主线程调用  

2. **性能优化**  
   - 单次 CreateMesh 调用顶点数建议 ≤ 65535（UE 顶点缓冲区限制）  
   - 复杂网格建议分批次生成  

3. **坐标精度**  
   - 墨卡托投影在低纬度地区精度较高，高纬度地区建议缩小地图区块  

4. **材质应用**  
   - 如未指定 MaterialInstance，网格将使用默认材质（粉色警告）  

---

## 示例配置  
### 创建道路网格  
```cpp
// 准备几何数据
FProceduralCustomMesh RoadData;
RoadData.Vertices = { FVector(0,0,0), FVector(100,0,0), FVector(50,50,0) };
RoadData.Triangles = { 0, 1, 2 };
RoadData.Normals = { FVector::UpVector, FVector::UpVector, FVector::UpVector };

// 调用生成函数
UStaticMesh* RoadMesh = UMapGenFunctionLibrary::CreateMesh(
    RoadData, 
    TArray<FProcMeshTangent>(), 
    LoadObject<UMaterialInstance>(".../Asphalt_Inst"),
    "SanFrancisco",
    "Roads",
    FName("MainRoad_Section1")
);
```

---

## 附录  
### 依赖项  
- **Unreal Engine 模块**  
  - StaticMeshDescription（网格数据处理）  
  - AssetRegistry（资产管理与保存）  
  - RenderCore（渲染命令控制）  

- **CARLA 组件**  
  - ProceduralCustomMesh 数据结构  
  - CarlaMapGenerator 坐标系规范  

### 已知问题与解决方案  
| 现象                    | 原因                | 解决方案                      |
|-------------------------|---------------------|-------------------------------|
| 网格显示为不可见        | 未设置碰撞体        | 检查 bBuildSimpleCollision 参数 |
| 投影坐标偏移            | 参考原点设置错误    | 验证 lat0/lon0 是否为地图中心  |
| 材质 UV 拉伸            | UV0 数据未正确提供  | 检查 Data.UV0 数组与顶点对应性 |

### 调试支持  
- **日志追踪**  
  ```cpp
  UE_LOG(LogCarlaMapGenFunctionLibrary, Warning, TEXT("MaterialInstance is nullptr"));
  ```
- **实时预览**  
  - 在编辑器中运行 `CleanupGEngine()` 可刷新资产状态  
### 坐标转换公式图示
       ┌───────────────────────┐
       │ 横向墨卡托投影公式     │
       │ x = R・asinh(sinΔλ/√(tan²φ + cos²Δλ)) │
       │ y = R・atan(tanφ/cosΔλ)               │
       └───────────────────────┘   
### 性能优化建议图表
 ```mermaid
   graph LR
  A[高顶点数] --> B(分块处理)
  B --> C{每块≤65535顶点}
  C -->|是| D[并行生成]
  C -->|否| B
   ```
---
