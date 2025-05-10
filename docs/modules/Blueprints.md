1. 概述
UMapGenFunctionLibrary 是用于 Carla 地图生成的核心工具类，提供网格构建、资源管理、坐标投影等功能。
主要功能：

将自定义网格数据转换为 Unreal Engine 的 FMeshDescription。

创建并保存静态网格资源（UStaticMesh）。

执行墨卡托投影转换（地理坐标→引擎坐标）。

辅助渲染命令与资源清理。

2. 核心函数说明
2.1 BuildMeshDescriptionFromData
功能：将自定义网格数据转换为 FMeshDescription，包含顶点、三角形、法线、UV 等信息。
参数：

Data (FProceduralCustomMesh): 输入网格数据（顶点、三角形、UV 等）。

ParamTangents (TArray<FProcMeshTangent>): 顶点切线数据。

MaterialInstance (UMaterialInstance*): 材质实例。

返回值：

FMeshDescription: 构建的网格描述对象，用于后续生成静态网格。

注意事项：

若 ParamTangents 数量与顶点数不匹配，切线数据将无法正确设置。

UV 通道默认填充为 (0,0)，需确保输入 UV 数据正确。

2.2 CreateMesh
功能：基于 FMeshDescription 创建并保存静态网格资源（.uasset）。
参数：

Data (FProceduralCustomMesh): 输入网格数据。

ParamTangents (TArray<FProcMeshTangent>): 顶点切线数据。

MaterialInstance (UMaterialInstance*): 材质实例。

MapName (FString): 地图名称，用于资源路径。

FolderName (FString): 资源存储文件夹。

MeshName (FName): 网格资源名称。

返回值：

UStaticMesh*: 生成的静态网格资源指针，失败返回 nullptr。

关键流程：

构建资源包路径（如 /Game/CustomMaps/MapName/Static/FolderName/MeshName）。

调用 BuildMeshDescriptionFromData 生成网格描述。

创建 UStaticMesh 对象并设置材质、碰撞体等属性。

保存资源并通知资源管理器。

注意事项：

目录创建逻辑被注释（PlatformFile.CreateDirectory），需手动启用。

需确保材质实例有效，否则记录错误日志。

2.3 GetTransversemercProjection
功能：将经纬度坐标转换为引擎内的平面墨卡托投影坐标（单位：厘米）。
公式：

参数：

lat / lon (float): 目标点经纬度（度）。

lat0 / lon0 (float): 参考原点经纬度（度）。

返回值：

FVector2D: 投影后的平面坐标（X: 横向，Y: 纵向，已转换为厘米）。