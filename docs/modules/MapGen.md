函数说明
1. GetBaseMeshTag
cpp
复制
ECityMapMeshTag CityMapMeshTag::GetBaseMeshTag()
功能
获取基础道路网格标签类型

返回值
ECityMapMeshTag::RoadTwoLanes_LaneLeft - 表示默认的双车道左车道道路类型

典型用途
初始化道路生成系统时作为默认道路配置

道路网络生成的基准参考类型

2. GetRoadIntersectionSize
cpp
复制
uint32 CityMapMeshTag::GetRoadIntersectionSize()
功能
获取道路交叉口的基准尺寸

返回值
5u (uint32类型) - 表示交叉口的边长尺寸（单位需根据项目约定，通常为米）

注意
该值可能对应交叉路口地面网格的边长参数

用于生成标准十字路口的尺寸计算

3. ToString
cpp
复制
FString CityMapMeshTag::ToString(ECityMapMeshTag Tag)
功能
将枚举标签转换为可读字符串

参数
Tag (ECityMapMeshTag) - 需要转换的道路网格枚举标签

返回值
对应枚举值的字符串名称

若枚举类型未找到则返回 "Invalid"

实现细节
使用 FindObject 在内存中查找 ECityMapMeshTag 枚举类型

通过虚幻引擎的反射系统获取枚举值的字符串表示

包含错误处理机制，确保非法枚举值的容错性

典型用途
调试时输出可读的标签信息

日志记录时保存枚举类型信息

可视化界面显示标签类型

使用示例
cpp
复制
// 获取基础道路标签
ECityMapMeshTag BaseTag = CityMapMeshTag::GetBaseMeshTag();

// 获取交叉口尺寸参数
uint32 IntersectionSize = CityMapMeshTag::GetRoadIntersectionSize();

// 枚举值转字符串
FString TagName = CityMapMeshTag::ToString(ECityMapMeshTag::RoadFourLanes_LaneLeft);
// 输出结果："RoadFourLanes_LaneLeft"