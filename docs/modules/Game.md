UCarlaStatics::GetAllMapNames 函数说明文档
函数功能
UCarlaStatics::GetAllMapNames 函数用于获取项目中所有符合条件的地图资源名称列表。该函数通过递归扫描项目内容目录下的 .umap 文件，经过路径处理和过滤后，返回符合 Unreal 引擎资源引用规范的标准化地图名称列表。

函数原型
cpp
static TArray<FString> UCarlaStatics::GetAllMapNames();
返回值

TArray<FString>：处理后的地图名称列表，格式为 /Game/Path/To/MapName。

实现流程
1. 文件查找
目标文件类型：.umap 文件（Unreal 地图文件）。

查找目录：项目内容目录（FPaths::ProjectContentDir()）。

递归查找：启用递归搜索，包含子目录。

过滤条件：排除隐藏文件及目录。

cpp
IFileManager::Get().FindFilesRecursive(
    MapNameList, 
    *FPaths::ProjectContentDir(), 
    TEXT("*.umap"), 
    true,   // 递归查找
    false,  // 不包含目录
    false); // 不包含隐藏文件
2. 过滤规则
移除路径中包含 TestMaps 的条目（测试地图）。

移除路径中包含 OpenDriveMap 的条目（OpenDrive 相关地图）。

cpp
MapNameList.RemoveAll([](const FString &Name) { 
    return Name.Contains("TestMaps"); 
});
MapNameList.RemoveAll([](const FString &Name) { 
    return Name.Contains("OpenDriveMap"); 
});
3. 路径处理
对每个文件路径进行以下处理：

分割路径：以 Content/ 为分隔符，提取后半部分。

示例输入：C:/Project/Content/Maps/Town01.umap

分割结果：TmpStrList[1] = "Maps/Town01.umap"

移除扩展名：替换 .umap 后缀为空字符串。

标准化路径：添加 /Game/ 前缀，生成 Unreal 资源路径。

cpp
// 示例处理流程
MapNameList[i] = "C:/Project/Content/Maps/Town01.umap";
MapNameList[i].ParseIntoArray(TmpStrList, TEXT("Content/"), true);
// TmpStrList[1] = "Maps/Town01.umap"
MapNameList[i] = TmpStrList[1].Replace(TEXT(".umap"), TEXT(""));
// 结果: "Maps/Town01"
MapNameList[i] = "/Game/" + MapNameList[i];
// 最终结果: "/Game/Maps/Town02"