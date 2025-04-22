# Settings
---
# 一.CarlaSettings.CPP内容
## 1、核心架构 - 系统配置子系统
### 1.1 初始化构建模块
- **代码示例**： 
```cpp
const FName UCarlaSettings::CARLA_ROAD_TAG = FName("CARLA_ROAD");
const FName UCarlaSettings::CARLA_SKY_TAG = FName("CARLA_SKY");
```
### 1.2 质量登记转换器
- **功能**：实现将字符串转枚举函数和枚举转字符串函数的功能，以便支持用户定制化配置和系统状态反馈。
- **代码示例**： 
```cpp
//字符串转枚举函数
// 将配置字符串转换为质量等级枚举（对应用户定制能力）
static EQualityLevel QualityLevelFromString(
    const FString& QualitySettingsLevel, 
    EQualityLevel Default = EQualityLevel::Epic) 
{
    if (QualitySettingsLevel.Equals("Low")) 
        return EQualityLevel::Low;    // 低画质模式
    if (QualitySettingsLevel.Equals("Epic")) 
        return EQualityLevel::Epic;   // 高画质模式
    return Default;                  // 默认画质
}

//②枚举转字符串函数
// 将质量等级枚举转换为可读字符串（用于状态反馈）
FString QualityLevelToString(EQualityLevel QualitySettingsLevel) 
{
    const UEnum* ptr = FindObject<UEnum>(
        ANY_PACKAGE, 
        TEXT("EQualityLevel"), 
        true
    );
    return ptr ? 
        ptr->GetNameStringByIndex(static_cast<int32>(QualitySettingsLevel)) :
        FString("Invalid");  // 容错处理
}
```
## 2、使用实践 - 配置管理
### 2.1 文件配置加载流
- **功能**：文件配置加载流实现多级配置管理，它能够智能处理相对/绝对路径转换，通过内存映射高效加载INI配置文件至系统，并记录当前配置状态。支持命令行参数>配置文件>默认值的优先级覆盖策略，确保动态配置更新的同时维护运行可追溯性。
- **代码示例**： 
```cpp
// INI文件加载路径处理
static bool GetSettingsFilePathFromCommandLine(FString &Value) {
    if (FParse::Value(FCommandLine::Get(), TEXT("-carla-settings="), Value)) {
        if (FPaths::IsRelative(Value)) {
            // 路径转换核心逻辑
            Value = FPaths::ConvertRelativePathToFull(FPaths::LaunchDir(), Value);
        }
        return true;
    }
    return false;
}

// 从文件加载配置
void UCarlaSettings::LoadSettingsFromFile(const FString &FilePath, bool bLogOnFailure) {
    UE_LOG(LogCarla, Log, TEXT("Loading CARLA settings from \"%s\""), *FilePath);
    FIniFile ConfigFile(FilePath);  // 内存地图实例化
    LoadSettingsFromConfig(ConfigFile, *this, true);  // 全量加载
    CurrentINIFileName = FilePath;  // 状态维护
}
```
### 2.2 命令行覆盖机制
- **功能**：命令行覆盖机制实现最高优先级参数设置，动态分配端口链（RPC+1/2）并实时切换质量等级，确保用户定制化配置覆盖文件和默认值。
- **代码示例**： 
```cpp
// 参数优先级控制（体现车辆注册机制）
void UCarlaSettings::ApplyCommandLineSettings() {
    uint32 Value;
    // 端口动态计算（RPC+1/2策略）
    if (FParse::Value(FCommandLine::Get(), TEXT("-world-port="), Value)) {
        RPCPort = Value;          // RPC端口绑定
        StreamingPort = Value + 1u;  // 流媒体端口
        SecondaryPort = Value + 2u;  // 辅助端口
    }

    // 质量等级实时切换（用户定制能力）
    FString StringQualityLevel;
    if (FParse::Value(FCommandLine::Get(), TEXT("-quality-level="), StringQualityLevel)) {
        QualityLevel = QualityLevelFromString(StringQualityLevel, EQualityLevel::Epic);
    }
}
```
- **核心功能流程图**：
 ```mermaid
    A[启动参数解析] --> B{含-carla-settings参数?}
    B -->|是| C[路径转换处理]
    B -->|否| D[使用默认配置]
    C --> E[加载INI配置文件]
    E --> F[应用命令行覆盖]
    F --> G[生成最终配置]
```
- **流程图说明**：
```mermaid
​​节点类型​​：
矩形节点：[操作步骤]
菱形节点：{条件判断}
箭头方向：--> 表示流程走向
```
```mermaid
​​逻辑分支​​：
条件判断结果为"是"时走路径转换流程
条件判断结果为"否"时直接使用默认配置
```
```mermaid
​​流程序列​​：
启动参数 → 配置文件检测 → 路径处理/默认配置 → 文件加载 → 参数覆盖 → 最终配置生成
```
## 3.高级功能 - 运行模式
- **功能**：从配置文件读取同步模式开关和渲染引擎禁用状态，实现仿真步调协同控制与渲染资源动态管理，优化系统运行性能。
### 3.1同步模式设置
- **功能**：协调仿真步调控制，确保系统状态一致性
- **代码示例**： 
```cpp
// 同步模式设置（协同机制实现）
ConfigFile.GetBool(S_CARLA_SERVER, TEXT("SynchronousMode"), Settings.bSynchronousMode);
```
### 3.2渲染开关
- **功能**：禁用渲染引擎降负载，优化资源提升性能
- **代码示例**： 
```cpp
ConfigFile.GetBool(S_CARLA_SERVER, TEXT("DisableRendering"), Settings.bDisableRendering);
```
## 4.系统维护 - 诊断机制
- **功能**：实时记录配置文件名、网络端口、同步模式、渲染状态及质量等级等关键参数，提供运行时状态快照，支持系统诊断与异常监控。
- **核心参数**： 
```cpp
UE_LOG(LogCarla, Log, TEXT(">> CARLA Settings @ %s"), *CurrentINIFileName);     // 配置源追踪
UE_LOG(LogCarla, Log, TEXT("[Network] RPCPort=%d"), RPCPort);                  // 网络端口监控
UE_LOG(LogCarla, Log, TEXT("[Control] SynchronousMode=%s"),                    // 同步模式状态
EnabledDisabled(bSynchronousMode));
UE_LOG(LogCarla, Log, TEXT("[Render] DisableRendering=%s"),                    // 渲染引擎状态
EnabledDisabled(bDisableRendering));
UE_LOG(LogCarla, Log, TEXT("[Quality] Level=%s"),                              // 画质等级反馈
*QualityLevelToString(QualityLevel));
```



# 二.CarlaSettings.h内容
## 1.关键依赖模块
- **功能**：实现质量等级动态调整机制，支持蓝图系统调用。
- **核心功能流程图**：
 ```mermaid
    A[CarlaSettings.h] --> B[Engine/StaticMesh.h]
    A --> C[QualityLevelUE.h]
    A --> D[CarlaSettings.generated.h]
    B --> E[Engine/StaticMeshActor.h]
    C --> F[CoreUObject.h]
```
- **关键依赖关系说明**：
  - `静态网格依赖链`：
    - ` `：通过StaticMesh.h实现对场景物体的精细控制
    - ` `：CarlaSettings -> StaticMesh -> StaticMeshActor
  - `质量等级系统`：
      - ` `：QualityLevelUE.h提供标准化的质量等级定义
     - ` `：与CoreUObject的交互实现蓝图可调用接口
  - `反射系统支持`：
     - ` `：.generated.h文件确保所有UPROPERTY/UCLASS标记的属性
     - ` `：支持编辑器可见性和序列化功能

 ## 2.类定义架构
 ### 2.1 类声明规范
 - **核心类定义**： 
```cpp
UCLASS(BlueprintType, Blueprintable, config=Game, defaultconfig)
class CARLA_API UCarlaSettings : public UObject
{
    GENERATED_BODY()
    // 类成员定义...
};
```
- **关键元数据说明**：

| 元数据 | 类型 | 功能描述 | 应用示例 |
|--------|------|----------|----------|
| **BlueprintType** | 类标记 | 将C++类暴露给蓝图系统，允许在蓝图中作为变量类型使用 | `UCLASS(BlueprintType)`<br>允许在蓝图中创建`UCarlaSettings`类型变量 |
| **Blueprintable** | 类标记 | 允许基于该C++类创建蓝图子类 | `UCLASS(Blueprintable)`<br>可在编辑器右键创建基于`UCarlaSettings`的蓝图资产 |
| **config=Game** | 配置标记 | 指定配置保存的目标INI文件 | `UCLASS(config=Game)`<br>配置将保存在`Config/Game.ini`文件中 |
| **defaultconfig** | 持久化标记 | 保持编辑器中修改的默认值 | `UCLASS(defaultconfig)`<br>在编辑器中修改的默认值会持久化保存 |


  - `作用`：
    - ` `：可在蓝图中创建该类型的变量
     - ` `：支持创建蓝图子类进行扩展
- **继承关系图示**：
 ```mermaid
    UObject <|-- UCarlaSettings
    UCarlaSettings : +GENERATED_BODY()
    UCarlaSettings : +BlueprintType
    UCarlaSettings : +config=Game
```
### 2.2 质量等级管理系统
- **枚举与接口定义**： 
- **核心代码**： 
```cpp
UENUM(BlueprintType)
enum class EQualityLevel : uint8 
{
    Low    UMETA(DisplayName="Low Quality"),
    Epic   UMETA(DisplayName="Epic Quality")
};

UFUNCTION(BlueprintCallable, Category="Rendering")
void SetQualityLevel(EQualityLevel InQualityLevel);

UFUNCTION(BlueprintCallable, Category="Rendering")
EQualityLevel GetQualityLevel() const;
```
- **设计特性分析**：
  - `1.类型安全系统`：
    - ` `：使用enum class避免隐式转换
    - ` `：UE的UENUM宏提供反射支持
  - `2.蓝图集成方案`：
    - ` `：BlueprintCallable标记公开给蓝图
    - ` `：Category组织编辑器中的显示位置
- **质量等级控制流程**：
 ```mermaid
    participant BP as 蓝图系统
    participant CPP as UCarlaSettings
    participant Render as 渲染系统
    
    BP->>CPP: SetQualityLevel(Epic)
    CPP->>Render: UpdateAllMaterials()
    Render-->>CPP: 确认完成
    CPP-->>BP: 执行回调
```
## 3.核心配置参数
### 3.1网络服务配置
- **质量等级控制流程**：

| 参数 | 类型 | 默认值 | 取值范围 | 动态修改 | 说明 |
|------|------|--------|----------|----------|------|
| **RPCPort** | uint32 | 2000 | 1024-65535 | 需重启 | 远程过程调用主端口，用于核心通信 |
| **StreamingPort** | uint32 | 2001 | 自动计算 | 实时更新 | 传感器数据流端口，固定为RPCPort+1 |
| **bSynchronousMode** | bool | false | true/false | 实时切换 | 启用时严格按帧同步，用于确定性测试 |
| **PrimaryIP** | std::string | "" | IPv4格式 | 实时更新 | 主服务器地址，空表示本地主机 | 

- ** 端口计算逻辑**：
```cpp
// 自动派生端口计算
void CalculateDerivedPorts() {
    StreamingPort = RPCPort + 1;  // 固定+1偏移
    SecondaryPort = RPCPort + 2;  // 固定+2偏移
    
    // 有效性验证
    static constexpr uint32 MIN_PORT = 1024;
    static constexpr uint32 MAX_PORT = 65535;
    RPCPort = FMath::Clamp(RPCPort, MIN_PORT, MAX_PORT);
}
```
### 3.2 画质控制参数
- **材质系统配置**：
  - `道路材质配置`：
  ```cpp
   // 低画质道路材质数组
   UPROPERTY(config, EditFixedSize)
   TArray<FStaticMaterial> LowRoadMaterials;
    // 高画质道路材质数组  
    UPROPERTY(config, EditFixedSize) 
    TArray<FStaticMaterial> EpicRoadMaterials;
  ```
    - `材质切换流程`：
  ```mermaid
    participant Settings as 设置系统
    participant Render as 渲染器
    Settings->>Render: GetQualityLevel()
    alt Low质量
        Render->>Render: ApplyMaterials(LowRoadMaterials)
    else Epic质量
        Render->>Render: ApplyMaterials(EpicRoadMaterials)
    end 
  ```  
  - `渲染控制参数`：

    | 参数 | 类型 | 默认值 | 元数据约束 | 单位
    |------|------|--------|----------|----------|
    | LowStaticMeshMaxDrawDistance | float| 10000.0 | ClampMin=5000.0	 | 厘米 | 
    |LowLightFadeDistance | float | 1000.0 | - | 厘米 | 
    | bDisableRendering| bool | false | - | -| 
  - `典型配置示例`： 
    ```cpp
    [CARLA/QualitySettings]
    LowStaticMeshMaxDrawDistance=15000.0
    LowRoadMaterials=(MaterialSlotName="Road",MaterialInterface=...)
    ```
   - `参数应用流程`：
        ```mermaid
      participant Config as 配置文件
      participant System as 设置系统
      participant Render as 渲染引擎
    
      Config->>System: 加载画质参数
      System->>Render: ApplyQualitySettings()
      Render->>Render: 更新材质(Low/Epic)
      Render->>Render: 调整绘制距离
        ```
## 4.配置加载系统
### 4.1 多源加载接口
- **核心加载方法**：

| 方法 | 参数 | 返回值 | 功能描述 |
|------|------|--------|----------|
| `LoadSettings()` | 无 | void | **综合加载**<br>按优先级加载所有可用配置源 |
| `LoadSettingsFromString(const FString& INIContent)` | INI格式字符串 | void | **内存配置加载**<br>从字符串直接解析配置 |
| `LoadSettingsFromFile(const FString& FilePath, bool bLogOnFailure)` | 文件路径<br>失败是否记录日志 | void | **文件配置加载**<br>从INI文件读取配置 |

**加载优先级**：
1. 命令行参数 (`-carla-settings=`)
2. 项目配置文件 (`Config/CarlaSettings.ini`)
3. 代码默认值
    ```mermaid
    graph TD
    A[命令行参数] --> C[生效配置]
    B[INI文件] --> C
    D[默认值] --> C
    ```
### 4.2 调试支持功能
- **调试相关属性**：
- **核心代码**： 
    ```cpp
    // 当前加载的配置源文件路径
    UPROPERTY(Category="Debug", VisibleAnywhere)
    FString CurrentFileName;

    // 配置日志输出（示例格式）
    void LogSettings() const
    {
        UE_LOG(LogCarla, Log, TEXT("Current Config: %s"), *CurrentFileName);
        UE_LOG(LogCarla, Log, TEXT("RPCPort = %d"), RPCPort); 
    }
    ```
- **日志输出示例**： 
    ```cpp
    LogCarla: Current Config: D:/Project/Config/Carla.ini
    LogCarla: RPCPort = 4000
    LogCarla: QualityLevel = Epic
    ```
- **调试功能对比**：

    | 功能 | 实现方式 | 使用场景 | 
    |------|------|--------|
    | 配置源追踪 | CurrentFileName | 确认最终生效的配置文件 | 
    | 参数检查 | LogSettings() | 运行时验证参数值 | 
    | 加载失败处理 | bLogOnFailure | 配置文件缺失时记录错误|
## 5. 高级功能实现
### 5.1 ROS2集成
- **核心配置项**：
    ```cpp
    // ROS桥接功能开关
    UPROPERTY(Category="ROS2", config, EditAnywhere)
    bool bEnableROS2 = false;
    ```
- **设计架构**：
    ```mermaid
    graph LR
    A[CARLA核心] --> B{ROS2启用?}
    B -->|是| C[启动ROS桥接]
    B -->|否| D[跳过初始化]
    C --> E[建立Topic通信]
    ```
- **关键特性**：
  - `独立于画质系统的专属配置节`
  - `配置持久化到CarlaSettings.ini`
  - `运行时动态开关ROS服务`
### 5.2  场景标记系统
- **标签常量定义**：
    ```cpp
   // 道路对象标签
   static const FName CARLA_ROAD_TAG = TEXT("CARLA_ROAD");

  // 天空球体标签  
  static const FName CARLA_SKY_TAG = TEXT("CARLA_SKY");
    ```
- **应用场景**：
  - `场景对象分类检索`
  - `批量处理特定类型Actor`
  - `运避免硬编码字符串风险`
## 6. 属性元数据系统
### 6.1 属性定义规范
- **典型示例**：
    ```cpp
    UPROPERTY(
    Category="QualitySettings/Low",
    BlueprintReadOnly,
    EditAnywhere,
    config,
    meta=(ClampMin="5000.0", UIMin="5000.0")
   )
   float LowRoadPieceMeshMaxDrawDistance = 10000.0f;
    ```
### 6.2 元数据功能矩阵
| 
元数据 | 作用域 | 参数格式 | 功能说明 |
|------|------|--------|----------|
| config | Property | - | 保存到INI配置文件 |
| BlueprintReadOnly	 | Property	 |  - |  蓝图可见但不可写 |
| ClampMin| Meta | "5000.0" | 数值最小值约束 |
| EditCondition| Meta | "bEnableFeature"| 属性编辑条件控制
 |
## 7. 扩展设计接口
### 7.1 材质配置系统
- **数据结构**：
    ```cpp
    struct FQualityMaterialPreset {
    TMap<FName, UMaterialInterface*> SlotMaterials;
    float MaxDrawDistance;
    };

    TArray<FQualityMaterialPreset> RoadMaterialPresets;
    ```
- **特性**：
  - `支持按材质插槽名称匹配`
  - `多质量等级预设配置`
  - `自动材质切换机制`
### 7.2 光照控制系统
- **光照控制参数表**：

| 参数名称 | 数据类型 | 默认值 | 单位/范围 | 功能描述 |
|----------|----------|--------|-----------|----------|
| **LowLightFadeDistance** | `float` | 1000.0 | 厘米(cm) | 控制光照效果的渐隐距离，超过此距离的光照效果会逐渐减弱 |
| **bCastDynamicShadows** | `bool` | `false` | true/false | 是否启用动态阴影投射功能，开启后动态物体会产生实时阴影 |
| **ShadowResolution** | `int32` | 512 | 像素(pixels) | 定义阴影贴图的分辨率，数值越大阴影质量越高但性能消耗越大 |
3. ShadowResolution
    - `1. LowLightFadeDistance`
      - `代码示例`：
          ```cpp
         struct FQualityMaterialPreset {
        TMap<FName, UMaterialInterface*> SlotMaterials;
        float MaxDrawDistance;
        };

        TArray<FQualityMaterialPreset> RoadMaterialPresets;
        ```
    - `2. bCastDynamicShadows`
    - `3. ShadowResolution`
- **优化策略：**：
    ```mermaid
        A[质量等级] --> B{Low?}
        B -->|是| C[减少阴影计算]
        B -->|否| D[全特效渲染]
    ```
# 三.CarlaSettingsDelegate.cpp内容
## 1.核心架构
### 1.1 类定义与初始化























# 四.CarlaSettingsDelegate.h内容