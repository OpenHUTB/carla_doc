# AI
# 目录
- [1. **AIControllerFactory 类**](#1-aicontrollerfactory-类)  
  - [概述](#概述)  
  - [成员函数](#成员函数)  
    - [GetDefinitions](#getdefinitions)  
    - [SpawnActor](#spawnactor)  
  - [错误处理体系](#错误处理体系)  
  - [继承关系](#继承关系)  
- [2. **WalkerAIController 类**](#2-walkeraicontroller-类)  
  - [定义与职责](#定义与职责)  
  - [构造函数](#构造函数)  
  - [成员变量与属性](#成员变量与属性)  
  - [使用场景](#使用场景)  
- [3. **Unreal Engine 集成机制**](#3-unreal-engine-集成机制)  
  - [反射系统集成](#反射系统集成)  
  - [Actor生成机制](#actor生成机制)  
  - [组件管理](#组件管理)  
- [4. 代码实例与用法](#4-代码实例与用法)  
  - [创建 AI 控制器](#创建-ai-控制器)  
  - [配置 Walker AI](#配置-walker-ai)  
  - [错误调试](#错误调试)  
- [5. 相关文件与依赖](#5-相关文件与依赖)  
  - [头文件依赖树](#头文件依赖树)  
  - [依赖的引擎模块](#依赖的引擎模块)  
  - [Buildcs配置示例](#buildcs配置示例)  
---

## 1. **AIControllerFactory 类**  
### 概述  
`AIControllerFactory`是CARLA仿真平台中实现AI控制器动态生成的核心工厂类，继承自`ACarlaActorFactory`。其职责包括：  
- 定义支持的AI控制器类型（如行人、车辆等）  
- 通过Unreal Engine的反射系统实现类型注册  
- 提供标准化的Actor生成接口与错误处理机制 

### 成员函数  
#### `GetDefinitions`  
- **功能**：返回工厂支持的AI控制器定义集合，用于引擎的类型发现与蓝图集成。
- **代码示例**：  
  ```cpp
  TArray<FactorDefinitions> AAIControllerFactory::GetDefinitions() {
      // 使用工具类创建Walker控制器定义
      auto WalkerController = ABFL::MakeGenericDefinition(
          TEXT("controller"), TEXT("ai"), TEXT("walker")
      );
      // 绑定具体类对象
      WalkerController.Class = WalkerAIController::StaticClass();
      return { WalkerController };
  }
  ```  
- **参数说明**：
  - `TEXT("controller")`：控制器类型ID，需全局唯一。
  - `TEXT("ai")`：用于编辑器分类的元数据。
  - `TEXT("walker")`：语义标签，支持场景语义查询。

#### `SpawnActor`
- **功能**：根据描述和位置生成具体的AI控制器实例。  
- **关键参数**：  
  - `FTransform`：指定生成位置和旋转。  
  - `FactorDescription`：Actor类型和配置。  
- **方法签名**： 
  ```cpp
  FActorSpawnResult SpawnActor(
    const FTransform& SpawnAtTransform,
    const FActorDescription& Description,
    FActorSpawnParameters SpawnParameters = FActorSpawnParameters()
  );
  ```
- **关键流程**：
  - 世界上下文验证：
  ```cpp
  UWorld* World = GetWorld();
  if (!World || !World->IsGameWorld()) {
    UE_LOG(LogCarla, Fatal, TEXT("Invalid world context for AI controller spawning"));
    return FActorSpawnResult(ENullPointerError);
  }
  ```
  - 碰撞处理：强制忽略碰撞生成Actor（`ESpawnActorCollisionHandleMethod::AlwaysSpawn`）。 
  ```cpp
  SpawnParameters.SpawnCollisionHandlingMethod = 
    ESpawnActorCollisionHandlingMethod::AlwaysSpawn;  // 忽略物理碰撞
  }
  ```
  - 实例化与错误捕获：
   ```cpp
  AActor* SpawnedActor = World->SpawnActor<AActor>(
    Description.Class, 
    SpawnAtTransform, 
    SpawnParameters
  );
  if (!SpawnedActor) {
    UE_LOG(LogCarla, Error, TEXT("Failed to spawn AI controller of type %s"), 
        *Description.Id);
    return FActorSpawnResult(ESpawnActorErrorCode::UnknownError);
  }
  ```

### 错误处理体系

| 错误类型               | 触发条件                          | 处理策略                             | 错误码       |
|--------------------|-----------------------------------|--------------------------------------|--------------|
| **ENullWorldContext**    | 世界对象未初始化或无效            | 终止生成流程，返回错误码             | `0x8001`     |
| **EClassNotRegistered**  | 控制器类未注册到反射系统          | 检查 `UCLASS()` 宏与编译依赖         | —            |
| **ESpawnCollisionBlock** | 碰撞处理策略未覆盖物理阻挡        | 强制设置为 `AlwaysSpawn` 模式        | —            |

#### 说明
- **ENullWorldContext**  
   - 触发时立即终止生成流程，避免无效操作占用资源。  
   - 错误码 `0x8001` 用于快速定位问题场景。  

- **EClassNotRegistered**  
   - 需验证反射系统配置（如 `UCLASS()` 宏是否遗漏）。  
   - 检查编译依赖链是否包含目标类的头文件。  

- **ESpawnCollisionBlock**  
   - 默认碰撞策略（如 `AdjustIfPossible`）可能无法覆盖复杂物理环境。  
   - 强制使用 `AlwaysSpawn` 模式可确保生成流程完成，但可能引发物理重叠。  
#### 代码实现
- 以空世界检查为例：  
  ```cpp
  if (World == nullptr) {
     E_LOG(logCarla, Error, TEXT("AAIControllerFactory: cannot spawn controller into an empty world."));
     return {};
  }
  ```

### 继承关系  
- **父类**  ：ACarlaActorFactory（CARLA仿真平台的Actor工厂基类）。
- **子类**  ：通过重写 GetDefinitions 和 SpawnActor 实现定制化AI控制器生成。

---

## 2. **WalkerAIController 类**  
### 定义与职责  
- **继承关系**  ：继承自 AActor，作为行人AI控制器的轻量级句柄。
  ```cpp
  classDiagram
    AActor <|-- AWalkerAIController
    class AActor {
        +RootComponent: USceneComponent*
        +BeginPlay()
        +Tick(float DeltaSeconds)
    }
    class AWalkerAIController {
        -bIsActive: bool
        +SetMovementTarget(FVector Target)
        +OnNavigationComplete()
    }
  ``` 
- **核心作用** 
  - 轻量级代理：作为服务端与客户端控制逻辑的中介，避免直接操作物理实体。
  - 状态同步：通过RPC（Remote Procedure Call）实现跨网络的状态同步。
  - 生命周期管理：处理控制器的激活/休眠状态切换。

### 构造函数  
- **初始化设置**  
  - 禁止Actor的Tick功能：`PrimaryActorTick.bCanEverTick = false`。  
  - 隐藏根组件：`RootComponent->bHiddenInGame = true`。  

- **代码实现**  
  ```cpp
  AWalkerAIController(const FObjectInitializer &ObjectInitializer)
      : Super(ObjectInitializer) {
      PrimaryActorTick.bCanEverTick = false;
      RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
      RootComponent->bHiddenInGame = true;
  }
  ``` 
### 成员变量与属性
- **PrimaryActorTick**  ：设置为 false，禁止帧更新以优化性能。
- **RootComponent**  ：默认场景组件，用于管理Actor的变换和渲染，游戏运行时不可见。

### 使用场景
- **行人模拟**  ：在CARLA仿真中，通过此类控制行人的基础属性（如位置、状态）。
- **多客户端协作**  ：作为服务端与客户端间的代理，传递控制指令。

---

## 3. **Unreal Engine 集成机制**
### 反射系统集成
- **UCLASS宏扩展**  ：UCLASS宏用于在Unreal Engine中注册C++类到反射系统，使得这些类可以被蓝图访问和继承，以及支持序列化、垃圾回收等功能。
  - 示例：
    ```cpp
    UCLASS(
      Blueprintable,                     // 允许蓝图继承
      ClassGroup = (Custom),             // 自定义编辑器分类
      meta = (DisplayName = "AI Controller Factory")  // 编辑器显示名称
    )
    class CARLA_API AAIControllerFactory : public ACarlaActorFactory {
      GENERATED_BODY()
      // ...
    };
    ```  
- **属性暴露到编辑器**  
  - 示例：
    ```cpp
    UPROPERTY(
      EditDefaultsOnly,                  // 仅在默认值编辑器中可修改
      Category = "AI Controller",        // 分类标签
      meta = (Tooltip = "Controller类型ID")  // 悬浮提示
    )
    FString ControllerTypeId = TEXT("default");      
    ```   
  
### Actor生成机制
- **生成流程**  ：通过 World->SpawnActor，传入类名、变换参数和碰撞处理模式。
- **方法生成实例**  ：
  ```cpp
  auto *Controller = World->SpawnActor<AActor>(Description.Class, Transform, SpawnParameters);
  ``` 
- **SpawnActor内部流程**  
  - 内存分配：通过FMemory::Malloc在引擎内存池中分配对象空间。
  - 组件初始化：调用InitializeComponent()递归初始化子组件。
  - 注册到世界：将Actor添加到UWorld::PersistentLevel的Actor列表。
  - 事件触发：广播OnActorSpawned事件，通知监听系统。
- **碰撞检测**  ：
  ```cpp
  // 使用Sweep检测避免穿模
  FCollisionQueryParams CollisionParams;
  CollisionParams.bTraceComplex = true;  // 使用复杂碰撞体
  if (World->SweepTestByChannel(StartLoc, EndLoc, FQuat::Identity, ECC_Visibility, CollisionShape, CollisionParams)) {
    // 处理碰撞阻挡
  }
  ``` 
- **碰撞处理**  ：强制生成（ESpawnActorCollisionHandleMethod::AlwaysSpawn）。

### 组件管理
- **默认组件**  ：创建 USceneComponent 作为根组件，用于坐标系管理。
- **示例**  ：
  ```cpp
  RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
  ```
- **可见性控制**  ：通过 bHiddenInGame 属性隐藏非渲染组件。

---

## 4. 代码实例与用法
### 创建 AI 控制器
  ```cpp
  // 定义Walker控制器类型
  auto WalkerController = ABFL::MakeGenericDefinition(
    TEXT("controller"), TEXT("ai"), TEXT("walker")
  );

  // 生成实例
  FTransform SpawnTransform; // 设置生成位置
  FactorSpawnResult Result = Factory->SpawnActor(SpawnTransform, WalkerController);
  ```

### 配置 Walker AI
- **属性修改**  ：通过客户端接口更新行人的目标位置或移动速度。
- **事件绑定**  ：注册碰撞事件或状态变更回调。

### 错误调试
- **日志排查**  ：
  ```cpp
  if (Controller == nullptr) {
    UE_LOG(logCarla, Error, TEXT("生成控制器失败"));
  }
  ```
- **断点调试**  ：检查 World 对象有效性及 SpawnParameters 配置。

---

## 5. 相关文件与依赖
### 头文件依赖树
```plaintext
AI/
    ├── AIControllerFactory.h  
    ├── CarlaActorFactory.h  
    ├── ActorBlueprintFunctionLibrary.h  
    ├── WalkerAIController.h  
    └── GameFramework/Actor.h  
 ```

### 依赖的引擎模块
- **Actor系统**  ：GameFramework/Actor.h（Actor基类）。
- **反射系统**  ：UObject/UObjectGlobals.h（支持 UCLASS 和 GENERATED_BODY）。
- **工具库**  ：ActorBlueprintFunctionLibrary.h（提供快速定义工具）。

### Build.cs配置示例
  ```cpp
  PublicDependencyModuleNames.AddRange(new string[] {
    "Core",
    "CoreUObject",
    "Engine",
    "Carla",  // CARLA插件模块
    "NavigationSystem"  // 依赖导航系统
  });
  ```

