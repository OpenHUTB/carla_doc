# 载具

VehicleControl 结构中的值直接传递给 PhysX Vehicle 插件（请参阅 [CarlaWheeledVehicle.cpp](https://github.com/carla-simulator/carla/blob/422d0de1c4ba34238d960a5405ab227044199e13/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp#L78-L102) ）。

# <一>、FAckermannController

## 一、`FAckermannController` 类

`FAckermannController` 类是一个用于实现阿克曼转向控制的控制器类，主要用于控制轮式车辆的行驶，包括速度、加速度、转向等方面的控制。该类使用了 PID 控制器来对速度和加速度进行调节，以实现对车辆的精确控制。

## 二、文件信息

### 包含头文件

- `AckermannController.h`：定义了 `FAckermannController` 类和相关结构体。
- `CarlaWheeledVehicle.h`：定义了 `ACarlaWheeledVehicle` 类，用于表示轮式车辆。
- 使用 `#pragma once` 指令，确保头文件仅被编译一次，避免因重复包含导致的编译错误，提升编译效率。

## 三、功能

### 3.1构造函数和析构函数

- `FAckermannController::~FAckermannController()`：析构函数，用于释放 `FAckermannController` 对象所占用的资源。

### 3.2获取和应用设置

- `FAckermannControllerSettings FAckermannController::GetSettings() const`：获取当前控制器的设置，包括速度和加速度 PID 控制器的参数。
- `void FAckermannController::ApplySettings(const FAckermannControllerSettings& Settings)`：应用传入的控制器设置，更新速度和加速度 PID 控制器的参数。

### 3.3设置目标点

- `void FAckermannController::SetTargetPoint(const FVehicleAckermannControl& AckermannControl)`：设置目标控制参数，包括转向角、转向速度、速度、加速度和 jerk 等，并对这些参数进行裁剪。

### 3.4重置控制器

- `void FAckermannController::Reset()`：重置控制器的状态，包括速度和加速度 PID 控制器的状态，以及车辆的控制参数和状态。

### 3.5运行控制循环

- `void FAckermannController::RunLoop(FVehicleControl& Control)`：运行控制循环，包括横向控制和纵向控制，最后更新车辆的控制命令。

### 3.6横向控制

- `void FAckermannController::RunControlSteering()`：根据目标转向角和转向速度，计算当前的转向角。

### 3.7纵向控制

1. 完全停止控制
   - `bool FAckermannController::RunControlFullStop()`：判断车辆是否需要完全停止，如果需要则将刹车设置为最大值，油门设置为零。
2. 倒车控制
   - `void FAckermannController::RunControlReverse()`：根据车辆的当前速度和目标速度，判断是否可以切换到倒车模式。
3. 速度控制
   - `void FAckermannController::RunControlSpeed()`：使用速度 PID 控制器计算目标加速度，并对其进行裁剪。
4. 加速度控制
   - `void FAckermannController::RunControlAcceleration()`：使用加速度 PID 控制器计算目标油门或刹车值，并对其进行裁剪。
5. 更新车辆控制命令
   - `void FAckermannController::UpdateVehicleControlCommand()`：根据目标油门或刹车值，更新车辆的油门和刹车控制命令。

### 3.8更新车辆状态和物理参数

1. 更新车辆状态
   
   - `void FAckermannController::UpdateVehicleState(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的状态，包括转向角、速度和加速度等。
2. 更新车辆物理参数

   - `void FAckermannController::UpdateVehiclePhysics(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的最大转向角。

3.转向控制

```
USTRUCT(BlueprintType)
struct CARLA_API FVehicleAckermannControl
{
  GENERATED_BODY()

  UPROPERTY(Category = "Vehicle Ackermann Control", EditAnywhere, BlueprintReadWrite)
  float Steer = 0.0f;

  UPROPERTY(Category = "Vehicle Ackermann Control", EditAnywhere, BlueprintReadWrite)
  float SteerSpeed = 0.0f;

  UPROPERTY(Category = "Vehicle Ackermann Control", EditAnywhere, BlueprintReadWrite)
  float Speed = 0.0f;

  UPROPERTY(Category = "Vehicle Ackermann Control", EditAnywhere, BlueprintReadWrite)
  float Acceleration = 0.0f;

  UPROPERTY(Category = "Vehicle Ackermann Control", EditAnywhere, BlueprintReadWrite)
  float Jerk = 0.0f;
};
```

该代码定义了一个用于车辆阿克曼转向控制的结构体，能够在虚幻引擎的蓝图系统中使用。通过这个结构体，开发者可以对车辆的转向角度、转向速度、行驶速度、加速度和加加速度等参数进行方便的管理和调整，进而实现车辆的阿克曼转向控制模拟。

### 3.9车辆灯光状态

​	此结构体运用了 Unreal Engine 的反射系统，具备在蓝图里进行读写操作的能力。它的每个属性都代表着一种特定的灯光状态，并且默认情况下这些灯光都是关闭的。

​	新添加的 `Hazard` 属性用于表示车辆危险警示灯的状态，默认值为 `false`，即初始状态下危险警示灯是关闭的。和其他属性一样，它也可以在编辑器中编辑，并且能在蓝图里进行读写操作。

```
// 新增的危险警示灯状态属性
  UPROPERTY(Category = "Vehicle Lights", EditAnywhere, BlueprintReadWrite)
  bool Hazard = false;
```

3.10车辆的阿克曼转向控制参数



### 3.11功能补充

在Vehicle/VehiclePhysicsControl.h**文件中添加新的设置

```
// 刹车设置
UPROPERTY(Category = "Vehicle Brake Physics Control", EditAnywhere, BlueprintReadWrite)
 float BrakeForce = 0.0f;

UPROPERTY(Category = "Vehicle Brake Physics Control", EditAnywhere, BlueprintReadWrite)
 float HandbrakeForce = 0.0f;
```

- `BrakeForce`：代表车辆刹车时的制动力，默认值为 0.0f。
- `HandbrakeForce`：表示手刹的制动力，默认值为 0.0f。

## 四、类型

### 4.1枚举类型

1.定义：使用 `UENUM(BlueprintType)` 宏定义一个可在蓝图系统中使用的枚举类型 `ECarlaWheeledVehicleState`。其底层类型为 `uint8`，属于强类型枚举，可避免枚举值命名冲突。

 2.代码使用场景：

- **调试**：开发人员在调试 CARLA 轮式车辆相关功能时，可借助该枚举类型清晰地表示车辆的不同状态，便于定位和解决问题。
- **可视化**：在 Unreal Engine 编辑器中，可通过显示名称直观地查看车辆状态，方便开发和测试。
- **蓝图开发**：该枚举类型可在蓝图系统中使用，方便非编程人员通过蓝图可视化编程的方式使用车辆状态信息。

3.注意事项

- 若对代码进行修改，需注意生成文件 `CarlaWheeledVehicleState.generated.h` 可能需要重新生成，以确保反射信息和蓝图绑定的正确性。
- 由于枚举类型为强类型，在使用时需明确指定枚举类型，避免类型不匹配问题。

4.功能补充

新增枚举成员：
Turning：表示车辆正在转弯的状态。
Parking：表示车辆正在停车的状态。

```
case ECarlaWheeledVehicleState::Turning:
   return TEXT("Turning");
case ECarlaWheeledVehicleState::Parking:
   return TEXT("Parking");
```

注：该代码补充对应文档**CarlaWheeledVehicleState.h**







# <二>WheeledVehicleAIController 类代码文档

## 一、概述

`WheeledVehicleAIController`类是用于控制轮式车辆的人工智能控制器，主要服务于车辆模拟场景。该类通过实现自动驾驶、障碍物检测、路线规划等功能，管理车辆在虚拟环境中的行为。它依赖`ACarlaWheeledVehicle`、`ARoutePlanner`等多个类，协同完成车辆控制逻辑。

## 二、代码结构与功能模块

### 2.1 静态局部方法

1. RayCast 方法
   - **功能**：通过光线投射，判断从起点`Start`到终点`End`之间是否存在阻挡物。该方法使用`LineTraceSingleByObjectType`执行光线投射，忽略发起投射的`Actor`本身，并依据投射结果判断是否击中阻挡物。
   - 参数
     - `Actor`：发起光线投射的`AActor`对象。
     - `Start`：光线投射起始位置向量。
     - `End`：光线投射结束位置向量。
   - **返回值**：若光线投射击中阻挡物，返回`true`；否则返回`false`。
2. IsThereAnObstacleAhead 方法
   - **功能**：判断车辆前方是否存在障碍物。通过获取车辆前向向量、边界框大小等信息，结合速度计算光线投射距离，分别在车辆中心、右侧、左侧三个方向执行光线投射，只要有一个方向检测到障碍物即返回`true`。
   - 参数
     - `Vehicle`：被检测的`ACarlaWheeledVehicle`对象。
     - `Speed`：车辆当前速度。
     - `Direction`：车辆行驶方向向量。
   - **返回值**：若车辆前方存在障碍物，返回`true`；否则返回`false`。
3. ClearQueue 方法
   - **功能**：清空指定类型的队列`Queue`。通过创建一个空队列并与原队列交换，实现清空操作。
   - 参数
     - `Queue`：待清空的`std::queue<T>`类型队列。
   - **返回值**：无。

### 2.2 构造函数与析构函数

1. 构造函数 AWheeledVehicleAIController
   - **功能**：初始化`AWheeledVehicleAIController`对象，创建随机数引擎`RandomEngine`并设置随机种子，同时配置该控制器可参与`Tick`更新，且`Tick`优先级设定在物理计算之前。
   - 参数
     - `ObjectInitializer`：用于初始化对象的参数。
2. 析构函数～AWheeledVehicleAIController
   - **功能**：释放`AWheeledVehicleAIController`对象占用的资源，当前为空实现，后续若有资源分配需补充释放逻辑。

### 2.3 AController 相关方法

1. OnPossess 方法
   - **功能**：当控制器接管`pawn`时触发，首先调用父类`OnPossess`方法，接着判断是否已控制车辆，若已控制则输出错误日志并返回；然后将传入的`pawn`转换为`ACarlaWheeledVehicle`类型，获取车辆最大转向角度，配置自动驾驶参数，并在未设置路线图时尝试从世界中获取。
   - 参数
     - `aPawn`：被接管的`APawn`对象。
2. OnUnPossess 方法
   - **功能**：当控制器失去对`pawn`的控制时调用，调用父类对应方法，并将车辆指针`Vehicle`置空。
   - **参数**：无。
3. Tick 方法
   - **功能**：每帧调用，先标记性能分析事件范围，调用父类`Tick`方法；若未控制车辆则直接返回；若未启用自动驾驶且控制不具粘性，放松车辆控制；最后刷新车辆控制。
   - 参数
     - `DeltaTime`：距上一帧的时间间隔。

### 2.4 自动驾驶相关方法

1. ConfigureAutopilot 方法
   - **功能**：根据传入参数配置自动驾驶状态。设置自动驾驶启用标志`bAutopilotEnabled`，若不保留当前状态，则重置车辆转向、油门、刹车等控制输入，清空目标位置队列，设置车辆自动驾驶状态，并将交通灯状态设为绿灯。
   - 参数
     - `Enable`：是否启用自动驾驶。
     - `KeepState`：是否保留当前状态。

### 2.5 交通相关方法

1. SetFixedRoute 方法
   - **功能**：设置车辆固定行驶路线。若`bOverwriteCurrent`为`true`，清空当前目标位置队列，再将新的位置依次添加到队列中。
   - 参数
     - `Locations`：包含目标位置的`TArray<FVector>`数组。
     - `bOverwriteCurrent`：是否覆盖当前路线。

## 三、流程图展示



```
初始化流程（蓝色）：

OnPossess：控制器接管车辆时的初始化过程

ConfigureAutopilot：自动驾驶配置过程

SetFixedRoute：设置固定路线流程

持续运行流程（橙色）：

Tick：每帧执行的主逻辑流程

决策节点（红色）：

各种条件判断（是否控制车辆、自动驾驶是否启用等）

执行动作（绿色）：

车辆控制相关操作

数据清理操作

状态更新操作

特殊节点：

绿色开始节点

橙色时钟样式的Tick循环节点

流程展示了从控制器初始化、持续运行到车辆控制的全过程，包括异常处理、自动驾驶逻辑、障碍物检测、路线跟随等重要功能模块的交互关系。
```

# <三>`UWheeledVehicleMovementComponentNW` 类文档

## 一、类的概述

`UWheeledVehicleMovementComponentNW` 类继承自某个基类（`Super`），主要用于处理轮式车辆的运动相关设置和操作，包括车辆的发动机设置、变速器设置、车轮设置等，同时负责与物理引擎（PhysX）进行交互，以实现车辆的模拟和运动更新。

## 二、成员变量

1. **`EngineSetup`**：存储车辆发动机相关的数据，包括转动惯量（`MOI`）、最大转速（`MaxRPM`）、不同工况下的阻尼率等，这些数据用于配置发动机的行为。
2. **`TransmissionSetup`**：包含车辆变速器的设置信息，如离合器强度（`ClutchStrength`）、换挡时间（`GearSwitchTime`）、倒档齿轮比（`ReverseGearRatio`）、最终传动比（`FinalRatio`）以及前进档的相关数据（`ForwardGears`）等，用于控制车辆的换挡逻辑和传动系统。
3. **`WheelSetups`**：一个数组，用于存储车轮的设置信息，每个元素对应一个车轮的设置，包括车轮的骨骼名称等。
4. **`DifferentialSetup`**：存储车辆差速器的设置数据，用于配置差速器的行为。
5. **`IdleBrakeInput`**：表示车辆的闲置刹车输入值。
6. **`SteeringCurve`**：一个富曲线（`FRichCurve`），用于描述车辆转向速度与转向角度之间的关系，通过添加不同速度下的转向比例关键帧来定义。

## 三、构造函数 `UWheeledVehicleMovementComponentNW(const FObjectInitializer& ObjectInitializer)`

1. **功能**：初始化车辆运动组件的默认设置，从 PhysX 获取默认的发动机数据，并设置发动机的相关参数，如转动惯量、最大转速、阻尼率等。同时初始化变速器的一些默认值，以及设置转向速度曲线的初始关键帧，并初始化车轮设置数组和差速器设置数组。
2. **参数**：`ObjectInitializer` 用于初始化对象的参数。

## 四、编辑器相关函数 `PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)`

1. **功能**：当在编辑器中修改组件的属性时被调用，根据修改的属性（`DownRatio`、`UpRatio`、`SteeringCurve`）对相关设置进行调整。例如，当修改 `DownRatio` 时，确保每个前进档的 `DownRatio` 不大于 `UpRatio`；当修改 `UpRatio` 时，确保 `UpRatio` 不小于 `DownRatio`；当修改 `SteeringCurve` 时，确保曲线上的值在 0 到 1 之间。

2. **参数**：`PropertyChangedEvent` 包含了属性变化的相关信息，如修改的属性名称等。

   ## 五、静态函数

   5. 1`GetVehicleDifferentialNWSetup(const TArray<FVehicleNWWheelDifferentialData>& Setup, PxVehicleDifferentialNWData& PxSetup)`

   1. **功能**：根据给定的车轮差速器设置数组，设置 PhysX 车辆差速器的数据，将每个车轮的驱动状态设置到 `PxSetup` 中。
   2. 参数：
      - `Setup`：一个包含车辆车轮差速器数据的数组。
      - `PxSetup`：PhysX 车辆差速器数据对象，用于存储设置后的差速器数据。

   5.2、静态函数 `GetVehicleEngineSetup(const FVehicleNWEngineData& Setup, PxVehicleEngineData& PxSetup)`

   1. **功能**：根据给定的车辆发动机数据，设置 PhysX 车辆发动机的数据。包括将发动机的转动惯量、最大转速、阻尼率等从自定义单位转换为 PhysX 所需的单位，并设置扭矩曲线数据，将自定义的扭矩曲线转换为 PhysX 格式的扭矩曲线。
   2. 参数：
      - `Setup`：包含车辆发动机数据的对象。
      - `PxSetup`：PhysX 车辆发动机数据对象，用于存储设置后的发动机数据。

   5.3、静态函数 `GetVehicleGearSetup(const FVehicleNWTransmissionData& Setup, PxVehicleGearsData& PxSetup)`

   1. **功能**：根据给定的车辆变速器数据，设置 PhysX 车辆齿轮的数据。包括设置换挡时间、倒档齿轮比、前进档齿轮比以及最终传动比等。
   2. 参数：
      - `Setup`：包含车辆变速器数据的对象。
      - `PxSetup`：PhysX 车辆齿轮数据对象，用于存储设置后的齿轮数据。

   5.4、静态函数 `GetVehicleAutoBoxSetup(const FVehicleNWTransmissionData& Setup, PxVehicleAutoBoxData& PxSetup)`

   1. **功能**：根据给定的车辆变速器数据，设置 PhysX 车辆自动变速器的数据。包括设置每个前进档的升档比和降档比，以及空档的升档比和自动变速器的延迟时间。
   2. 参数：
      - `Setup`：包含车辆变速器数据的对象。
      - `PxSetup`：PhysX 车辆自动变速器数据对象，用于存储设置后的自动变速器数据。

1. ## 五、成员函数 

   `（1）FindPeakTorque()`

   1. **功能**：在发动机扭矩曲线中查找最大扭矩值，通过遍历扭矩曲线的关键帧，找出扭矩值的最大值。
   2. **返回值**：返回找到的最大扭矩值。

   （2）`GetCustomGearBoxNumForwardGears() const`

   1. **功能**：获取自定义变速箱的前进档数量，返回存储前进档数据的数组的数量。
   2. **返回值**：返回前进档的数量。

   （3）、成员函数 `SetupVehicle()`

   1. **功能**：设置车辆的相关属性和物理模拟数据。首先检查更新的基元（`UpdatedPrimitive`），如果车轮设置数量小于 2 则不进行设置。然后设置车辆的形状、质量属性，分配并设置车轮数据，根据车辆数据和车轮模拟数据设置驱动数据，创建车辆驱动对象，将驱动数据应用到物理引擎中的车辆对象上，并缓存车辆和车辆驱动对象。
   2. **注意事项**：在设置过程中，对物理引擎的操作是通过 `FPhysicsCommand::ExecuteWrite` 进行的，确保在正确的上下文中调用。

   （4）成员函数 `UpdateSimulation(float DeltaTime)`

   1. **功能**：更新车辆的模拟数据。如果车辆驱动对象为空则不进行更新。设置原始输入数据，包括油门、转向、刹车和手刹的输入值。根据自动变速器的使用情况设置升档和降档的输入。将转向曲线转换为 PhysX 格式的固定大小查找表，设置平滑数据，并使用这些数据更新车辆的驱动模拟数据。
   2. **参数**：`DeltaTime` 表示时间间隔，用于模拟的时间步长。

   （5）成员函数 `UpdateEngineSetup(const FVehicleNWEngineData& NewEngineSetup)`

   1. **功能**：更新车辆的发动机设置。如果车辆驱动对象不为空，根据新的发动机设置数据，设置 PhysX 车辆发动机的数据。
   
2. **参数**：`NewEngineSetup` 包含新的发动机设置数据的对象。
   
      （6）、成员函数 `UpdateDifferentialSetup(const TArray<FVehicleNWWheelDifferentialData>& NewDifferentialSetup)`
   
      1. **功能**：更新车辆的差速器设置。如果车辆驱动对象不为空，根据新的差速器设置数组，设置 PhysX 车辆差速器的数据。
      2. **参数**：`NewDifferentialSetup` 包含新的差速器设置数据的数组。
      
      
   （7）、成员函数 `UpdateTransmissionSetup(const FVehicleNWTransmissionData& NewTransmissionSetup)`
      
      1. **功能**：更新车辆的变速器设置。如果车辆驱动对象不为空，根据新的变速器设置数据，设置 PhysX 车辆的齿轮数据和自动变速器数据。
   2. **参数**：`NewTransmissionSetup` 包含新的变速器设置数据的对象。
         
   
      （8）、成员函数 `BackwardsConvertCm2ToM2NW(float& val, float defaultValue)`
      
      1. **功能**：将从厘米平方到米平方的单位进行转换，如果 `val` 的值不等于默认值，则进行转换。
      2. 参数：
      - `val`：需要转换的数值。
         - `defaultValue`：默认值，用于判断是否进行转换。
      
      
      （9）、成员函数 `Serialize(FArchive& Ar)`
   
      1. **功能**：对组件进行序列化和反序列化操作。在反序列化时（加载时），根据不同的 UE4 版本进行单位转换，将发动机和变速器的一些设置从旧单位转换为新单位，确保兼容性。
   2. **参数**：`Ar` 用于进行序列化和反序列化的存档对象。
         
      
      （10）、成员函数 `ComputeConstants()`
      
      1. **功能**：计算一些常量，调用基类的 `ComputeConstants` 函数，并将发动机的最大转速（`MaxEngineRPM`）设置为发动机设置中的最大转速（`EngineSetup.MaxRPM`）。

   （11）、成员函数 `GetTireData(physx::PxVehicleWheels* InWheels, UVehicleWheel* InWheel)`

1. **功能**：获取轮胎数据，根据给定的 PhysX 车辆车轮对象和车辆车轮对象，返回对应的轮胎数据指针。
2. 参数：
   - `InWheels`：PhysX 车辆车轮对象。
   - `InWheel`：车辆车轮对象。
   - **返回值**：返回轮胎数据的指针。

（12）、成员函数 `GetWheelShapeMapping(physx::PxVehicleWheels* InWheels, uint32 InWheel)`

1. **功能**：获取车轮形状映射，根据给定的 PhysX 车辆车轮对象和车轮索引，返回对应的车轮形状索引。
2. 参数：
   - `InWheels`：PhysX 车辆车轮对象。
   - `InWheel`：车轮索引。
   - **返回值**：返回车轮形状索引。

（13）、成员函数 `GetWheelData(physx::PxVehicleWheels* InWheels, uint32 InWheel)`

1. **功能**：获取车轮数据，根据给定的 PhysX 车辆车轮对象和车轮索引，返回对应的车轮数据。
2. 参数：
   - `InWheels`：PhysX 车辆车轮对象。
   - `InWheel`：车轮索引。
   - **返回值**：返回车轮数据对象。





## 参考

* [函数vehile.apply_control实际上如何工作的？](https://github.com/carla-simulator/carla/issues/1427)
