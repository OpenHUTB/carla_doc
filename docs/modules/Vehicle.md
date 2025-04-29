# 载具

VehicleControl 结构中的值直接传递给 PhysX Vehicle 插件（请参阅 [CarlaWheeledVehicle.cpp](https://github.com/carla-simulator/carla/blob/422d0de1c4ba34238d960a5405ab227044199e13/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp#L78-L102) ）。



## 一、`FAckermannController` 类

`FAckermannController` 类是一个用于实现阿克曼转向控制的控制器类，主要用于控制轮式车辆的行驶，包括速度、加速度、转向等方面的控制。该类使用了 PID 控制器来对速度和加速度进行调节，以实现对车辆的精确控制。

## 二、文件信息

### 版权信息

该代码版权归 2021 年巴塞罗那自治大学（UAB）的计算机视觉中心（CVC）所有，代码遵循 MIT 许可证。

### 包含头文件

- `AckermannController.h`：定义了 `FAckermannController` 类和相关结构体。
- `CarlaWheeledVehicle.h`：定义了 `ACarlaWheeledVehicle` 类，用于表示轮式车辆。
- 使用 `#pragma once` 指令，确保头文件仅被编译一次，避免因重复包含导致的编译错误，提升编译效率。

## 三、类成员和方法

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

## 参考

* [函数vehile.apply_control实际上如何工作的？](https://github.com/carla-simulator/carla/issues/1427)
