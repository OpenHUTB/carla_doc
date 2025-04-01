# 载具

VehicleControl 结构中的值直接传递给 PhysX Vehicle 插件（请参阅 [CarlaWheeledVehicle.cpp](https://github.com/carla-simulator/carla/blob/422d0de1c4ba34238d960a5405ab227044199e13/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/CarlaWheeledVehicle.cpp#L78-L102) ）。



## 一、`FAckermannController` 类

`FAckermannController` 类是一个用于实现阿克曼转向控制的控制器类，主要用于控制轮式车辆的行驶，包括速度、加速度、转向等方面的控制。该类使用了 PID 控制器来对速度和加速度进行调节，以实现对车辆的精确控制。

### 1.1文件信息

版权信息：

该代码版权归 2021 年巴塞罗那自治大学（UAB）的计算机视觉中心（CVC）所有，代码遵循 MIT 许可证。

包含头文件：

- `AckermannController.h`：定义了 `FAckermannController` 类和相关结构体。
- `CarlaWheeledVehicle.h`：定义了 `ACarlaWheeledVehicle` 类，用于表示轮式车辆。

### 1.2类成员和方法

#### 3.1构造函数和析构函数

- `FAckermannController::~FAckermannController()`：析构函数，用于释放 `FAckermannController` 对象所占用的资源。

#### 3.2获取和应用设置

- `FAckermannControllerSettings FAckermannController::GetSettings() const`：获取当前控制器的设置，包括速度和加速度 PID 控制器的参数。
- `void FAckermannController::ApplySettings(const FAckermannControllerSettings& Settings)`：应用传入的控制器设置，更新速度和加速度 PID 控制器的参数。

#### 3.3设置目标点

- `void FAckermannController::SetTargetPoint(const FVehicleAckermannControl& AckermannControl)`：设置目标控制参数，包括转向角、转向速度、速度、加速度和 jerk 等，并对这些参数进行裁剪。

#### 3.4重置控制器

- `void FAckermannController::Reset()`：重置控制器的状态，包括速度和加速度 PID 控制器的状态，以及车辆的控制参数和状态。

#### 3.5运行控制循环

- `void FAckermannController::RunLoop(FVehicleControl& Control)`：运行控制循环，包括横向控制和纵向控制，最后更新车辆的控制命令。

#### 3.6横向控制

- `void FAckermannController::RunControlSteering()`：根据目标转向角和转向速度，计算当前的转向角。

#### 3.7纵向控制

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

#### 3.8更新车辆状态和物理参数

1. 更新车辆状态
   - `void FAckermannController::UpdateVehicleState(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的状态，包括转向角、速度和加速度等。
2. 更新车辆物理参数
   - `void FAckermannController::UpdateVehiclePhysics(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的最大转向角。

此代码定义了一个名为 `FAckermannControllerSettings` 的结构体，用于存储阿克曼控制器的各项设置参数。该结构体使用了虚幻引擎（UE）的相关宏，可在蓝图中使用，且其成员变量能在编辑器中进行编辑。

## 二、`AckermannControllerSettings类

### 2.1文件信息

1. **版权与许可信息**：代码遵循 MIT 许可协议，版权归 2021 年巴塞罗那自治大学（UAB）的计算机视觉中心（CVC）所有。
2. **头文件包含**：包含了 `AckermannControllerSettings.generated.h` 头文件，该文件用于生成与当前结构体相关的代码。
3. 结构体定义：
   - 使用 `USTRUCT` 宏定义了 `FAckermannControllerSettings` 结构体，并标记为可在蓝图中使用（`BlueprintType`）。
   - `GENERATED_BODY()` 宏用于生成结构体的必要代码。

### 2.2成员方法

1. 成员变量：
   - 结构体包含多个成员变量，这些变量使用 `UPROPERTY` 宏暴露给 UE 的属性系统。
   - 所有成员变量都属于 `"Ackermann Controller Settings"` 分类，可在编辑器中任何位置进行编辑（`EditAnywhere`），并且在蓝图中可读可写（`BlueprintReadWrite`）。
   - 成员变量分为速度控制和加速度控制两组，每组包含比例系数（`Kp`）、积分系数（`Ki`）和微分系数（`Kd`），初始值均为 0.0f。

​	

## 参考

* [函数vehile.apply_control实际上如何工作的？](https://github.com/carla-simulator/carla/issues/1427)
