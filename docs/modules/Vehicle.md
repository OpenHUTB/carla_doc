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

### 3.9PID类

​	1.这是一个实现了经典 PID（比例 - 积分 - 微分）控制算法的类。

​	2.PID成员变量

- `Kp`：比例系数，默认值为 0.0f。

- `Ki`：积分系数，默认值为 0.0f。

- `Kd`：微分系数，默认值为 0.0f。

- `SetPoint`：目标值。

- `MinOutput`：输出下限，默认值为 -1.0f。

- `MaxOutput`：输出上限，默认值为 1.0f。

- `Proportional`：比例项，初始值为 0.0f。

- `Integral`：积分项，初始值为 0.0f。

- `Derivative`：微分项，初始值为 0.0f。

- `LastError`：上一次的误差，初始值为 0.0f。

- `LastInput`：上一次的输入值，初始值为 0.0f。

  3.PID成员函数

  - `PID()`：默认构造函数。
  - `PID(float Kp, float Ki, float Kd)`：带参数的构造函数，用于初始化 PID 系数。
  - `~PID()`：默认析构函数。
  - `void SetTargetPoint(float Point)`：设置目标值。
  - `float Run(float Input, float DeltaTime)`：运行 PID 控制算法，根据输入值和时间间隔计算输出。
  - `void Reset()`：重置 PID 控制器的内部状态。

## 参考

* [函数vehile.apply_control实际上如何工作的？](https://github.com/carla-simulator/carla/issues/1427)
