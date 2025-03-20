### 一、整体概述



此代码实现了一个基于 PID 控制器的阿克曼车辆控制器 `FAckermannController`。PID 控制器用于控制车辆的速度和加速度，而 `FAckermannController` 则负责整合车辆状态信息、应用控制参数以及执行控制逻辑。

### 二、类及成员函数说明

#### 2.1`PID` 类

这是一个实现了经典 PID（比例 - 积分 - 微分）控制算法的类。

##### 2.2PID成员变量

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

##### 2.3PID成员函数

- `PID()`：默认构造函数。
- `PID(float Kp, float Ki, float Kd)`：带参数的构造函数，用于初始化 PID 系数。
- `~PID()`：默认析构函数。
- `void SetTargetPoint(float Point)`：设置目标值。
- `float Run(float Input, float DeltaTime)`：运行 PID 控制算法，根据输入值和时间间隔计算输出。
- `void Reset()`：重置 PID 控制器的内部状态。

#### 2.4`FAckermannController` 类

这是一个用于阿克曼车辆控制的类，包含了速度和加速度的 PID 控制器。

##### 2.5FAC成员变量

- `SpeedController`：速度 PID 控制器，初始参数为 Kp = 0.15f，Ki = 0.0f，Kd = 0.25f。
- `AccelerationController`：加速度 PID 控制器，初始参数为 Kp = 0.01f，Ki = 0.0f，Kd = 0.01f。
- `UserTargetPoint`：用户设定的目标控制值。
- `TargetSteer`：目标转向角，初始值为 0.0。
- `TargetSteerSpeed`：目标转向速度，初始值为 0.0。
- `TargetSpeed`：目标速度，初始值为 0.0。
- `TargetAcceleration`：目标加速度，初始值为 0.0。
- `TargetJerk`：目标加加速度，初始值为 0.0。
- `MaxAccel`：最大加速度，默认值为 3.0f [m/s²]。
- `MaxDecel`：最大减速度，默认值为 8.0f [m/s²]。
- `Steer`：转向控制值，初始值为 0.0f。
- `Throttle`：油门控制值，初始值为 0.0f。
- `Brake`：刹车控制值，初始值为 0.0f。
- `bReverse`：是否倒车标志，初始值为 false。
- `SpeedControlAccelDelta`：速度控制加速度增量，初始值为 0.0f。
- `SpeedControlAccelTarget`：速度控制目标加速度，初始值为 0.0f。
- `AccelControlPedalDelta`：加速度控制踏板增量，初始值为 0.0f。
- `AccelControlPedalTarget`：加速度控制目标踏板值，初始值为 0.0f。
- `DeltaTime`：仿真时间间隔，初始值为 0.0f [s]。
- `VehicleMaxSteering`：车辆最大转向角，初始值为 0.0f [rad]。
- `VehicleSteer`：车辆当前转向角，初始值为 0.0f [rad]。
- `VehicleSpeed`：车辆当前速度，初始值为 0.0f [m/s]。
- `VehicleAcceleration`：车辆当前加速度，初始值为 0.0f [m/s²]。
- `LastVehicleSpeed`：车辆上一次速度，初始值为 0.0f [m/s]。
- `LastVehicleAcceleration`：车辆上一次加速度，初始值为 0.0f [m/s²]。

##### 2.6FAC成员函数

- `FAckermannController()`：默认构造函数。
- `~FAckermannController()`：析构函数。
- `FAckermannControllerSettings GetSettings() const`：获取控制器的设置。
- `void ApplySettings(const FAckermannControllerSettings& Settings)`：应用控制器的设置。
- `void UpdateVehicleState(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的状态信息。
- `void UpdateVehiclePhysics(const ACarlaWheeledVehicle* Vehicle)`：更新车辆的物理信息。
- `void SetTargetPoint(const FVehicleAckermannControl& AckermannControl)`：设置目标控制值。
- `void Reset()`：重置控制器的内部状态。
- `void RunLoop(FVehicleControl& Control)`：运行控制循环，更新控制指令。
- `void RunControlSteering()`：执行横向控制（转向控制）。
- `bool RunControlFullStop()`：执行完全停车控制，返回是否成功停车。
- `void RunControlReverse()`：执行倒车控制。
- `void RunControlSpeed()`：执行速度控制。
- `void RunControlAcceleration()`：执行加速度控制。
- `void UpdateVehicleControlCommand()`：更新车辆的控制指令。

### 三、代码使用示例

```cpp
解释// 创建阿克曼控制器对象
FAckermannController Controller;

// 应用设置
FAckermannControllerSettings Settings;
Controller.ApplySettings(Settings);

// 更新车辆状态
ACarlaWheeledVehicle* Vehicle = ...; // 获取车辆指针
Controller.UpdateVehicleState(Vehicle);

// 设置目标控制值
FVehicleAckermannControl AckermannControl;
Controller.SetTargetPoint(AckermannControl);

// 运行控制循环
FVehicleControl Control;
Controller.RunLoop(Control);
```

### 四、注意事项

- 在使用 `PID` 控制器时，需要合理调整 `Kp`、`Ki` 和 `Kd` 参数，以获得良好的控制效果。
- 在调用 `RunLoop` 函数之前，需要确保已经更新了车辆的状态信息和设置了目标控制值。
- 代码中对积分项进行了饱和限制，以避免积分饱和问题。