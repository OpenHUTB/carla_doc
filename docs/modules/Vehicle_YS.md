# Vehicle详细解释文档

## 一、概述

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

## 四、详细代码解释

### 4.1获取和应用设置

```cpp
FAckermannControllerSettings FAckermannController::GetSettings() const {
  FAckermannControllerSettings Settings;
 
  Settings.SpeedKp = SpeedController.Kp;
  Settings.SpeedKi = SpeedController.Ki;
  Settings.SpeedKd = SpeedController.Kd;
 
  Settings.AccelKp = AccelerationController.Kp;
  Settings.AccelKi = AccelerationController.Ki;
  Settings.AccelKd = AccelerationController.Kd;
 
  return Settings;
}
 
void FAckermannController::ApplySettings(const FAckermannControllerSettings& Settings) {
  SpeedController.Kp = Settings.SpeedKp;
  SpeedController.Ki = Settings.SpeedKi;
  SpeedController.Kd = Settings.SpeedKd;
 
  AccelerationController.Kp = Settings.AccelKp;
  AccelerationController.Ki = Settings.AccelKi;
  AccelerationController.Kd = Settings.AccelKd;
}
```

- `GetSettings` 方法将速度和加速度 PID 控制器的参数复制到 `FAckermannControllerSettings` 结构体中并返回。
- `ApplySettings` 方法将传入的 `FAckermannControllerSettings` 结构体中的参数应用到速度和加速度 PID 控制器中。

### 4.1.1 构造函数
  ```cpp
  // 默认构造函数，初始化PID参数及车辆状态
FAckermannController::FAckermannController() {
    // 初始化PID参数（假设默认值）
    SpeedController.Kp = 0.5f;
    SpeedController.Ki = 0.1f;
    SpeedController.Kd = 0.2f;
    
    AccelerationController.Kp = 1.0f;
    AccelerationController.Ki = 0.5f;
    AccelerationController.Kd = 0.3f;
    
    // 其他状态初始化
    Reset();
}
```
### 4.2设置目标点

```cpp
void FAckermannController::SetTargetPoint(const FVehicleAckermannControl& AckermannControl) {
  UserTargetPoint = AckermannControl;
 
  TargetSteer = FMath::Clamp(UserTargetPoint.Steer, -VehicleMaxSteering, VehicleMaxSteering);
  TargetSteerSpeed = UserTargetPoint.SteerSpeed;//保留符号以支持双向转向速度
  TargetSpeed = UserTargetPoint.Speed;
  TargetAcceleration = FMath::Abs(UserTargetPoint.Acceleration);
  TargetJerk = FMath::Abs(UserTargetPoint.Jerk);
}
```



- 将传入的 `FVehicleAckermannControl` 结构体赋值给 `UserTargetPoint`。
- 对转向角进行裁剪，确保其在最大转向角范围内。
- 取转向速度、加速度和 jerk 的绝对值。

### 4.3重置控制器

```cpp
void FAckermannController::Reset() {
  // 重置控制器
  SpeedController.Reset();
  AccelerationController.Reset();
 
  // 重置控制器参数
  Steer = 0.0f;
  Throttle = 0.0f;
  Brake = 0.0f;
  bReverse = false;
 
  SpeedControlAccelDelta = 0.0f;
  SpeedControlAccelTarget = 0.0f;
 
  AccelControlPedalDelta = 0.0f;
  AccelControlPedalTarget = 0.0f;
 
  // 重置车辆状态
  VehicleSpeed = 0.0f;
  VehicleAcceleration = 0.0f;
 
  LastVehicleSpeed = 0.0f;
  LastVehicleAcceleration = 0.0f;
}
```

- 重置速度和加速度 PID 控制器的状态。
- 将控制器的参数和车辆的状态重置为零。

### 4.4运行控制循环

```cpp
void FAckermannController::RunLoop(FVehicleControl& Control) {
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
 
  // 横向控制
  RunControlSteering();
 
  // 纵向控制
  bool bStopped = RunControlFullStop();
  if (!bStopped) {
    RunControlReverse();
    RunControlSpeed();
    RunControlAcceleration();
    UpdateVehicleControlCommand();
  }
 
  // 更新控制命令
  Control.Steer = Steer / VehicleMaxSteering;
  Control.Throttle = FMath::Clamp(Throttle, 0.0f, 1.0f);
  Control.Brake = FMath::Clamp(Brake, 0.0f, 1.0f);
  Control.bReverse = bReverse;
}
```

- 首先进行横向控制，调用 `RunControlSteering` 方法。
- 然后进行纵向控制，先判断是否需要完全停止，若不需要则依次进行倒车控制、速度控制、加速度控制和更新车辆控制命令。
- 最后更新传入的 `FVehicleControl` 结构体中的控制命令。

### 4.5横向控制

```cpp
void FAckermannController::RunControlSteering() {
  if (FMath::Abs(TargetSteerSpeed) < 0.001) {
    Steer = TargetSteer;
  } else {
    float SteerDelta = TargetSteerSpeed * DeltaTime;
    if (FMath::Abs(TargetSteer - VehicleSteer) < SteerDelta) {
      Steer = TargetSteer;
    } else {
      float SteerDirection = (TargetSteer > VehicleSteer) ? 1.0f : -1.0f;
      Steer = VehicleSteer + SteerDirection * (TargetSteerSpeed * DeltaTime);
    }
  }
}
```



- 如果目标转向速度小于 0.001，则直接将转向角设置为目标转向角。
- 否则，根据目标转向速度和时间步长计算转向角的变化量，并根据当前转向角和目标转向角的差值进行更新。

### 4.6纵向控制

#### 4.6.1完全停止控制

```cpp
bool FAckermannController::RunControlFullStop() {
  // 从这个速度在全刹车打开
  float FullStopEpsilon = 0.1; //[m/s]
 
  if (FMath::Abs(VehicleSpeed) < FullStopEpsilon && FMath::Abs(UserTargetPoint.Speed) < FullStopEpsilon) {
    Brake = 1.0;
    Throttle = 0.0;
    return true;
  }
  return false;
}
```



- 如果车辆的当前速度和目标速度都小于 0.1m/s，则将刹车设置为最大值，油门设置为零，并返回 `true`。
- 否则返回 `false`。

#### 4.6.2倒车控制

```cpp
void FAckermannController::RunControlReverse() {
  // 从这个位置上可以切换到倒档
  float StandingStillEpsilon = 0.1;  // [m/s]
 
  if (FMath::Abs(VehicleSpeed) < StandingStillEpsilon) {
    // 停车不动，允许改变行驶方向
    if (TargetPoint.Speed < 0) { //使用裁剪后的目标速度，防止出现可能导致逻辑混乱
      // 改变驾驶方向到倒车。
      bReverse = true;
    } else if (UserTargetPoint.Speed >= 0) {
      // 将驾驶方向改为前进。
      bReverse = false;
    }
  } else {
    if (FMath::Sign(VehicleSpeed) * FMath::Sign(UserTargetPoint.Speed) == -1) {
      // 请求改变驾驶方向。
      // 首先，我们必须完全停下来，然后才能改变行驶方向
      TargetSpeed = 0.0;
    }
  }
}
```



- 如果车辆的当前速度小于 0.1m/s，则可以根据目标速度的正负来切换行驶方向。
- 如果车辆正在行驶且目标速度的方向与当前速度的方向相反，则将目标速度设置为零，需要先完全停止才能改变行驶方向。

#### 4.6.3速度控制
在速度控制中，MaxAccel和MaxDecel代表车辆最大物理加速度/减速度。
```cpp
// 类成员变量（需在类定义中声明）
float MaxAccel = 3.0f;   // 默认最大加速度 3m/s²
float MaxDecel = 8.0f;   // 默认最大减速度 8m/s²（紧急制动）

// 或通过车辆物理参数动态获取
void FAckermannController::UpdateVehiclePhysics(const ACarlaWheeledVehicle* Vehicle) {
    VehicleMaxSteering = FMath::DegreesToRadians(Vehicle->GetMaximumSteerAngle());
    MaxAccel = Vehicle->GetMaxAcceleration(); // 假设存在此方法
    MaxDecel = Vehicle->GetMaxBrakingDeceleration();
}
```
```cpp
void FAckermannController::RunControlSpeed() {
  SpeedController.SetTargetPoint(TargetSpeed);
  SpeedControlAccelDelta = SpeedController.Run(VehicleSpeed, DeltaTime);
 
  // 剪裁边界
  float ClippingLowerBorder = -FMath::Abs(TargetAcceleration);
  float ClippingUpperBorder = FMath::Abs(TargetAcceleration);
  if (FMath::Abs(TargetAcceleration) < 0.0001f) {
    // 根据 AckermannDrive 的定义：如果为零，则使用最大值
    ClippingLowerBorder = -MaxDecel;
    ClippingUpperBorder = MaxAccel;
  }
  SpeedControlAccelTarget += SpeedControlAccelDelta;
  SpeedControlAccelTarget = FMath::Clamp(SpeedControlAccelTarget,
      ClippingLowerBorder, ClippingUpperBorder);
}
```



- 设置速度 PID 控制器的目标速度。
- 运行速度 PID 控制器，计算加速度的变化量。
- 根据目标加速度对计算得到的加速度进行裁剪。

#### 4.6.4.加速度控制

```cpp
void FAckermannController::RunControlAcceleration() {
  AccelerationController.SetTargetPoint(SpeedControlAccelTarget);
  AccelControlPedalDelta = AccelerationController.Run(VehicleAcceleration, DeltaTime);
 
  // 剪裁边界
  AccelControlPedalTarget += AccelControlPedalDelta;
  AccelControlPedalTarget = FMath::Clamp(AccelControlPedalTarget, -1.0f, 1.0f);
}
```



- 设置加速度 PID 控制器的目标加速度。
- 运行加速度 PID 控制器，计算油门或刹车值的变化量。
- 对计算得到的油门或刹车值进行裁剪，确保其在 -1.0 到 1.0 之间。

#### 4.6.5 更新车辆控制命令

```cpp
void FAckermannController::UpdateVehicleControlCommand() {
   // 负值：减速请求，正值：加速请求
  if (AccelControlPedalTarget < 0.0f) {
    // 减速请求
    if (bReverse) {
      Throttle = FMath::Abs(AccelControlPedalTarget);
      Brake = 0.0f;
    } else {
      // 前进时：负加速度对应刹车
      Throttle = 0.0f;
      Brake = FMath::Abs(AccelControlPedalTarget);
    }
  } else {
    // 加速请求
    if (bReverse) {
     // 倒车时：正加速度对应刹车（减速倒车）
      Throttle = 0.0f;
      Brake = FMath::Abs(AccelControlPedalTarget);
    } else {
      // 前进时：正加速度对应油门
      Throttle = FMath::Abs(AccelControlPedalTarget);
      Brake = 0.0f;
    }
  }
}
```

根据目标油门或刹车值和倒车标志，更新车辆的油门和刹车控制命令。

### （七）更新车辆状态和物理参数

#### 7.1更新车辆状态

```cpp
void FAckermannController::UpdateVehicleState(const ACarlaWheeledVehicle* Vehicle) {
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
 
  LastVehicleSpeed = VehicleSpeed;
  LastVehicleAcceleration = VehicleAcceleration;
 
  // 更新仿真状态
  DeltaTime = Vehicle->GetWorld()->GetDeltaSeconds();
 
  // 更新车辆状态
  VehicleSteer = Vehicle->GetWheelSteerAngle() * VehicleMaxSteering; 
  //VehicleSteer应从车辆物理状态获取实际转向角，
  VehicleSpeed = Vehicle->GetVehicleForwardSpeed() / 100.0f;  // From cm/s to m/s
  float CurrentAcceleration = (VehicleSpeed - LastVehicleSpeed) / DeltaTime;
  // 对加速度应用平均滤波器。
  VehicleAcceleration = (4.0f*LastVehicleAcceleration + CurrentAcceleration) / 5.0f;//在UpdateVehicleState中，加速度通过低通滤波平滑处理，减少噪声影响。
}
```



#### 7.2更新车辆物理参数

```cpp
void FAckermannController::UpdateVehiclePhysics(const ACarlaWheeledVehicle* Vehicle) {
  VehicleMaxSteering = FMath::DegreesToRadians(Vehicle->GetMaximumSteerAngle());
}
```

## 五、应用场景

- **自动驾驶测试**：构建多样化车辆场景，检验算法的鲁棒性。
- **交通仿真模拟**：模拟真实交通流，优化交通策略。
- **感知与决策训练**：丰富环境中的车辆行为，为感知模型提供多样样本。
- **性能评估**：调节车辆参数测试系统极限。
```
