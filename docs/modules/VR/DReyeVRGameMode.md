# 游戏模式

## 模块概述
DReyeVRGameMode 是 DReyeVR 扩展模块的核心游戏模式类，负责管理游戏的核心逻辑，包括玩家控制、EgoVehicle（主驾驶车辆）、重播系统、音效控制和调试可视化。该模块基于 Unreal Engine 的 ACarlaGameModeBase 扩展，集成 Carla 仿真环境与 DReyeVR 的自定义功能。

## 类结构
ADReyeVRGameMode
继承自 ACarlaGameModeBase，负责游戏初始化、输入处理、EgoVehicle 管理及重播控制。

| 变量名         | 类型                                | 描述  |
|-------------|-----------------------------------|-----|
| Player | TWeakObjectPtr<APlayerController> |  玩家控制器弱引用 |
| DReyeVR_Pawn | TWeakObjectPtr<ADReyeVRPawn> |  DReyeVR 主视角 Pawn 弱引用 |
| EgoVehiclePtr | TWeakObjectPtr<AEgoVehicle> |  EgoVehicle 弱引用 |
| SpectatorPtr | TWeakObjectPtr<APawn> |  观察者视角 Pawn 弱引用 |
| bDoSpawnEgoVehicleTransform | bool |  是否使用预设生成点生成 EgoVehicle |
| SpawnEgoVehicleTransform | FTransform |  预设生成点变换 |
| ReplayTimeFactor | double |  重播时间缩放因子（1.0x 为实时） |
| bReplaySync | bool |  是否启用同步重播模式 |

## 成员函数

| 函数名               | 描述                                      |
|------------------|-----------------------------------------|
| BeginPlay()      | 初始化玩家、输入组件、DReyeVR Pawn 和 EgoVehicle                              |
| Tick(float DeltaSeconds)      | 处理重播控制、调试绘制和输入更新                              |
| SetupPlayerInputComponent()      | 绑定玩家输入动作（如切换摄像机、控制 EgoVehicle）                             |
| SetupDReyeVRPawn()      | 生成或获取 DReyeVR 主视角 Pawn              |
| SetupEgoVehicle()      | 生成或获取 EgoVehicle 实例              |
| SetupSpectator()      | 初始化观察者视角 Pawn              |
| ReplayPlayPause()      | 切换重播暂停/播放状态              |
| ReplaySpeedUp()      | 增加重播速度              |
| ReplaySlowDown()      | 降低重播速度              |
| DrawBBoxes()      | 绘制场景中所有车辆的调试边界框              |

## 核心功能
1. 游戏初始化

    * 玩家与输入绑定

    在 BeginPlay() 中初始化玩家控制器，绑定输入动作（如 PossessEgoVehicle、ReplayPlayPause）。

    * DReyeVR Pawn 生成

    通过 SetupDReyeVRPawn() 生成主视角 Pawn，并自动将其分配给玩家控制器。

    * EgoVehicle 管理

    根据配置参数 AutomaticallySpawnEgo 决定是否生成 EgoVehicle，默认使用 TeslaM3 配置。

2. 输入处理

* 摄像机切换

  支持通过按键切换主视角（驾驶员视角、第三人称视角）和观察者视角。

* EgoVehicle 控制

  * 键盘/手柄输入直接控制车辆（油门、刹车、转向）。
  * 支持通过 PossessEgoVehicle 和 PossessSpectator 切换控制权。


3. 重播系统

* 重播控制

    通过 ReplayPlayPause()、ReplaySpeedUp() 等函数控制重播进度，支持时间缩放（0.0x 暂停至 4.0x 加速）。

* 同步模式

    同步模式下禁用插值，确保重播与原始录制帧率一致。

4. 音效管理

* 动态音量调整

    根据配置参数调整 EgoVehicle、非 Ego 车辆和环境音效的音量，例如：
    ```cpp
    ACarlaWheeledVehicle::Volume = NonEgoVolumePercent / 100.f;
    ```

5. 调试可视化

* 边界框绘制

    在编辑器模式下绘制所有车辆的调试边界框，便于观察物体位置：
    ```cpp
    void ADReyeVRGameMode::DrawBBoxes() {
        // 使用 FDebugDraw 绘制包围盒
    }
    ```


## 配置参数
通过 GeneralParams 读取 DReyeVRConfig.ini 配置文件，关键参数包括：
```yaml
[Sound]
EgoVolumePercent=80
NonEgoVolumePercent=50

[Game]
DoSpawnEgoVehicleTransform=true
SpawnEgoVehicleTransform=(X=0,Y=0,Z=0)

[Replayer]
UseCarlaSpectator=false
ReplayInterpolation=true
```

## 使用示例
```yaml
// 在 BeginPlay 中初始化游戏模式
void ADReyeVRGameMode::BeginPlay() {
    Super::BeginPlay();
    SetupPlayerInputComponent();
    SetupDReyeVRPawn();
    if (GeneralParams.Get<bool>("Game", "AutomaticallySpawnEgo")) {
        SetupEgoVehicle();
    }
}

// 绑定输入动作
void ADReyeVRGameMode::SetupPlayerInputComponent() {
    InputComponent->BindAction("ReplayPlayPause", IE_Pressed, this, &ADReyeVRGameMode::ReplayPlayPause);
}
```


## 依赖模块
* **Carla Core**：依赖 Carla 的 ACarlaGameModeBase 和重播系统。
* **DReyeVR Extensions**：集成 EgoVehicle、传感器和自定义输入处理。
* **Unreal Engine**：使用内置的 APlayerController 和 UInputComponent。


## 扩展建议
1. 自定义车辆支持

    在 DReyeVRConfig.ini 中添加新车辆配置，扩展 VehicleTypes 数组。
2. 重播增强

    添加时间戳同步功能，支持精确跳转到特定时间点。

3. 调试工具

    集成 DrawDebugString 显示实时车辆状态（速度、方向）。


