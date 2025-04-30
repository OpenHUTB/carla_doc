
# CARLA Recorder 模块说明文档
---

## 📑 目录

1. [模块概述](#模块概述)  
2. [主要类与职责](#主要类与职责)  
3. [功能说明](#功能说明)  
   - [初始化](#初始化)  
   - [录像控制](#录像控制)  
   - [回放控制](#回放控制)  
   - [查询功能](#查询功能)  
4. [关键数据结构](#关键数据结构)  
5. [依赖模块](#依赖模块)  
6. [附录：包结构](#附录包结构)

---

## 模块概述

CarlaRecorder 模块主要负责 CARLA 中仿真数据的录制与回放操作。包括对车辆、行人、光照、碰撞等实体状态的记录、存储与回放控制。其实现涉及数据包编码、查询分析、时间控制等多个子模块。

---

## 主要类与职责

| 类名 | 文件 | 说明 |
|------|------|------|
| `ACarlaRecorder` | `CarlaRecorder.h / CarlaRecorder.cpp` | 主要控制类，继承自 Unreal 的 `AActor`，提供录像/回放接口。|
| `CarlaReplayer` | `CarlaReplayer.h` | 实现回放控制，包括时间倍率、忽略特定角色等设置。|
| `CarlaRecorderQuery` | `CarlaRecorderQuery.h` | 用于分析已录制文件的数据，如碰撞信息、阻塞信息等。|

---

## 功能说明

### 初始化

```cpp
ACarlaRecorder::ACarlaRecorder()
```
- 设置 TickGroup 为 `TG_PrePhysics`
- 默认禁用录像功能

### 录像控制

```cpp
void Enable()
void Disable()
void Ticking(float DeltaSeconds)
```

- `Ticking()` 方法被定期调用以采集仿真状态
- 使用 `PlatformTime` 与 `VisualTime` 跟踪物理时间与视觉时间
- 与 `CarlaEpisode` 和 `FActorRegistry` 联动获取仿真环境信息

### 回放控制

```cpp
std::string ReplayFile(std::string Name, double TimeStart, double Duration, uint32_t FollowId, bool ReplaySensors)
void SetReplayerTimeFactor(double TimeFactor)
void SetReplayerIgnoreHero(bool IgnoreHero)
void SetReplayerIgnoreSpectator(bool IgnoreSpectator)
void StopReplayer(bool KeepActors)
```

- 通过 `ReplayFile()` 加载文件并控制回放参数
- 支持设置时间倍率与忽略特定角色
- `StopReplayer()` 控制是否保留回放后生成的角色对象

### 查询功能

```cpp
std::string ShowFileInfo(std::string Name, bool bShowAll)
std::string ShowFileCollisions(std::string Name, char Type1, char Type2)
std::string ShowFileActorsBlocked(std::string Name, double MinTime, double MinDistance)
```

- `QueryInfo()`：查看录制文件的基础信息  
- `QueryCollisions()`：查询特定对象类型之间的碰撞事件  
- `QueryBlocked()`：查找因距离/时间被阻挡的实体

---

## 关键数据结构

| 结构体 | 描述 |
|--------|------|
| `CarlaRecorderPlatformTime` | 记录平台物理时间信息，用于时间对齐 |
| `CarlaRecorderVisualTime` | 存储视觉时间戳，与画面帧同步 |
| `CarlaRecorderPacketId` | 枚举各类数据包标识，如 FrameStart、Collision、EventAdd 等 |

---

## 依赖模块

- `Carla/Actor/ActorDescription.h`
- `Carla/Vehicle/CarlaWheeledVehicle.h`
- `Carla/Lights/CarlaLight.h`
- `Carla/Traffic/TrafficLightBase.h`
- `Carla/Walker/WalkerControl.h`
- `Carla/Game/CarlaEpisode.h`
- `CarlaRecorderAnimVehicle`、`CarlaRecorderCollision` 等子模块（用于细粒度记录）

---