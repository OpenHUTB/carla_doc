## 📑 目录（Table of Contents）

### 第一章：碰撞事件传感器（sensor.other.collision）

- [第一章：CARLA 碰撞事件传感器系统（sensor.other.collision）](#第一章carla-碰撞事件传感器系统sensorothercollision)
  - [1 模块概览](#1-模块概览)
  - [2 传感器注册与调用原理](#2-传感器注册与调用原理)
  - [3 数据结构：CollisionEvent](#3-数据结构collisionevent)
  - [4 序列化器分析](#4-序列化器分析collisioneventserializer)
  - [5 RawData 与 SensorData 概述](#5-rawdata-与-sensordata-概述)
  - [6 Python API 使用示例](#6-python-api-使用示例)
  - [7 小结与拓展](#7-小结与拓展)

### 第二章：空传感器（sensor.other.noop）

- [第二章：CARLA 空传感器系统（sensor.other.noop）](#第二章carla-空传感器系统sensorothernoop)
  - [1 模块概览](#1-模块概览-1)
  - [2 传感器注册与调用原理](#2-传感器注册与调用原理-1)
  - [3 序列化器分析](#3-序列化器分析noopserializer)
  - [4 数据结构与传输机制](#4-数据结构与传输机制)
  - [5 Python API 使用示例](#5-python-api-使用示例-1)
  - [6 小结与拓展](#6-小结与拓展-1)

### 第三章：IMU 传感器（sensor.other.imu）

- [第三章：CARLA惯性测量单元传感器系统（sensor.other.imu）](#第三章carla惯性测量单元传感器系统sensorotherimu)
  - [1 模块概览](#1-模块概览-2)
  - [2 工作流程与序列化机制](#2-工作流程与序列化机制)
  - [3 数据结构详解](#3-数据结构详解)
  - [4 Python API 调用示例](#4-python-api-调用示例)
  - [5 拓展用例与集成应用](#5-拓展用例与集成应用)
  - [6 限制与对比分析](#6-限制与对比分析)
  - [7 总结与建议](#7-总结与建议-1)

### 第四章：GNSS 传感器（sensor.other.gnss）

- [第四章：CARLA 全球导航卫星系统传感器（sensor.other.gnss）](#第四章carla-全球导航卫星系统传感器sensorothergnss)
  - [1 模块概览](#1-模块概览-3)
  - [2 工作流程与传输机制](#2-工作流程与传输机制)
  - [3 数据结构解析](#3-数据结构解析)
  - [4 序列化机制分析](#4-序列化机制分析)
  - [5 Python API 使用示例](#5-python-api-使用示例-2)
  - [6 应用拓展与组合使用](#6-应用拓展与组合使用)
  - [7 总结与建议](#7-总结与建议-2)


---

# 第一章：CARLA 碰撞事件传感器系统（sensor.other.collision）

---

## 1 模块概览

 ![flowchart_1.png](..%2Fimg%2Fmodules%2Fflowchart_1.png)

本章详细讲解 CARLA 模拟器中的一类简单传感器：`sensor.other.collision`，即碰撞事件传感器。该传感器不输出连续的数据流信息，而是在检测到实体碰撞时以事件形式即时触发。其设计初衷是为用户提供车辆与环境中其他对象之间物理交互的监控能力。

在自动驾驶仿真、强化学习安全评估或碰撞规避算法验证中，`sensor.other.collision` 提供了关键的反馈信号。通过对其数据结构、序列化机制与客户端回调触发流程的深入解析，可全面理解该传感器在 CARLA 感知系统中的运行机制及其工程实现。
    
---

## 2 传感器注册与调用原理

CARLA 所有传感器，包括 `sensor.other.collision`，均基于统一的 Actor 注册与绑定框架。其创建与工作流程可分为以下几个核心阶段：

- **蓝图注册阶段**：CARLA 所有传感器类型在 Unreal Engine 的蓝图系统中定义，供客户端调用。
- **Actor 创建阶段**：Python 客户端通过 `world.spawn_actor()` 创建 `"sensor.other.collision"` 实例。
- **传感器通道建立**：Actor 自动绑定底层的 `SensorData` 数据类型，用户通过 `.listen()` 注册事件处理函数。
- **事件触发与传输机制**：
  1. 服务端生成 `CollisionEvent` 事件（Bullet 引擎）
  2. 使用 [`CollisionEventSerializer`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/s11n/CollisionEventSerializer.h) 编码为 `RawData`
  3. 通过 RPC 网络发送
  4. 客户端使用 `Deserialize()` 解码为 `SensorData`
  5. 注入回调函数中进行处理
该流程构成事件捕捉 → 数据序列化 → 网络传输 → 解码还原 → 客户端处理的完整通路。

---

## 3 数据结构：CollisionEvent

定义文件：[`carla/sensor/data/CollisionEvent.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/data/CollisionEvent.h)

```cpp
struct CollisionEvent {
  rpc::Actor self_actor;
  rpc::Actor other_actor;
  geom::Vector3D normal_impulse;
};
```

- `self_actor`：传感器附着实体（如车辆）
- `other_actor`：碰撞对象（如障碍物、NPC）
- `normal_impulse`：冲击力矢量，反映碰撞强度与方向

---

## 4 序列化器分析：CollisionEventSerializer

定义文件：  
[`CollisionEventSerializer.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/s11n/CollisionEventSerializer.h)  
[`CollisionEventSerializer.cpp`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/s11n/CollisionEventSerializer.cpp)

```cpp
struct Data {
  rpc::Actor self_actor;
  rpc::Actor other_actor;
  geom::Vector3D normal_impulse;
  MSGPACK_DEFINE_ARRAY(self_actor, other_actor, normal_impulse)
};
```

```cpp
template <typename SensorT>
static Buffer Serialize(
    const SensorT &,
    rpc::Actor self_actor,
    rpc::Actor other_actor,
    geom::Vector3D normal_impulse) {
  return MsgPack::Pack(Data{self_actor, other_actor, normal_impulse});
}
```

```cpp
static SharedPtr<SensorData> Deserialize(RawData &&data) {
  return SharedPtr<SensorData>(new data::CollisionEvent(std::move(data)));
}
```

---

## 5 RawData 与 SensorData 概述

数据结构定义：  
[`RawData.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/RawData.h)  
[`SensorData.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/SensorData.h)

```cpp
class RawData {
public:
  size_t GetFrame() const;
  double GetTimestamp() const;
  const rpc::Transform &GetSensorTransform() const;
  const uint8_t *begin() const;
  size_t size() const;
};

class SensorData : public EnableSharedFromThis<SensorData>, private NonCopyable {
protected:
  SensorData(size_t frame, double timestamp, const rpc::Transform &sensor_transform);
};
```

---

## 6 Python API 使用示例

参考文件：[`collision_sensor.py`](https://github.com/OpenHUTB/carla_doc/blob/dev/src/examples/collision_sensor.py)

```
def on_collision(event):
    actor = event.other_actor
    impulse = event.normal_impulse
    print(f"发生碰撞！对象类型：{actor.type_id}，冲击向量：{impulse}")
```

```
blueprint_library = world.get_blueprint_library()
bp = blueprint_library.find('sensor.other.collision')
transform = carla.Transform(carla.Location(x=0, y=0, z=2.0))
sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
sensor.listen(on_collision)
```

---

## 7 小结与拓展

- `sensor.other.collision` 是轻量、事件驱动的传感器
- 使用 `MsgPack + RawData` 组合，编码效率高
- 常用于：碰撞检测、事故分析、惩罚信号生成

### 可拓展方向：

- 增加字段（如速度向量、碰撞位置）
- 结合图像、GNSS 数据实现回溯系统
- 与强化学习结合生成动态规避策略

---

# 第二章：CARLA 空传感器系统（sensor.other.noop）

---

## 1 模块概览

 ![flowchart_2.png](..%2Fimg%2Fmodules%2Fflowchart_2.png)

本章介绍 CARLA 模拟器中最为简化的一类传感器：`sensor.other.noop`，即“无操作传感器”（No-Operation Sensor）。顾名思义，该传感器不会向客户端发送任何数据，其存在的意义更多是作为客户端挂载传感器的占位符、功能测试器或新传感器开发的最小模板。

`sensor.other.noop` 适用于以下场景：

- 构建新型传感器时的起始模板；
- 用于网络与回调机制调试，无需实际数据；
- 占位传感器（需要绑定但暂时不产出数据）；
- 模拟传感器失败或无响应场景下的系统行为。

其核心特征是：**不参与数据流的任何阶段**，服务端不会推送数据，客户端接收到数据则立即报错。

---

## 2 传感器注册与调用原理

`sensor.other.noop` 遵循 CARLA 所有传感器统一的注册、挂载、通信框架，其调用流程包括以下几个阶段：

- **蓝图注册阶段**：该传感器在 Unreal Engine 蓝图系统中以 `"sensor.other.noop"` 注册，蓝图库中包含其类型定义。
- **Actor 创建阶段**：客户端通过 `world.spawn_actor()` 创建该传感器实例，传入 `"sensor.other.noop"` 字符串作为蓝图 ID，实例可附着至车辆或动态对象。
- **传感器监听绑定**：
  - 可通过 `.listen()` 注册 Python 回调函数；
  - 但由于此传感器不会生成任何事件，回调函数永远不会被实际调用；
  - 若数据进入系统，将由序列化器直接拒绝并抛出异常。

---

## 3 序列化器分析：NoopSerializer

该序列化器专为“客户端不接收数据”的传感器设计，核心逻辑是禁止任何数据反序列化。

### 3.1 接口定义

```cpp
/// Dummy serializer that blocks all the data.
class NoopSerializer {
public:
  [[noreturn]] static SharedPtr<SensorData> Deserialize(RawData &&data);
};
```

说明：

- `[[noreturn]]` 是 C++11 标准修饰符，表示该函数永不返回；
- 调用该函数将抛出异常，标志该传感器设计为“不可反序列化”。

### 3.2 函数实现

```cpp
SharedPtr<SensorData> NoopSerializer::Deserialize(RawData &&) {
  throw_exception(std::runtime_error("NoopSerializer: Invalid data received."));
}
```

说明：

- 使用 CARLA 提供的 `throw_exception` 工具函数；
- 一旦客户端尝试处理此类型数据，即触发中断错误；
- 该机制确保 `sensor.other.noop` 无法被误用。

---

## 4 数据结构与传输机制

该传感器不绑定任何结构化数据类型：

- 不包含自定义数据结构（如 `NoopEvent`）；
- 不通过 `RawData` 传输任何内容；
- 无有效 `SensorData` 派生类实例生成；
- 虽然支持 `.listen()` 注册机制，但永不触发。

因此，`sensor.other.noop` 是一种“存在但不发声”的传感器，仅用于挂载占位或调试目的。

---

## 5 Python API 使用示例

虽然该传感器不传输数据，仍可正常在 Python 中创建与注册监听。

```
# 定义回调函数（注意：不会被调用）
def on_noop_event(event):
    print("此处不应出现任何输出")

# 获取蓝图库并查找蓝图
bp = world.get_blueprint_library().find('sensor.other.noop')

# 设置传感器相对位置
transform = carla.Transform(carla.Location(x=0, y=0, z=2.0))

# 创建传感器并附着至车辆
noop_sensor = world.spawn_actor(bp, transform, attach_to=vehicle)

# 注册监听函数（将不会触发）
noop_sensor.listen(on_noop_event)
```

**可选调试建议**：

- 可用日志验证传感器挂载成功；
- 可调用 `.destroy()` 观察销毁逻辑；
- 可组合多个传感器观察 `.listen()` 机制行为。

---

## 6 小结与拓展

- `sensor.other.noop` 是 CARLA 中结构最简、用途最特殊的一类传感器；
- 其核心设计目的是：禁止数据流通，仅做占位和调试；
- 非常适合作为自定义传感器的开发模板或序列化链测试工具。

### 拓展建议：

- 作为新事件传感器的起点模板；
- 用于测试 `.listen()` 和 `.destroy()` 回调流程；
- 模拟传感器故障、异常或丢包处理机制；
- 可组合图像、IMU 等实际传感器测试是否发生调用竞争或系统冲突。


---

# 第三章：CARLA惯性测量单元传感器系统（sensor.other.imu）

---

## 1 模块概览

 ![flowchart_3.png](..%2Fimg%2Fmodules%2Fflowchart_3.png)

`sensor.other.imu` 是 CARLA 提供的惯性测量单元（IMU）传感器，用于捕捉车辆的三维加速度、角速度及方向信息，广泛应用于轨迹估计、姿态解算、导航融合等自动驾驶仿真任务中。

该传感器会周期性发送事件，传递以下数据：

* **加速度计**：三轴加速度 `Vector3D(x, y, z)`，单位 m/s²
* **陀螺仪**：三轴角速度 `Vector3D(x, y, z)`，单位 rad/s
* **罗盘**：航向角（方向朝向），单位弧度

默认以 20Hz 更新频率输出，支持在蓝图中调整采样速率。

---

## 2 工作流程与序列化机制

IMU 传感器的数据流程如下：

1. **事件捕获**：模拟器服务端采集当前帧的 IMU 数据
2. **数据序列化**：使用 `IMUSerializer` 将加速度、角速度与罗盘值打包为 MsgPack 编码字节流（参见 `IMUSerializer.h/.cpp`）
3. **网络传输**：打包后的 `RawData` 通过 RPC 系统传输至客户端
4. **数据还原**：客户端通过 `IMUMeasurement` 类将字节流还原为结构化对象
5. **回调触发**：Python 层注册的 `.listen()` 回调函数获得该事件并执行处理逻辑


---

## 3 数据结构详解

IMU 数据通过 `IMUMeasurement` 类对外暴露，定义位置：

📄 [`carla/sensor/data/IMUMeasurement.h`](https://github.com/carla-simulator/carla/blob/dev/LibCarla/source/carla/sensor/data/IMUMeasurement.h)

该类封装如下：

```
class IMUMeasurement : public SensorData {
public:
  geom::Vector3D GetAccelerometer() const;
  geom::Vector3D GetGyroscope() const;
  float GetCompass() const;
};
```

* **加速度**：IMU 加速度计测得的三轴值
* **角速度**：陀螺仪角速度
* **罗盘值**：航向角（通常为 0\~2π 的弧度）

所有字段通过 `IMUSerializer::Deserialize()` 方法从 `RawData` 还原，序列化结构如下：

```
struct IMUData {
  Vector3D accelerometer;
  Vector3D gyroscope;
  float compass;
};
```

通过 `MSGPACK_DEFINE_ARRAY(acc, gyro, compass)` 支持 MsgPack 自动编码。

---

## 4 Python API 调用示例

以下代码展示如何部署 IMU 传感器、注册监听器，并处理接收到的数据：

```
def on_imu(event):
    acc = event.accelerometer
    gyro = event.gyroscope
    compass = event.compass
    print(f"[IMU] 加速度: {acc} | 角速度: {gyro} | 罗盘: {compass}")

# 创建传感器
bp = world.get_blueprint_library().find('sensor.other.imu')
transform = carla.Transform(carla.Location(x=0, y=0, z=1.0))
imu_sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
imu_sensor.listen(on_imu)
```

---

## 5 拓展用例与集成应用

IMU 常与 GNSS、相机、雷达等传感器联用，实现高级状态估计：

**与 GNSS 融合：**

```
def on_gnss(gnss):
    print(f"[GNSS] 纬度: {gnss.latitude}, 经度: {gnss.longitude}")

imu_sensor.listen(on_imu)
gnss_sensor.listen(on_gnss)
```

**估算车辆速度（积分法，简化示例）：**

```
velocity = carla.Vector3D()

def on_imu_integration(imu):
    global velocity
    dt = 0.05  # 默认 20Hz
    a = imu.accelerometer
    velocity.x += a.x * dt
    velocity.y += a.y * dt
    velocity.z += a.z * dt
    print(f"估算速度: {velocity}")
```

---

## 6 限制与对比分析

| 属性   | CARLA IMU  | 真实 IMU（如 Xsens） |
| ---- | ---------- | --------------- |
| 更新频率 | 默认 20Hz，可调 | 最高 > 1000Hz     |
| 噪声建模 | 默认无噪声      | 含偏置、漂移等随机扰动     |
| 时间同步 | 自动与仿真对齐    | 需硬件级时间戳         |
| 数据输出 | 无 CSV 导出   | 支持标准格式          |

---

## 7 总结与建议

* `sensor.other.imu` 提供结构清晰、易用的姿态/运动状态反馈能力；
* 可拓展为带噪模型、加权滤波、数据融合平台；
* 建议与 `sensor.other.gnss`、`sensor.camera.rgb` 联合使用，支持轨迹分析、三维重建等研究任务；
* 后续可引入 IMU 噪声模拟模块、数据导出功能。


---

# 第四章：CARLA 全球导航卫星系统传感器（sensor.other.gnss）

---

## 1 模块概览

 ![flowchart_4.png](..%2Fimg%2Fmodules%2Fflowchart_4.png)

`sensor.other.gnss` 是 CARLA 中用于获取地理位置信息的传感器，模拟真实世界中的全球导航卫星系统（GNSS）设备，如 GPS、北斗等。传感器输出包括：

* **经度（longitude）**：单位°，范围 $-180°, 180°$
* **纬度（latitude）**：单位°，范围 $-90°, 90°$
* **海拔（altitude）**：单位 m，代表相对于海平面的高度

该传感器可用于场景定位、路径重建、轨迹分析等仿真任务中，常与 IMU、地图匹配模块联合使用。

---

## 2 工作流程与传输机制

GNSS 的数据传输流程如下：

1. **事件生成**：服务端在当前仿真帧中生成地理位置信息
2. **数据打包**：通过 `GnssSerializer` 将 `GeoLocation` 序列化为 MsgPack 格式的 `RawData`
3. **网络传输**：打包后的字节流通过 CARLA 的 RPC 系统发送给客户端
4. **数据还原**：客户端使用 `GnssMeasurement` 类解码 `RawData`，得到结构化数据
5. **Python 层监听**：`.listen()` 接口注册的函数在每次更新中触发回调


---

## 3 数据结构解析

定义位置：[`GnssMeasurement.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/data/GnssMeasurement.h)

```cpp
class GnssMeasurement : public SensorData {
public:
  geom::GeoLocation GetGeoLocation() const;
  double GetLongitude() const;
  double GetLatitude() const;
  double GetAltitude() const;
};
```

其中：

* `GeoLocation` 结构体封装了 `{ longitude, latitude, altitude }`
* 每帧更新一次，数据来源为服务端环境模拟的车辆真实位置信息

默认输出单位为：

| 字段        | 类型     | 单位   |
| --------- | ------ | ---- |
| Longitude | double | 度（°） |
| Latitude  | double | 度（°） |
| Altitude  | double | 米（m） |

---

## 4 序列化机制分析

定义文件：
[`GnssSerializer.h`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/s11n/GnssSerializer.h)
[`GnssSerializer.cpp`](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/sensor/s11n/GnssSerializer.cpp)

```cpp
// 客户端解码逻辑
SharedPtr<SensorData> GnssSerializer::Deserialize(RawData &&data) {
  return SharedPtr<SensorData>(new data::GnssMeasurement(std::move(data)));
}
```

序列化结构体使用 `MSGPACK_DEFINE_ARRAY` 自动完成：

```cpp
struct GeoLocation {
  double latitude;
  double longitude;
  double altitude;
  MSGPACK_DEFINE_ARRAY(latitude, longitude, altitude)
};
```

该机制保证 GNSS 数据体积小、解码快，适合高频传输与跨语言兼容。

---

## 5 Python API 使用示例

```python
# 定义回调函数
def on_gnss(event):
    lat = event.latitude
    lon = event.longitude
    alt = event.altitude
    print(f"[GNSS] 纬度: {lat:.6f}, 经度: {lon:.6f}, 海拔: {alt:.2f}m")
```

```python
# 创建并绑定传感器
bp = world.get_blueprint_library().find('sensor.other.gnss')
transform = carla.Transform(carla.Location(x=0.7, y=0.0, z=1.6))
gnss_sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
gnss_sensor.listen(on_gnss)
```

可选蓝图参数（`bp.set_attribute(...)`）包括：

| 属性            | 描述         | 默认值    |
| ------------- | ---------- | ------ |
| `sensor_tick` | 传感器更新时间（秒） | `0.05` |

---

## 6 应用拓展与组合使用

GNSS 传感器常用于以下场景：

* **路径重建**：记录车辆运行轨迹用于回放
* **定位精度验证**：与地图匹配算法结合测试鲁棒性
* **IMU + GNSS 融合**：实现基于扩展卡尔曼滤波（EKF）的定位

### 示例：GNSS 与 IMU 联合使用

```python
def on_fused_data(gnss, imu):
    location = (gnss.latitude, gnss.longitude)
    acceleration = imu.accelerometer
    print(f"[FUSION] 坐标: {location}, 加速度: {acceleration}")
```

该模式可用于轨迹重建、地理围栏检测、自动驾驶行为控制等任务。

---

## 7 总结与建议

* `sensor.other.gnss` 提供高精度地理位置信息，是自动驾驶仿真系统中的关键模块；
* 与 IMU、摄像头、地图等配合可实现高级 SLAM 与定位算法验证；
* 若需实现更真实的模型，可拓展添加噪声模型、地形遮挡、信号丢失模拟等功能。

---

