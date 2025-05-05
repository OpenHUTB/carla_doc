# CARLA 碰撞事件传感器系统（sensor.other.collision）

---

## 1 模块概览

本文档详细讲解 CARLA 模拟器中的一类简单传感器：`sensor.other.collision`，即碰撞事件传感器。该传感器不输出连续数据流，而是在发生事件时触发，用于监测模拟中车辆或行人与其他物体之间的物理接触。通过对其数据结构、序列化机制、客户端接收方式的解析，揭示其在 CARLA 感知系统中的运行机制。

---

## 2 传感器注册与调用原理

CARLA 所有传感器，包括 `sensor.other.collision`，都基于统一的传感器注册机制。碰撞传感器的生命周期由客户端 Python API 控制，但核心的绑定与数据通路如下：

- **蓝图注册阶段**：传感器类型如 `sensor.other.collision` 在 Unreal Engine 蓝图中定义，并注册至 Blueprint Library 中。
- **Actor 创建阶段**：客户端通过 `world.spawn_actor` 方法从蓝图中创建对应的传感器 Actor，并将其附着至车体等实体上。
- **传感器通道建立**：Actor 绑定了底层的 `SensorData` 类型（例如 `CollisionEvent` 派生自 `SensorData`），并通过 `listen()` 方法注册 Python 回调函数。
- **事件触发与回调机制**：
  1. 服务端 C++ 模拟器捕捉到物理碰撞，生成 `CollisionEvent` 数据结构。
  2. 通过 `CollisionEventSerializer` 使用 MsgPack 编码为 `RawData` 数据包。
  3. 通过 RPC 管道将 `RawData` 传输至客户端。
  4. 客户端自动调用对应的 `Deserialize()` 方法还原 `SensorData` 对象。
  5. 最终将对象作为参数传入 `sensor.listen(fn)` 中的 Python 函数。

这一机制实现了“事件 → 序列化 → 网络传输 → 客户端恢复 → 回调”的完整闭环。

---

## 3 数据结构：CollisionEvent

文件：`carla/sensor/data/CollisionEvent.h`

该结构用于表示碰撞时的核心信息，封装了碰撞双方的 Actor 信息和冲击矢量，便于在物理分析、异常事件检测等应用中使用。

```cpp
struct CollisionEvent {
  rpc::Actor self_actor;           // 当前传感器所属对象（如车辆本体）
  rpc::Actor other_actor;          // 碰撞的另一个对象（如行人、路灯）
  geom::Vector3D normal_impulse;   // 碰撞时的冲击力向量（单位 N·s）
};
```

- `self_actor` 与 `other_actor` 提供了 Actor 的 ID 和类型等元信息，便于在客户端快速识别。
- `normal_impulse` 用于衡量碰撞强度，是三维向量，表示冲击的方向和幅值。

---

## 4 序列化器分析：CollisionEventSerializer

文件：`sensor/s11n/CollisionEventSerializer.{h,cpp}`

该类用于将 `CollisionEvent` 数据结构打包为字节流 RawData 并进行网络传输，支持高效的序列化和反序列化。

### 4.1 数据托运结构

```cpp
struct Data {
  rpc::Actor self_actor;
  rpc::Actor other_actor;
  geom::Vector3D normal_impulse;
  MSGPACK_DEFINE_ARRAY(self_actor, other_actor, normal_impulse)
};
```

该结构通过 `MSGPACK_DEFINE_ARRAY` 宏定义，使其具备序列化能力，便于通过 `MsgPack` 编码传输。

### 4.2 序列化（Server 端）

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

- 此函数在服务端运行，将 `CollisionEvent` 信息打包为 `Buffer`，随后通过 RPC 系统发送至客户端。

### 4.3 反序列化（Client 端）

```cpp
static SharedPtr<SensorData> Deserialize(RawData &&data) {
  return SharedPtr<SensorData>(new data::CollisionEvent(std::move(data)));
}
```

- 客户端接收到 `RawData` 后，会通过该方法重构为 `CollisionEvent` 对象并触发回调函数。

> 注：该模块采用轻量级高性能的 `MessagePack` 作为数据序列化方案，编码数据体积远小于 JSON 或 XML，适用于实时仿真系统。

---

## 5 RawData 与 SensorData 概述

在 CARLA 的传感器架构中，所有传感器数据在底层都通过 `RawData` 进行传输，而结构化的数据（如碰撞事件）则会通过 `SensorData` 类进一步封装。这两者的关系如下：

- `RawData`：底层的字节数据容器，封装网络传输数据、帧号、传感器位置等信息。
- `SensorData`：从 `RawData` 派生出的抽象类，表示可由客户端识别的数据类型。

以 `CollisionEvent` 为例，其结构如下：

```cpp
// RawData（基类）
class RawData {
public:
  size_t GetFrame() const;
  double GetTimestamp() const;
  const rpc::Transform &GetSensorTransform() const;
  const uint8_t *begin() const;  // 数据起始位置
  size_t size() const;           // 数据大小
};

// SensorData（结构化数据封装）
class SensorData : public EnableSharedFromThis<SensorData>, private NonCopyable {
protected:
  SensorData(size_t frame, double timestamp, const rpc::Transform &sensor_transform);
};

// CollisionEvent（具体数据结构）
class CollisionEvent : public SensorData {
  // 包含 self_actor, other_actor, normal_impulse
};
```

### 5.1 在序列化器中的角色

- 服务端将 `CollisionEvent` 数据打包进 `RawData` 缓冲区；
- 客户端调用 `Deserialize(RawData &&)`，根据序列化协议反向构造 `SensorData` 对象；
- 最终通过 `sensor.listen()` 回调函数传递给 Python 用户使用。

该结构便于扩展，所有结构化数据传感器（如 IMU、碰撞、GNSS）均基于这一机制实现。

---

## 6 Python API 使用示例

> 注：以下示例假定你已在主程序中定义了 `client`、`vehicle`、`transform` 等变量。

```
# 定义回调函数：当碰撞发生时，将调用该函数处理事件信息
def on_collision(event):
    actor = event.other_actor              # 获取碰撞对方的 Actor 对象
    impulse = event.normal_impulse        # 获取碰撞冲击向量（Vector3D）
    print(f"发生碰撞！对象类型：{actor.type_id}，冲击向量：{impulse}")

# 以下变量需在外部提前定义（通常在主程序中）：
# client = carla.Client('localhost', 2000)  # 连接模拟器
# world = client.get_world()  # 获取当前 CARLA 世界对象
# blueprint_library = world.get_blueprint_library()  # 获取蓝图库
# transform = carla.Transform(...)  # 定义传感器相对位置
# vehicle = world.spawn_actor(...)  # 目标车辆对象

# 从蓝图库中检索碰撞传感器蓝图（类型为 sensor.other.collision）
bp = blueprint_library.find('sensor.other.collision')

# 生成传感器对象并附着到已有车辆上
sensor = world.spawn_actor(bp, transform, attach_to=vehicle)

# 注册监听函数，回调将持续监听碰撞事件
sensor.listen(on_collision)
```

- `on_collision` 是监听器函数，在碰撞事件到达时执行。
- 可进一步拓展为：记录日志、分析速度/角度、触发避障逻辑等。

---

## 7 文件分布

| 文件                                | 位置             | 功能描述               |
| --------------------------------- | -------------- | ------------------ |
| `CollisionEvent.h`                | `sensor/data/` | 定义碰撞事件数据结构         |
| `CollisionEventSerializer.h/.cpp` | `sensor/s11n/` | 编码/解码事件数据为 RawData |
| `RawData.h`                       | `sensor/`      | 抽象数据包装基类，用于数据收发    |
| `Python 示例`                       | 用户自定义脚本        | 调用传感器、监听回调、打印信息等   |

---

## 8 小结与拓展

- `sensor.other.collision` 是 CARLA 中最轻量级的事件传感器之一，仅在物理碰撞时触发。
- 序列化逻辑采用 MsgPack + 自定义数据结构，传输高效、实现简洁。
- 事件回调机制适合用于行为分析、数据驱动建模、仿真故障检测等场景。
- 可扩展场景包括：
  - 联合 IMU/GNSS 分析事故发生前后的运动状态
  - 结合图像数据回溯碰撞现场
  - 搭配控制模块生成碰撞规避策略
  - 增加时间戳、碰撞位置、速度向量等字段，实现更精细建模



---

