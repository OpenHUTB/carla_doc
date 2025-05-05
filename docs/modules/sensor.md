---

# CARLA 碰撞事件传感器系统（sensor.other.collision）

---

## 1 模块概览

本文档详细讲解 CARLA 模拟器中的一类简单传感器：`sensor.other.collision`，即碰撞事件传感器。该传感器不输出连续的数据流信息，而是在检测到实体碰撞时以事件形式即时触发。其设计初衷是为用户提供车辆与环境中其他对象之间物理交互的监控能力。

在自动驾驶仿真、强化学习安全评估或碰撞规避算法验证中，`sensor.other.collision` 提供了关键的反馈信号。通过对其数据结构、序列化机制与客户端回调触发流程的深入解析，可全面理解该传感器在 CARLA 感知系统中的运行机制及其工程实现。

---

## 2 传感器注册与调用原理

CARLA 所有传感器，包括 `sensor.other.collision`，均基于统一的 Actor 注册与绑定框架。其创建与工作流程可分为以下几个核心阶段：

* **蓝图注册阶段**：CARLA 所有传感器类型在 Unreal Engine 的蓝图系统中定义，开发者通过编辑蓝图将传感器功能挂载至引擎内部的 Blueprint Library，供客户端后续调用。

* **Actor 创建阶段**：在 Python 客户端中，用户通过 `world.spawn_actor()` 接口以字符串 `"sensor.other.collision"` 作为蓝图 ID，创建传感器实例。通常该实例会附着至车体、行人或任意动态对象上。

* **传感器通道建立**：创建成功后，传感器 Actor 内部会自动绑定底层的 C++ 数据类型 `SensorData`，用于承载具体传感器输出数据。开发者需通过 `.listen()` 方法传入 Python 回调函数，用于接收和处理传感器事件。

* **事件触发与回调机制**：

  1. 服务端检测到碰撞事件（通常由 Bullet 物理引擎触发）后，生成一个 `CollisionEvent` 数据对象；
  2. 使用 `CollisionEventSerializer` 序列化该事件为 MsgPack 格式的 `RawData`；
  3. `RawData` 被压缩并通过 CARLA 的 RPC 网络模块发送至客户端；
  4. 客户端接收到数据后，调用 `Deserialize()` 方法还原为 `SensorData` 派生类实例；
  5. 该实例最终作为参数传入 Python 中的回调函数，实现完整的闭环响应流程。

该机制实现了高度解耦的“事件捕获 → 数据编码 → 网络传输 → 数据还原 → 用户逻辑处理”链路，具有良好的可拓展性与实时性。

---

## 3 数据结构：CollisionEvent

定义位置：`carla/sensor/data/CollisionEvent.h`

该结构体用于封装一次碰撞事件中涉及的核心物理信息，包含了事件发生时涉及的两个 Actor 以及碰撞时的冲击力矢量。该信息可用于事后分析、行为回放、异常检测或损伤评估等场景。

```cpp
struct CollisionEvent {
  rpc::Actor self_actor;           // 当前传感器绑定的主对象（通常为车辆）
  rpc::Actor other_actor;          // 与主对象发生碰撞的对方对象
  geom::Vector3D normal_impulse;   // 碰撞产生的冲击力矢量，单位为 N·s（牛·秒）
};
```

* `self_actor`：通常为车辆本体，提供 Actor ID、类别等元信息；
* `other_actor`：可为任意 CARLA 世界中的实体，如行人、静态物体等；
* `normal_impulse`：三维冲击力矢量，反映碰撞的强度和方向，是判断碰撞严重性的重要依据。

该数据结构在事件级传感器设计中结构精简、信息明确，适用于高频低冗余的数据通信场景。

---

## 4 序列化器分析：CollisionEventSerializer

定义文件：`sensor/s11n/CollisionEventSerializer.{h, cpp}`

该类封装了将 `CollisionEvent` 数据结构编码为字节流的逻辑，核心任务是在服务端将结构化数据转为 MsgPack 编码格式，并在客户端恢复为原始结构，用于触发 Python 层的回调函数。

### 4.1 数据托运结构定义

```cpp
struct Data {
  rpc::Actor self_actor;
  rpc::Actor other_actor;
  geom::Vector3D normal_impulse;
  MSGPACK_DEFINE_ARRAY(self_actor, other_actor, normal_impulse)
};
```

* 该结构通过 `MSGPACK_DEFINE_ARRAY` 宏注册为 MsgPack 的编码目标，具备自动序列化功能；
* 所有字段均保持与 `CollisionEvent` 同步，确保事件数据在网络传输过程中的一致性与完整性。

### 4.2 序列化函数（服务端）

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

* 将结构化的 `CollisionEvent` 转换为一个 `Buffer` 缓冲区，用于发送；
* 支持泛型传感器模板，可复用于多个事件类型。

### 4.3 反序列化函数（客户端）

```cpp
static SharedPtr<SensorData> Deserialize(RawData &&data) {
  return SharedPtr<SensorData>(new data::CollisionEvent(std::move(data)));
}
```

* 客户端调用该函数，将 `RawData` 解码为 C++ 层的 `CollisionEvent` 对象；
* 自动触发上层 Python 的 `.listen()` 注册函数。

>  注：CARLA 使用的 `MessagePack` 是一种高效的二进制序列化格式，具有体积小、编码快、跨语言等优势，尤其适用于高频传感器数据的传输。

---

## 5 RawData 与 SensorData 概述

CARLA 模拟器中，所有传感器数据的底层表示均基于 `RawData` 对象，它是传输层统一的字节容器。而结构化信息则通过继承自 `SensorData` 的子类表示。

```cpp
// 底层传输容器
class RawData {
public:
  size_t GetFrame() const;                    // 获取仿真帧编号
  double GetTimestamp() const;                // 获取仿真时间戳
  const rpc::Transform &GetSensorTransform() const; // 传感器在世界坐标系中的位置
  const uint8_t *begin() const;               // 数据起始地址
  size_t size() const;                        // 数据字节长度
};

// 抽象封装类
class SensorData : public EnableSharedFromThis<SensorData>, private NonCopyable {
protected:
  SensorData(size_t frame, double timestamp, const rpc::Transform &sensor_transform);
};

// 碰撞事件结构（派生类）
class CollisionEvent : public SensorData {
  // 成员变量：self_actor, other_actor, normal_impulse
};
```

该分层结构有效解耦了传感器数据的传输逻辑（RawData）与逻辑处理（SensorData），适用于多种类型传感器的统一调度。

### 5.1 在序列化器中的角色

* 服务端通过 `CollisionEventSerializer::Serialize()` 将 `SensorData` 内容编码为 `RawData`；
* 客户端通过 `Deserialize()` 将字节流还原为 `SensorData` 对象（如 `CollisionEvent`）；
* 再通过 `.listen()` 注册的回调函数传入 Python 层进行用户定义的处理。

---

## 6 Python API 使用示例

> 以下代码片段展示如何在客户端中部署碰撞传感器、注册回调函数，并在事件触发时获取相关信息。

```
# 定义回调函数：每当碰撞发生时，将自动调用该函数
def on_collision(event):
    actor = event.other_actor              # 获取碰撞对方 Actor（carla.Actor）
    impulse = event.normal_impulse        # 冲击力向量对象（carla.Vector3D）
    print(f"发生碰撞！对象类型：{actor.type_id}，冲击向量：{impulse}")
```

```
# 初始化流程（前提是已有 client、world、vehicle 实例）

# 获取蓝图库
blueprint_library = world.get_blueprint_library()

# 检索并实例化碰撞传感器蓝图
bp = blueprint_library.find('sensor.other.collision')

# 设置传感器安装位置（相对于车体）
transform = carla.Transform(carla.Location(x=0, y=0, z=2.0))

# 创建传感器实例并附着到车辆上
sensor = world.spawn_actor(bp, transform, attach_to=vehicle)

# 注册回调函数，开始监听碰撞事件
sensor.listen(on_collision)
```

该机制具备极高的实用性和拓展性，常用于自动驾驶测试与仿真安全评估中。

---

## 7 文件分布

| 文件名                               | 所在路径           | 功能描述           |
|-----------------------------------|----------------| -------------- |
| `CollisionEvent.h`                | `sensor/data/` | 定义事件传感器的数据结构   |
| `CollisionEventSerializer.h/.cpp` | `sensor/s11n/` | 事件数据序列化与反序列化逻辑 |
| `RawData.h`                       | `sensor/`      | 抽象数据传输容器定义     |
| Python 使用示例                    | 用户脚本目录         | 回调注册、数据接收与处理逻辑 |

---

## 8 小结与拓展

* `sensor.other.collision` 是 CARLA 提供的一类低资源占用、实时性强的事件触发型传感器，适合用于环境交互分析与行为建模。
* 底层数据传输基于 `MsgPack` 与 `RawData`，在效率与解耦之间取得良好平衡。
* 该模块适用于事故记录、碰撞回放、安全评估、强化学习惩罚机制等多种应用场景。

### 可拓展方向包括：

* **融合分析**：结合 IMU/GNSS 等连续型传感器实现碰撞前后的行为轨迹重构；
* **图像回溯**：联合摄像头图像数据用于场景理解、可视化回放；
* **策略生成**：结合 RL 模块实现基于碰撞反馈的避障与恢复机制；
* **结构增强**：可扩展事件结构，增加碰撞位置、相对速度等字段，以支持更精细的建模需求。

---

