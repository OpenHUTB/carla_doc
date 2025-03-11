# CARLA Streaming 组件说明文档

## 1. 概述
CARLA Streaming 组件提供了一种 **流式数据传输** 机制，支持 **TCP/UDP 通信**，用于自动驾驶仿真环境中的 **传感器数据流** 传输。  
它支持 **多客户端订阅**、**同步/异步模式**，并可与 **ROS（Robot Operating System）** 兼容。

---

## 2. Client（客户端）

### 2.1 功能概述
`Client` 类用于 **订阅数据流** 并进行处理，支持：
- **多流订阅**：基于 `Token` 订阅数据流。
- **取消订阅**：停止订阅指定数据流。
- **同步/异步模式**：
  - `Run()`：同步运行 `ThreadPool`，处理数据流。
  - `AsyncRun(size_t worker_threads)`：异步运行 `ThreadPool`，提高并发处理能力。

### 2.2 头文件依赖
```cpp
#include "carla/Logging.h"
#include "carla/ThreadPool.h"
#include "carla/streaming/Token.h"
#include "carla/streaming/detail/tcp/Client.h"
#include "carla/streaming/low_level/Client.h"
#include <boost/asio/io_context.hpp>