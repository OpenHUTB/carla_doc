# CARLA 多 GPU 模块技术文档（multigpu）

## 目录

1. 模块概述  
2. 核心功能  
3. 类与方法详解  
4. 通信机制  
5. 关键流程  
6. 配置参数  
7. 注意事项  
8. 示例代码  

---

## 1. 模块概述

`multigpu` 模块用于支持 CARLA 模拟器在多 GPU 环境下进行分布式渲染和任务计算。模块采用主从架构：主节点负责调度与控制，从节点承担实际渲染任务，从而提升模拟器的可扩展性与运行效率。

主要组件包括：

- **Primary**：主节点，调度中心，分发任务与监控从节点状态  
- **Secondary**：从节点，执行渲染与计算任务  
- **Router**：路由器，管理消息转发和节点通信  
- **Listener**：监听器，接收控制命令并分发  

---

## 2. 核心功能

- 多 GPU 分布式渲染任务协同执行  
- 高效稳定的节点间通信机制  
- 支持从节点的动态注册与任务重分配  
- 灵活的任务协议与自定义消息结构  

---

## 3. 类与方法详解

### Primary

主节点类，调度与管理核心。

- `Initialize()`：初始化主节点资源  
- `RegisterSecondary(SecondaryInfo info)`：注册从节点信息  
- `DistributeTask(Task task)`：任务分发接口  

### Secondary

从节点类，执行接收到的任务。

- `ConnectToPrimary(string address)`：连接主节点  
- `ReceiveTask()`：等待任务接收  
- `ExecuteTask(Task task)`：执行并返回结果  

### Router

- `RouteMessage(Message msg)`：解析并路由不同类型的消息  

### Listener

- `StartListening()`：开启监听端口  
- `HandleIncomingMessage(Message msg)`：处理并响应外部指令  

---

## 4. 通信机制

模块通信使用 TCP 或 UDP（可配置），支持命令、状态与数据三类消息。消息格式包括：

- **MessageType**：标识类型（命令 / 数据 / 心跳等）  
- **Payload**：消息数据体  
- **Checksum**：可选字段，校验完整性  

所有消息通过 Router 进行解包、路由和转发。

---

## 5. 关键流程

### 从节点注册

1. 从节点调用 `ConnectToPrimary()`  
2. 主节点执行 `RegisterSecondary()`  
3. 主节点更新从节点列表，准备任务分发  

### 任务调度与处理

1. 主节点调用 `DistributeTask()` 分配任务  
2. 从节点通过 `ReceiveTask()` 接收任务  
3. 从节点执行 `ExecuteTask()`，并返回结果  

---

## 6. 配置参数

### 主节点配置

| 参数            | 含义                 |
|-----------------|----------------------|
| ListenPort      | 主节点监听端口       |
| MaxSecondaries  | 支持的最大从节点数   |

### 从节点配置

| 参数              | 含义                     |
|-------------------|--------------------------|
| PrimaryAddress    | 主节点 IP 地址           |
| ReconnectInterval | 断线重连时间间隔（秒）   |

---

## 7. 注意事项

- 网络通信必须具备稳定的端口与带宽保障  
- 主节点内部数据结构需线程安全管理  
- 多线程执行任务时，确保 GPU 不冲突访问  
- 从节点应具备异常重连与任务重试机制  

---

## 8. 示例代码

```cpp
// 主节点初始化
Primary primary;
primary.Initialize();

// 从节点连接主节点
Secondary secondary;
secondary.ConnectToPrimary("127.0.0.1");

// 主节点任务分发
Task task = CreateRenderTask();
primary.DistributeTask(task);
```

---

# 9. 附录

## 9.1 依赖库

| 库名称            | 功能       | 说明                          |
|-------------------|------------|-------------------------------|
| Boost.Asio        | 网络通信   | 提供异步 TCP/UDP 支持          |
| spdlog            | 日志记录   | 轻量级、高性能日志库            |
| protobuf (可选)    | 数据序列化 | 高效结构化消息传输，用于自定义协议 |
| nlohmann/json     | JSON 支持  | 现代 C++ 风格 JSON 序列化与解析  |
| STL 线程组件      | 并发支持   | 用于多线程与线程同步控制         |

## 9.2 相关模块引用

| 模块名称               | 功能协作关系                   |
|------------------------|------------------------------|
| carla::rpc             | 使用 RPC 技术进行任务分发      |
| carla::rendering       | GPU 渲染任务执行的核心模块     |
| carla::sensor          | 渲染输出对接传感器系统         |
| carla::traffic_manager | 多 GPU 加速城市交通模拟流程    |
