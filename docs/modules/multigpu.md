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
实例：系统启动时，主节点调用 Initialize() 加载配置文件、启动内部服务（如任务队列和心跳检测），并绑定到 TCP 端口 8000。 
场景：主节点启动后完成资源准备，等待从节点连接。
- `RegisterSecondary(SecondaryInfo info)`：注册从节点信息  
实例：当从节点 Secondary-01 上线时，调用此方法将其 IP 192.168.1.101 和可用 GPU 资源信息注册到主节点的节点池中。 
场景：主节点维护可用从节点列表，后续任务将基于此列表分发。
- `DistributeTask(Task task)`：任务分发接口  
实例：收到一个图像渲染任务 Task-123 后，主节点通过负载均衡策略选择空闲的从节点 Secondary-02，调用此方法将任务发送到该节点。 
场景：任务分发的核心逻辑，确保任务高效执行。
### Secondary

从节点类，执行接收到的任务。

- `ConnectToPrimary(string address)`：连接主节点  
实例：从节点启动后，调用 `ConnectToPrimary("tcp://master-node:8000")`，与主节点建立长连接并发送注册请求。  
场景：从节点加入集群的第一步。
- `ReceiveTask()`：等待任务接收  
实例：从节点在后台运行 `ReceiveTask()`，阻塞等待主节点推送任务，收到任务后触发`ExecuteTask`。 
场景：持续监听任务的核心循环。
- `ExecuteTask(Task task)`：执行并返回结果  
实例：接收到数据处理任务 Task-456 后，从节点调用此方法运行 Python 脚本，完成后将结果文件路径返回给主节点。  
场景：实际执行任务的入口，支持自定义业务逻辑。
### Router

- `RouteMessage(Message msg)`：解析并路由不同类型的消息  
实例：收到一条类型为 TaskResult 的消息，路由解析后将其转发到主节点的 TaskManager 模块；若为 Heartbeat 类型，则转交给 HealthMonitor。  
场景：消息分类处理，确保系统模块解耦。
### Listener

- `StartListening()`：开启监听端口  
实例：监听器启动时调用此方法，在端口 8080 上监听 HTTP 请求，接收外部用户提交的任务或管理指令。   
场景：系统对外服务的入口。
- `HandleIncomingMessage(Message msg)`：处理并响应外部指令  
实例：收到一条 RestartNode 指令后，验证权限并触发主节点重启流程，最终返回 {"status": "success"} 的 JSON 响应。  
场景：处理用户或管理员的操作请求
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
