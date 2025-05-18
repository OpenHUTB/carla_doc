# FCarlaServer 类说明文档

`FCarlaServer` 是 CARLA 模拟器中用于管理服务器通信与模拟控制的核心类。该类主要负责：
   - 启动和停止RPC和数据流服务器；
   - 管理 CARLA Episode 的生命周期；
   - 控制 Actor 的创建与销毁；
   - 传感器数据的流式传输；
   - 多 GPU 支持下的数据路由。


## 类成员函数

### 构造函数与析构函数

- **FCarlaServer()**
  - 默认构造函数，用于创建 `FCarlaServer` 类的对象实例。
### 服务器启动与停止

- **FDataMultiStream Start(uint16_t RPCPort, uint16_t StreamingPort, uint16_t SecondaryPort)**
  启动服务器相关功能，
    - 传入 RPC 端口号（RPCPort）；
    - 流数据端口号（StreamingPort）；
    - 备用端口号（SecondaryPort）；
   返回一个多数据流对象（FDataMultiStream），用于多通道数据流处理。
- **void Stop()**
  - 停止服务器运行，释放相关资源，关闭各种连接等，将服务器置于停止状态。

### 场景管理

- **void NotifyBeginEpisode(UCarlaEpisode &Episode)**
  - 通知服务器开始一个 CARLA Episode（可能是一个模拟场景、任务等的阶段），传入对应的 `UCarlaEpisode` 对象引用，以便服务器知晓相关信息进行对应处理。

- **void NotifyEndEpisode()**
  - 通知服务器结束当前的 CARLA Episode，让服务器进行相应的清理、收尾等操作。

### 服务器运行

- **void AsyncRun(uint32 NumberOfWorkerThreads)**
    - 以异步方式运行服务器，传入工作线程数量（NumberOfWorkerThreads）参数，可让服务器利用多线程来高效处理各种任务，比如数据接收、处理和发送等。
- **void RunSome(uint32 Milliseconds)**
  - 运行服务器一段时间，传入时间（以毫秒为单位，Milliseconds）参数，在指定时长内执行服务器相关的逻辑，比如处理数据、更新状态等。
- **void Tick()**
  - 执行服务器的一次“滴答”操作，通常用于周期性地更新服务器状态、处理数据等，类似于游戏循环里的每一帧更新逻辑。

- **bool TickCueReceived()**
  - 检查是否接收到了“滴答”提示（Tick Cue），返回布尔值表示是否收到，可用于判断是否需要进行下一步相关操作等。

### 数据流管理

- **FDataStream OpenStream() const**
  - 打开一个数据流，返回一个数据流转对象（FDataStream），用于后续的数据读取、写入等操作，可能是针对特定的数据传输通道进行操作。

### 多 GPU 路由器与流服务器

- **std::shared_ptr<carla::multigpu::Router> GetSecondaryServer()**
  - 获取指向多 GPU 路由器（multigpu::Router）的共享指针，这个路由器可能用于在多 GPU 环境下对数据等进行路由转发、调度等功能，返回的是智能指针便于内存管理。

- **carla::streaming::Server &GetStreamingServer()**
  - 获取流服务器（streaming::Server）的引用，以便能直接操作流服务器对象，比如配置服务器参数、获取服务器状态信息等。

## 内部类 FPimpl
`FCarlaServer` 类内部定义了一个名为 FPimpl 的私有内部类，通常这种方式用于实现“编译防火墙”（Pimpl，即“Pointer to implementation”），将类的实现细节隐藏在内部类中，对外只暴露接口。

- **关键成员**：  
  - `StreamingServer`: 管理传感器数据流（如摄像头、激光雷达）。  
  - `SecondaryServer`: 多 GPU 环境下的数据路由控制器。  
  - `BindActions()`: 绑定 RPC 操作到具体实现（如 `SpawnActor`、`DestroyActor`）。
  
### FPimpl 类成员

|               成员                    | 说明                             |
|---------------------------------------|----------------------------------|
|`StreamingServer`                      | 传感器数据流服务器                |
|`SecondaryServer`                      | 多CPU路由器                      |
|`BroadcastStream`                      | 广播数据流                       |
|`Episode`                              | 当前模拟场景对象                  |
|`TickCuesReceived`                     | 已接收 Tick 信号数量              |


- **FPimpl(uint16_t RPCPort, uint16_t StreamingPort, uint16_t SecondaryPort)**
  - 构造函数，初始化 RPC 服务器、流服务器和多 GPU 路由器。

- **std::shared_ptr<carla::multigpu::Router> GetSecondaryServer()**
  - 获取多 GPU 路由器的共享指针。

- **carla::streaming::Server StreamingServer**
  - 流服务器对象，用于处理传感器数据的传输。

- **carla::streaming::Stream BroadcastStream**
  - 广播流对象，用于向多个客户端广播数据。

- **std::shared_ptr<carla::multigpu::Router> SecondaryServer**
  - 多 GPU 路由器对象，用于在多 GPU 环境下进行数据路由。

- **UCarlaEpisode Episode**
  - 当前 CARLA Episode 的指针，用于管理模拟场景。

- **std::atomic_size_t TickCuesReceived**
  - 用于记录接收到的“滴答”提示数量。




### FPimpl 类私有函数

- **void BindActions()**
  - 绑定 RPC 服务器的各种操作，包括场景加载、Actor 生成与销毁、传感器数据管理等。

## 辅助宏
这些宏用统一接口绑定和错误处理：
| 宏名                           | 说明                   |
| -------------------------------- | -------------------------- |
| `BIND_SYNC(name)`                     | 同步绑定 RPC 函数                   |
| `BIND_ASYNC(name)`                    | 异步绑定 RPC 函数                   |
| `REQUIRE_CARLA_EPISODE()`             | 确保 Episode 存在                   |
| `RESPOND_ERROR(str)`                  | 返回错误响应（字符串）               |
| `RESPOND_ERROR_FSTRING(fstr)`         | 返回错误响应（FString）             |
| `CARLA_ENSURE_GAME_THREAD()`          | 确保运行在游戏线程                  |





- **BIND_SYNC(name)**
  - 用于同步绑定 RPC 函数。
  
- **BIND_ASYNC(name)**
  - 用于异步绑定RPC函数
  - 用于同步绑定 RPC 函数（阻塞式调用，适用于简单操作）。
  
- **BIND_ASYNC(name)**
  - 用于异步绑定 RPC 函数（非阻塞式调用，适用于耗时操作）。
- **REQUIRE_CARLA_EPISODE()**
  - 确保当前有一个有效的 CARLA Episode 正在运行。

- **RESPOND_ERROR(str)**
  - 记录错误消息并返回一个 `ResponseError` 对象。

- **RESPOND_ERROR_FSTRING(fstr)**
  - 记录由 `FString` 表示的错误消息并返回一个 `ResponseError` 对象。

- **CARLA_ENSURE_GAME_THREAD()**
  - 确保代码在游戏线程中执行。

# Carla 服务器响应状态说明文档

## 版权信息
Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
本作品在 MIT 许可证下发布，许可证副本见 [The MIT License](https://opensource.org/licenses/MIT)。

## 枚举类型 ECarlaServerResponse
该枚举类型用于表示 Carla 服务器的响应状态，具体枚举值及其含义如下：

| 枚举值                           | 含义描述                   |
| -------------------------------- | -------------------------- |
| Success                          | 操作成功                   |
| ActorNotFound                    | 找不到指定的 actor         |
| ComponentNotFound                | 找不到指定的组件           |
| ActorTypeMismatch                | actor 类型不匹配           |
| FunctionNotSupported             | 不支持的函数调用           |
| NullActor                        | 空的 actor                 |
| MissingActor                     | 缺少 actor                 |
| NotAVehicle                      | 不是车辆类型的 actor       |
| WalkerDead                       | 行人已死亡                 |
| NotAWalker                       | 不是行人类型的 actor       |
| WalkerIncompatibleController     | 行人控制器不兼容           |
| AutoPilotNotSupported            | 自动驾驶不支持             |
| CarSimPluginNotEnabled           | 车辆模拟插件未启用         |
| NotATrafficLight                 | 不是交通信号灯             |
|FunctionNotAvailiableWhenDormant  |当处于休眠状态时函数不可用   |

## 函数 CarlaGetStringError
该函数用于根据给定的 ECarlaServerResponse 枚举值返回对应的错误描述字符串。

### 函数原型
```cpp
FString CarlaGetStringError(ECarlaServerResponse Response);
  ECarlaServerResponse ErrorCode = ECarlaServerResponse::ActorNotFound;
  FString ErrorMsg = CarlaGetStringError(ErrorCode); // 返回 "Actor not found"
