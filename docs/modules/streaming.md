概述
Client 类是 CARLA (Computer Vision Center) 框架中的一个流式数据客户端，用于订阅数据流并进行处理。它支持多流订阅，并提供同步和异步运行方式。

该类使用 Boost.Asio 进行异步 I/O 操作，并依赖 CARLA 提供的 ThreadPool 进行多线程处理。同时，它通过 low_level::Client<detail::tcp::Client> 作为底层通信实现，使用 TCP 进行数据传输。

主要功能
订阅流数据（Subscribe）：支持用户基于 Token 订阅特定的数据流。
取消订阅流（UnSubscribe）：可取消某个数据流的订阅。
同步运行（Run）：以同步方式启动 ThreadPool，处理数据流。
异步运行（AsyncRun）：以异步方式启动 ThreadPool，支持指定工作线程数量。
头文件依赖
本类依赖 CARLA 及 Boost 库，涉及的主要头文件如下：

carla/Logging.h：CARLA 的日志系统，可能用于调试或错误记录。
carla/ThreadPool.h：CARLA 的线程池实现，用于管理数据流的异步处理。
carla/streaming/Token.h：数据流的令牌（Token），用于标识不同的流。
carla/streaming/detail/tcp/Client.h：TCP 客户端，作为数据流的通信基础。
carla/streaming/low_level/Client.h：CARLA 低级流式处理客户端。
boost/asio/io_context.hpp：Boost.Asio 提供的 I/O 上下文，用于异步操作。
类定义
cpp
复制
编辑
class Client {
    using underlying_client = low_level::Client<detail::tcp::Client>;
Client 类内部封装了 underlying_client，其中 low_level::Client<detail::tcp::Client> 作为底层 TCP 通信实现。

构造函数
cpp
复制
编辑
Client() = default;
默认构造函数：不做任何初始化，依赖 underlying_client 默认状态。
cpp
复制
编辑
explicit Client(const std::string &fallback_address) : _client(fallback_address) {}
带参数的构造函数：
fallback_address：用于初始化 underlying_client，作为 TCP 连接的备用地址。
析构函数
cpp
复制
编辑
~Client() {
    _service.Stop();
}
在对象销毁时，调用 ThreadPool 的 Stop() 方法，确保线程池资源释放，避免程序崩溃。
数据流订阅
cpp
复制
编辑
template <typename Functor>
void Subscribe(const Token &token, Functor &&callback) {
    _client.Subscribe(_service.io_context(), token, std::forward<Functor>(callback));
}
参数：

token：数据流的唯一标识，用于订阅特定流。
callback：回调函数，处理订阅到的数据。
功能：

通过 _client.Subscribe() 调用底层 TCP 客户端进行数据流订阅。
使用 ThreadPool 的 io_context() 进行异步 I/O 事件处理。
⚠️ 注意：不能对同一个数据流重复订阅，即使是多流（MultiStream）。

取消数据流订阅
cpp
复制
编辑
void UnSubscribe(const Token &token) {
    _client.UnSubscribe(token);
}
功能：
取消对 token 代表的数据流的订阅。
同步运行
cpp
复制
编辑
void Run() {
    _service.Run();
}
功能：
以同步方式运行 ThreadPool，处理数据流。
异步运行
cpp
复制
编辑
void AsyncRun(size_t worker_threads) {
    _service.AsyncRun(worker_threads);
}
参数：

worker_threads：工作线程数量。
功能：

以异步方式运行 ThreadPool，适用于多线程高效处理数据流。
成员变量
cpp
复制
编辑
ThreadPool _service;
underlying_client _client;
_service：CARLA 提供的线程池，用于管理异步 I/O 操作。
_client：底层 TCP 客户端，负责流式数据传输。
⚠️ 变量顺序重要：

ThreadPool _service 需要先定义，确保 _client 能正确访问 io_context()。
总结
Client 作为 CARLA 流式数据的订阅客户端，提供：

多流订阅，但不能重复订阅同一流。
基于 TCP 进行数据通信，依赖 Boost.Asio 进行异步 I/O 操作。
支持同步与异步模式，可根据需求选择 Run() 或 AsyncRun(size_t)。
使用 ThreadPool 进行线程管理，确保高效运行。
该类适用于 CARLA 仿真系统中需要流式传输数据的场景，例如传输传感器数据、图像或车辆状态信息。