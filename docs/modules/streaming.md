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







概述
本文件定义了 CARLA (Computer Vision Center) 中 Streaming 组件的端点 (EndPoint) 相关功能，基于 Boost.Asio 提供的网络库，用于管理 TCP/UDP 网络连接的端点（IP 地址 + 端口）。

该文件的主要功能：

提供 完全定义（FullyDefinedEndPoint）和 部分定义（PartiallyDefinedEndPoint）的端点类型。
提供多种 EndPoint 创建方式，包括：
通过 boost::asio::ip::basic_endpoint<Protocol> 直接创建完全定义的端点。
通过 IP 地址 + 端口 创建完全定义的端点。
通过 端口号 创建部分定义的端点（默认使用 IPv4）。
提供工具函数：将字符串地址转换为 Boost.Asio IP 地址。
头文件依赖
cpp
复制
编辑
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
boost/asio/io_context.hpp：提供 io_context，用于管理 I/O 事件循环。
boost/asio/ip/address.hpp：用于处理 IPv4 和 IPv6 地址。
boost/asio/ip/tcp.hpp 和 boost/asio/ip/udp.hpp：分别支持 TCP 和 UDP 网络通信。
命名空间
该文件的所有代码都在 carla::streaming::detail 和 carla::streaming 命名空间下，以便组织 CARLA 的 Streaming 组件。

核心类型
端点类型定义
cpp
复制
编辑
struct FullyDefinedEndPoint {};
struct PartiallyDefinedEndPoint {};
FullyDefinedEndPoint：完全定义的端点，包含 IP 地址 和 端口。
PartiallyDefinedEndPoint：部分定义的端点，仅包含 端口，IP 地址默认使用 IPv4::v4()。
模板类 EndPoint<Protocol, EndPointType>
EndPoint 类是 boost::asio::ip::basic_endpoint<Protocol> 的封装，提供更清晰的端点管理。

完全定义的端点
cpp
复制
编辑
template <typename Protocol>
class EndPoint<Protocol, FullyDefinedEndPoint> {
public:
    explicit EndPoint(boost::asio::ip::basic_endpoint<Protocol> ep)
      : _endpoint(std::move(ep)) {}

    auto address() const { return _endpoint.address(); }
    uint16_t port() const { return _endpoint.port(); }

    operator boost::asio::ip::basic_endpoint<Protocol>() const {
      return _endpoint;
    }

private:
    boost::asio::ip::basic_endpoint<Protocol> _endpoint;
};
存储：

_endpoint：boost::asio::ip::basic_endpoint<Protocol>，包含 IP 地址和端口。
功能：

address()：返回 IP 地址。
port()：返回端口号。
operator boost::asio::ip::basic_endpoint<Protocol>()：支持转换为 boost::asio 的 basic_endpoint。
部分定义的端点
cpp
复制
编辑
template <typename Protocol>
class EndPoint<Protocol, PartiallyDefinedEndPoint> {
public:
    explicit EndPoint(uint16_t port) : _port(port) {}

    uint16_t port() const { return _port; }

    operator boost::asio::ip::basic_endpoint<Protocol>() const {
      return {Protocol::v4(), _port}; // 默认使用 IPv4
    }

private:
    uint16_t _port;
};
存储：

_port：端口号（没有 IP 地址）。
功能：

port()：返回端口号。
operator boost::asio::ip::basic_endpoint<Protocol>()：
仅使用端口，IP 地址默认为 IPv4::v4()。
辅助函数
创建本地主机地址
cpp
复制
编辑
static inline auto make_localhost_address() {
    return boost::asio::ip::make_address("127.0.0.1");
}
返回 127.0.0.1 地址，适用于 本机通信。
解析字符串地址
cpp
复制
编辑
static inline auto make_address(const std::string &address) {
    boost::asio::io_context io_context;
    boost::asio::ip::tcp::resolver resolver(io_context);
    boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), address, "",
                                                boost::asio::ip::tcp::resolver::query::canonical_name);
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    while (iter != end) {
        boost::asio::ip::tcp::endpoint endpoint = *iter++;
        return endpoint.address();
    }

    return boost::asio::ip::make_address(address);
}
功能：
解析 字符串 IP 地址 或 主机名。
使用 boost::asio::ip::tcp::resolver 进行 DNS 解析（例如，将 localhost 解析为 127.0.0.1）。
如果解析失败，则直接返回 boost::asio::ip::make_address(address)。
创建端点的工具函数
创建完全定义的端点
cpp
复制
编辑
template <typename Protocol>
static inline auto make_endpoint(boost::asio::ip::basic_endpoint<Protocol> ep) {
    return detail::EndPoint<Protocol, detail::FullyDefinedEndPoint>{std::move(ep)};
}
通过 boost::asio::ip::basic_endpoint 直接创建 EndPoint。
cpp
复制
编辑
template <typename Protocol>
static inline auto make_endpoint(const char *address, uint16_t port) {
    return make_endpoint<Protocol>({make_address(address), port});
}
通过 字符串地址 + 端口 创建 EndPoint。
cpp
复制
编辑
template <typename Protocol>
static inline auto make_endpoint(const std::string &address, uint16_t port) {
    return make_endpoint<Protocol>(address.c_str(), port);
}
通过 std::string 调用 const char* 版本，保持一致性。
创建部分定义的端点
cpp
复制
编辑
template <typename Protocol>
static inline auto make_endpoint(uint16_t port) {
    return detail::EndPoint<Protocol, detail::PartiallyDefinedEndPoint>{port};
}
仅通过 端口 创建端点（IP 默认为 IPv4::v4()）。
总结
该文件为 CARLA Streaming 组件提供了端点管理：

支持 TCP 和 UDP，封装 boost::asio::ip::basic_endpoint<Protocol>。
支持完全定义和部分定义的端点，灵活处理 IP 地址和端口。
提供辅助函数：
make_address() 解析字符串 IP 地址。
make_endpoint() 生成端点实例。
适用于网络通信场景，例如 数据流订阅、传感器数据传输等。
该实现充分利用 Boost.Asio 提供的网络功能，使 CARLA 的 Streaming 组件能够高效管理端点。



概述
Server 类是 CARLA (Computer Vision Center) 框架中的 流式传输服务器，用于管理数据流的创建、关闭、订阅和分发。每个流都有一个唯一的 Token，客户端可以使用该 Token 订阅流数据。

该服务器基于 Boost.Asio 进行 异步 I/O 处理，并使用 ThreadPool 进行多线程调度。底层通信采用 low_level::Server<detail::tcp::Server> 进行 TCP 传输。

主要功能
创建和管理数据流：
MakeStream() 创建新的数据流。
CloseStream(id) 关闭指定 ID 的流。
流式传输管理：
GetToken(sensor_id) 获取流的 Token，供客户端订阅。
SetSynchronousMode(is_synchro) 设定同步/异步模式。
服务器配置：
SetTimeout(timeout) 设置超时时间。
GetLocalEndpoint() 获取本地端点（IP + 端口）。
运行服务器：
Run() 以同步模式运行服务器。
AsyncRun(worker_threads) 以 异步模式 运行服务器，支持多线程。
ROS（Robot Operating System）支持：
EnableForROS(sensor_id)/DisableForROS(sensor_id) 启用或禁用流的 ROS 支持。
IsEnabledForROS(sensor_id) 检查流是否启用了 ROS。
头文件依赖
cpp
复制
编辑
#include "carla/ThreadPool.h"
#include "carla/streaming/detail/tcp/Server.h"
#include "carla/streaming/detail/Types.h"
#include "carla/streaming/low_level/Server.h"
#include <boost/asio/io_context.hpp>
carla/ThreadPool.h：CARLA 的线程池管理，支持并发流处理。
carla/streaming/detail/tcp/Server.h：TCP 服务器底层实现，提供数据传输能力。
carla/streaming/detail/Types.h：定义 流 ID 和 Token 类型。
carla/streaming/low_level/Server.h：CARLA 低级别服务器封装。
boost/asio/io_context.hpp：Boost.Asio 异步 I/O 支持。
类定义
cpp
复制
编辑
class Server {
    using underlying_server = low_level::Server<detail::tcp::Server>;
    using protocol_type = low_level::Server<detail::tcp::Server>::protocol_type;
    using token_type = carla::streaming::detail::token_type;
    using stream_id = carla::streaming::detail::stream_id_type;
underlying_server：底层 TCP 服务器 类型。
protocol_type：服务器协议（TCP）。
token_type：流订阅的 Token 类型。
stream_id：数据流 ID 类型。
构造函数
基于端口的服务器
cpp
复制
编辑
explicit Server(uint16_t port)
  : _server(_pool.io_context(), make_endpoint<protocol_type>(port)) {}
参数：

port：服务器监听的端口号。
功能：

绑定 本机 IP (IPv4::v4()) 和指定端口。
服务器数据流的 IO 事件由 _pool.io_context() 处理。
基于地址和端口的服务器
cpp
复制
编辑
explicit Server(const std::string &address, uint16_t port)
  : _server(_pool.io_context(), make_endpoint<protocol_type>(address, port)) {}
参数：

address：服务器绑定的 IP 地址。
port：服务器监听的端口。
功能：

绑定 指定的 IP 地址 和 端口，适用于多网卡环境。
支持内外网的服务器
cpp
复制
编辑
explicit Server(
    const std::string &address, uint16_t port,
    const std::string &external_address, uint16_t external_port)
  : _server(
      _pool.io_context(),
      make_endpoint<protocol_type>(address, port),
      make_endpoint<protocol_type>(external_address, external_port)) {}
参数：

address/port：服务器的 内部地址和端口。
external_address/external_port：服务器的 外部地址和端口（用于 NAT 转发）。
功能：

允许 内外网地址不同，适用于 云服务器或 NAT 场景。
析构函数
cpp
复制
编辑
~Server() {
  _pool.Stop();
}
功能：
停止 线程池，释放资源。
服务器操作
获取本地端点
cpp
复制
编辑
auto GetLocalEndpoint() const {
  return _server.GetLocalEndpoint();
}
返回：服务器的本地 IP 和端口。
设置超时时间
cpp
复制
编辑
void SetTimeout(time_duration timeout) {
  _server.SetTimeout(timeout);
}
参数：

timeout：超时时间。
功能：

设置 数据流的超时时间，防止 长期未订阅的流占用资源。
流操作
创建流
cpp
复制
编辑
Stream MakeStream() {
  return _server.MakeStream();
}
功能：
创建 新的数据流，供客户端订阅。
关闭流
cpp
复制
编辑
void CloseStream(carla::streaming::detail::stream_id_type id) {
  return _server.CloseStream(id);
}
参数：

id：要关闭的 流 ID。
功能：

关闭 指定 ID 的数据流。
服务器运行
同步模式
cpp
复制
编辑
void Run() {
  _pool.Run();
}
功能：
同步模式 运行 服务器线程池。
异步模式
cpp
复制
编辑
void AsyncRun(size_t worker_threads) {
  _pool.AsyncRun(worker_threads);
}
参数：

worker_threads：工作线程数量。
功能：

异步运行 服务器，适用于 高并发环境。
同步模式配置
cpp
复制
编辑
void SetSynchronousMode(bool is_synchro) {
  _server.SetSynchronousMode(is_synchro);
}
参数：

is_synchro：是否启用 同步模式。
功能：

设定 同步/异步 运行模式。
ROS 支持
获取流 Token
cpp
复制
编辑
token_type GetToken(stream_id sensor_id) {
  return _server.GetToken(sensor_id);
}
功能：
获取 流的唯一 Token，用于客户端订阅。
启用 ROS
cpp
复制
编辑
void EnableForROS(stream_id sensor_id) {
  _server.EnableForROS(sensor_id);
}
功能：
允许 ROS 系统订阅该流。
禁用 ROS
cpp
复制
编辑
void DisableForROS(stream_id sensor_id) {
  _server.DisableForROS(sensor_id);
}
检查 ROS 状态
cpp
复制
编辑
bool IsEnabledForROS(stream_id sensor_id) {
  return _server.IsEnabledForROS(sensor_id);
}
总结
支持 TCP 传输，用于 传感器数据流。
管理多个数据流，支持 动态创建/关闭。
异步/同步模式，支持 高并发。
支持 ROS，可用于机器人应用。
适用于 CARLA 仿真环境，管理 车辆、传感器等数据流。
该 Server 适用于 自动驾驶仿真、数据流管理、实时通信 等场景。







CARLA Streaming Stream 说明文档
概述
本文件定义了 CARLA (Computer Vision Center) 框架中 Streaming 组件的 Stream 类型，用于在服务器和客户端之间建立单向的数据流通道。

核心功能
单向数据流：服务器向客户端 发送数据，客户端可通过 订阅流令牌 进行访问。
多客户端支持：多个客户端可以同时订阅 同一个流，共享数据。
无订阅丢弃机制：如果 没有客户端订阅 该流，推送的数据会被 自动丢弃，不会占用额外的资源。
该 Stream 依赖 MultiStreamState 进行状态管理，确保流的正确操作和多客户端支持。

头文件依赖
cpp
复制
编辑
#include "carla/streaming/detail/MultiStreamState.h"
#include "carla/streaming/detail/Stream.h"
carla/streaming/detail/MultiStreamState.h
负责管理 多客户端订阅状态，即 Stream 支持多个客户端订阅的功能由其实现。
carla/streaming/detail/Stream.h
具体实现 数据流 的创建、管理和数据推送。
命名空间
该代码位于 carla::streaming 命名空间中：

cpp
复制
编辑
namespace carla {
namespace streaming {
carla：CARLA 项目的顶级命名空间。
streaming：CARLA 流式传输 组件的子命名空间。
类型定义
cpp
复制
编辑
using Stream = detail::Stream<detail::MultiStreamState>;
Stream 是 detail::Stream 的 模板特化，基于 MultiStreamState 进行管理：
detail::Stream：实现流的核心逻辑，如 数据传输、客户端订阅管理。
detail::MultiStreamState：
维护多个客户端的 订阅状态。
负责 管理数据流的可见性（是否被订阅）。
控制 数据推送（无订阅时丢弃数据）。
特点
服务器端向客户端推送数据。
支持多客户端订阅，使用 stream token 进行访问。
如果无客户端订阅，则数据会被丢弃，节省资源。
工作机制
1. 订阅与数据传输
服务器创建 Stream 并分配 流令牌 (stream token)。
客户端使用 流令牌 订阅 Stream。
已订阅：数据推送到所有订阅客户端。
未订阅：数据被 丢弃。
2. Stream 生命周期
Stream 创建 后，等待客户端 订阅。
至少一个客户端订阅，数据正常推送。
无客户端订阅，数据会 被丢弃，避免资源浪费。
客户端
cpp
复制
编辑
#include "carla/streaming/Client.h"

int main() {
    carla::streaming::Client client("127.0.0.1");  // 连接服务器
    auto token = ... // 服务器提供的 Token

    // 订阅流并接收数据
    client.Subscribe(token, [](auto data) {
        std::cout << "Received: " << data << std::endl;
    });

    client.Run();  // 开始监听数据
    return 0;
}
使用 Token 订阅 Stream。
接收数据 并输出。
CARLA Streaming Stream 的核心特性
CARLA Stream 组件提供了一种单向数据流机制，允许服务器向客户端传输数据。多个客户端可以订阅同一个流，通过 流令牌（Token） 访问数据。如果没有客户端订阅，该流的数据将会被自动丢弃，避免浪费资源。

Stream 依赖 MultiStreamState 进行状态管理，确保支持多客户端订阅，并根据订阅情况动态管理数据传输。

适用场景
Stream 组件适用于 自动驾驶仿真数据传输、多传感器数据流管理、实时监控、服务器-客户端式流媒体传输等应用场景。特别是在 CARLA 仿真系统 中，它被广泛用于 传感器数据流 的传输，确保多个客户端能够同时访问相同的数据源，而不会造成资源浪费。


概述
Token 是 CARLA (Computer Vision Center) 框架中 Streaming 组件的一部分，它用于 唯一标识 一个数据流，使客户端能够订阅和访问特定的流数据。

Token 由一个 固定大小的 24 字节数组 (std::array<unsigned char, 24u>) 组成，保证了唯一性和高效性。此外，它使用 MsgPack 序列化，支持高效的数据传输和存储。

头文件依赖
cpp
复制
编辑
#include "carla/MsgPack.h"
#include <array>
carla/MsgPack.h：
提供 MsgPack 序列化支持，允许 Token 进行 二进制数据编码，方便在网络通信中传输。
<array>：
使用 std::array 存储 24 字节的数据，确保 Token 大小固定，性能稳定。
命名空间
代码位于 carla::streaming 命名空间：

cpp
复制
编辑
namespace carla {
namespace streaming {
carla：CARLA 仿真框架的顶级命名空间。
streaming：用于管理 流式传输 相关功能的命名空间。
类定义
cpp
复制
编辑
class Token {
public:
    std::array<unsigned char, 24u> data;

    MSGPACK_DEFINE_ARRAY(data);
};
成员变量
std::array<unsigned char, 24u> data;
存储 24 字节的 Token 数据，用于唯一标识某个流。
unsigned char 数组可用于存储 随机生成的唯一标识符。
序列化支持
cpp
复制
编辑
MSGPACK_DEFINE_ARRAY(data);
MsgPack 支持：
允许 Token 自动序列化 和 反序列化，便于 网络传输和存储。
该宏将 data 数组注册为 MsgPack 数据结构，可以用于 高效二进制编码。
功能作用
唯一标识流

Token 作为流的 唯一 ID，用于客户端和服务器之间的数据订阅。
客户端需要 Token 来 请求特定流的数据。
支持多客户端

服务器可以为不同的流 生成不同的 Token，多个客户端可以使用相同的 Token 订阅相同的流。
高效序列化

使用 MsgPack 进行高效编码，相比 JSON 更紧凑，占用更少的存储和带宽。
工作机制
1. 服务器端
服务器 创建数据流 并生成 Token。
Token 通过 网络传输 发送给 客户端。
客户端使用 Token 订阅流，接收数据。
2. 客户端端
客户端获取 Token 后，向服务器 请求订阅流。
服务器验证 Token，如果有效，允许订阅。
订阅成功后，客户端可以 持续接收流数据。
客户端端
cpp
复制
编辑
#include "carla/streaming/Client.h"

int main() {
    carla::streaming::Client client("127.0.0.1");  // 连接服务器
    carla::streaming::Token token = ...;  // 服务器提供的 Token

    // 订阅流
    client.Subscribe(token, [](auto data) {
        std::cout << "Received: " << data << std::endl;
    });

    client.Run();  // 开始监听流数据
    return 0;
}
客户端 使用 Token 订阅流。
服务器推送的数据 会自动传输到客户端。
总结
Token 作为 CARLA Streaming 组件 的核心元素，提供了一种 安全、高效的方式 来唯一标识数据流。它具有以下特点：

用于唯一标识流

Token 是 数据流的唯一 ID，客户端需要 Token 才能订阅流数据。
支持多客户端订阅

服务器可以为不同流生成 不同的 Token，实现多个客户端订阅 同一数据流。
高效序列化

通过 MsgPack 进行二进制编码，提高数据传输效率，减少存储和网络开销。
与 Streaming 组件深度集成

Token 由服务器 生成 并 分发 给客户端，客户端使用 Token 订阅流 并接收数据。
Token 机制确保了 数据流传输的安全性和可控性，在 CARLA 仿真系统中，广泛用于传感器数据传输和远程数据监控。