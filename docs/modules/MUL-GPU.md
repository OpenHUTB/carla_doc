# <center> **CARLA -- 多 GPU 系统** <center>

##  概述
CARLA 项目支持多 GPU 架构，旨在通过高效的设计管理和协调多个 GPU 之间的资源和任务。通过这种架构，用户可以启动多个辅助服务器，每个辅助服务器利用系统中的专用 GPU 来执行渲染工作，而主服务器则专注于处理物理数据和场景同步。主服务器将用户创建的传感器分配到不同的辅助服务器，确保任务的分配更加高效，并且能够利用系统中每个 GPU 的计算能力。

在 CARLA 的多 GPU 系统中，用户无需担心复杂的后台计算。所有的计算和渲染工作都是透明的，用户直接与主服务器进行交互，而不需要关心数据是来自哪个辅助服务器。每个辅助服务器都可以与主服务器通过指定端口进行连接，执行特定的渲染任务，并将处理结果返回给主服务器。整个系统实现了 GPU 资源的优化管理，确保多个节点的协同工作能够发挥最大的计算效能。本文档将详细介绍每个类的功能、方法及它们之间的关系，帮助理解系统如何高效地运行。

![1.png](1)

## 启动多 GPU 系统
为了启动多 GPU 系统，首先需要启动一个主服务器，主服务器的功能是处理所
有物理数据，并将场景数据同步到其他辅助服务器。在启动主服务器时，可以使用 -nullrhi 参数来禁用渲染功能，从而确保主服务器不会占用 GPU 资源。主服务器通过一个指定的端口（默认为 2002）监听来自辅助服务器的连接请求，
帮助协调不同服务器之间的工作。
例如，启动主服务器的命令如下：
> ./CarlaUE4.sh -nullrhi
>
如果需要改变主服务器的监听端口，可以使用 -carla-primary-port 参数指定新的端口：
> ./CarlaUE4.sh -nullrhi -carla-primary-port=3002

## 辅助服务器的启动与配置
辅助服务器负责使用专用的 GPU 进行渲染任务。为了最大化 GPU 的使用效率，通常每个辅助服务器会与一个 GPU 对应。启动辅助服务器时，我们需要指定服务器将使用的 GPU、主服务器的 IP 地址和端口等信息。
启动辅助服务器时的命令通常包括以下参数：
>-carla-rpc-port：设置用于接收客户端连接的 RPC 端口（对于辅助服务器
不必特别需要，但该端口需为空闲状态）
>
>-carla-primary-host： 指定主服务器的 IP 地址。
>
>-carla-primary-port：指定主服务器的监听端口。
>
>-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter：指定辅助
服务器使用的 GPU 设备。
>
例如，如果主服务器和辅助服务器都在同一台机器上运行，并使用默认端口2002连接，命令如下：
>./CarlaUE4.sh -carla-rpc-port=3000 -carla-primary-host=127.0.0.1
-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=0
>
这条命令启动了一个辅助服务器，使用端口 3000 进行 RPC 通信，并通过默认端口 2002 连接到主服务器。此时，辅助服务器将使用 GPU 设备 0 进行渲染。
如果需要启动另一个辅助服务器并使用不同的 GPU 设备（例如 GPU 1），可以使
用以下命令：
>./CarlaUE4.sh -carla-rpc-port=4000 -carla-primary-host=127.0.0.1
-carla-primary-port=2002
-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=1
>
该命令启动了一个新的辅助服务器，使用端口 4000 进行 RPC 通信，连接到主服
务器，并将 GPU 设备设置为 1。
## 同步模式
在多 GPU 系统中，所有辅助服务器与主服务器进行连接后，系统会自动切换到
同步模式，确保各个节点的工作能够精确同步。默认情况下，系统的同步模式设
置为每 20 毫秒进行一次增量更新。这意味着每个服务器都会在固定的时间间隔
内同步其渲染任务和数据状态，以确保仿真过程的实时性和准确性。
一旦主服务器和第一个辅助服务器建立连接，系统会自动进入同步模式，开始按
预定的时间间隔进行数据同步和渲染工作。
## 1. Listener 类
### 功能:
Listener 类是多 GPU 系统中网络通信的入口点。它主要负责在指定的端口上监听来自其他节点（主节点或次级节点）的连接请求。通过这个类，节点能够互相发现并建立连接，为后续的数据传输和命令执行打下基础。在一个分布式系统中，多个节点需要通过网络进行数据和命令交换。
Listener 类的作用就是在网络中提供一个可接入的端口，其他节点可以通过这个端口发送连接请求。Listener 接收到请求后，建立连接并与客户端进行数据交换。
### 关键方法:
start_listening(): 启动监听进程，开始在指定的端口等待连接请求。当有连
接请求到达时，调用 accept_connection() 方法接受连接。
accept_connection(): 异步接受新的连接请求。当接收到连接时，会启动
handle_new_connection() 方法处理该连接，确保每个连接都能被单独管理和维
护。
handle_new_connection(): 处理成功建立的连接，通常包括初始化数据接收
线程、验证连接的有效性、启动后续的消息处理机制等。
### 详细功能:
通过 boost::asio 库提供的异步 I/O 机制，Listener 能够高效地处理多个
并发连接，不会阻塞主线程。
支持连接队列的管理，当多个连接请求同时到达时，系统能够有地依次处
理。
#### 代码示例:
```class Listener {
public:
Listener(boost::asio::io_context& io_context, uint16_t port)
: _acceptor(io_context,
boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)) {}
void start_listening() {
accept_connection();
}
private:
void accept_connection() {
_acceptor.async_accept([this](boost::system::error_code ec,
boost::asio::ip::tcp::socket socket) {
if (!ec) {
handle_new_connection(std::move(socket));
}
accept_connection();
});
}
void handle_new_connection(boost::asio::ip::tcp::socket socket) {
// 处理接收到的连接逻辑
}
boost::asio::ip::tcp::acceptor _acceptor;
};

```
	
## 2. PrimaryCommands 类
### 功能:
PrimaryCommands 类负责主节点向次级节点发送命令的逻辑。它提供了命令的序列化、发送和解析功能。通过该类，主节点可以高效地向一个或多个次级节点分发任务，并等待执行结果。
主节点需要通过命令来控制次级节点的行为，例如请求某个计算任务的执行，或者控制仿真过程中的状态变化。PrimaryCommands 负责将这些命令封装成标准格式并通过网络传输给次级节点。该类还负责接收次级节点的反馈，解析命令结果，并进行相应的后续操作。
### 关键方法:
send_command(MultiGPUCommand command): 将构造好的 MultiGPUCom￾mand 命令封装成网络数据包，并通过网络发送给次级节点。这个过程包括数据
的序列化和发送操作。
process_command(Buffer buffer): 处理从次级节点收到的命令反馈。解析缓
冲区中的数据，并根据命令内容执行相应的操作。
### 详细功能:
支持命令的批量处理，能够同时向多个次级节点发送命令，并同步等待结果。
支持命令的异步处理，在发送命令时不会阻塞主线程，可以继续执行其他任务。
#### 代码示例:
```class PrimaryCommands {
public:
void send_command(MultiGPUCommand command) {
// 构造并发送命令到次级节点
}
void process_command(Buffer buffer) {
// 处理接收到的命令
}
}; 
```
## 3. Router 类
### 功能:
Router 类是主节点与次级节点之间的桥梁，负责在收到来自主节点或次级节点的命令后，将这些命令正确地路由到目标节点进行处理。它保证了命令的精准传递，使得系统能够有效地进行分布式操作。
在一个多 GPU 环境中，命令可能需要在多个节点之间传递。Router 的主要任务是根据每个命令的目标，将其路由到正确的目标节点。它起到了流量控制的作用，确保系统中每个节点都能接收到它所需的命令并执行。
### 关键方法:
route_command(Buffer buffer, std::shared_ptr secondary): 将命令从主节点
转发到指定的次级节点。该方法确保命令根据目标的不同路由到正确的次级节点。
### 详细功能:
能够根据命令的不同类型和目标节点，动态调整路由策略，支持灵活的任务
分配。
提供一种集中管理的方式，使得主节点能够统一管理命令的路由，而不必直
接与每个次级节点交互。
#### 代码示例:
```class Router {
public:
Router() {}
void route_command(Buffer buffer, std::shared_ptr<Secondary>
secondary) {
// 将命令路由到正确的次级节点
secondary->process_command(buffer);
}
};
```
## 4. Secondary 类
### 功能:
Secondary 类代表了系统中的次级节点，负责执行从主节点或其他次级节点接收到的命令。每个次级节点可以与其他节点进行通信，接收指令并执行相应的操作。
次级节点通常负责并行处理计算任务、模拟任务或其他资源密集型任务。它通过网络与主节点或其他次级节点进行通信，接收任务并反馈执行结果。
### 关键方法:
Connect(): 建立与主节点或其他次级节点的 TCP 连接，准备接收来自其他
节点的命令和数据。
Stop(): 关闭与其他节点的连接，释放相关资源，通常用于系统关闭或节点
退出时调用。
Write(Buffer buffer): 向其他节点发送数据或命令，将数据通过 TCP 协议发
送到目标节点。
### 详细功能:
支持与多个节点的并发连接，能够同时处理来自不同节点的命令。
提供灵活的命令执行机制，能够在接收到命令后根据不同的策略执行任务。
#### 代码示例:
```class Secondary {
public:
Secondary(boost::asio::ip::tcp::endpoint ep,
SecondaryCommands::callback_type callback)
: _socket(io_context), _endpoint(ep), _callback(callback) {}
void Connect() {
// 连接到 TCP 端点
}
void Stop() {
// 停止与节点的通信并断开连接
}
void Write(Buffer buffer) {
// 向其他节点发送数据
}
private:
boost::asio::ip::tcp::socket _socket;
boost::asio::ip::tcp::endpoint _endpoint;
SecondaryCommands::callback_type _callback;
};
```
## 5. SecondaryCommands 类
### 功能:
SecondaryCommands 类是次级节点的命令处理中心。它负责解析接收到的命令并执行相应操作，同时通过回调机制将结果反馈给主节点或其他调用者。次级节点接收到的命令可能是各种计算任务或操作指令。
SecondaryCommands 需要根据命令类型选择合适的执行方法，并在完成后通知结
果。
### 关键方法:
set_secondary(std::shared_ptr secondary): 关联一个次级节点，使得
SecondaryCommands 可以访问并控制该节点的操作。
set_callback(callback_type callback): 设置命令执行后的回调函数，用于处理
执行结果。
process_command(Buffer buffer): 解析传入的命令，并调用合适的回调函数
通知命令的执行结果。
### 详细功能:
支持命令的异步执行，能够在命令执行过程中继续处理其他任务。
提供灵活的回调机制，能够根据不同的命令类型，调用不同的处理逻辑。
#### 代码示例:
```class SecondaryCommands {
public:
using callback_type = std::function<void(MultiGPUCommand,
carla::Buffer)>;
void set_secondary(std::shared_ptr<Secondary> secondary) {
_secondary = secondary;
}
void set_callback(callback_type callback) {
_callback = callback;
}
void process_command(carla::Buffer buffer) {
// 解析命令并调用回调函数
}
private:
std::shared_ptr<Secondary> _secondary;
callback_type _callback;
};
```
## 类之间的关系
Primary 与 Secondary：
Primary 是系统的主节点，负责发送命令给 Secondary。
Secondary 是次级节点，负责接收命令并执行相应操作。
PrimaryCommands 与 SecondaryCommands：
PrimaryCommands 负责主节点发送命令的逻辑。
SecondaryCommands 负责次级节点接收命令并执行的逻辑。
Router：Router 是 Primary 和 Secondary 之间的中介，负责将命令从主节点
路由到次级节点。
Listener：Listener 负责监听来自其他节点的连接请求，并建立网络连接。
IncomingMessage：IncomingMessage 负责接收和解析来自其他节点的消息，将
其传递给相应的命令处理器。
通过这些类的协作，CARLA 能够高效地在多 GPU 环境中进行分布式控制和数据
通信。

参考文档：[https://openhutb.github.io/carla_doc/adv_multigpu/](https://openhutb.github.io/carla_doc/adv_multigpu/)
