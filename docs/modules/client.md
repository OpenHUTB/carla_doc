# **CARLA 客户端部分文档**


## 概述

CARLA 是一个开源的自动驾驶模拟平台，提供了丰富的 API 以支持仿真环境的控制、车辆的管理以及自动驾驶算法的验证。客户端部分的 API 主要通过 `Client` 类与模拟器进行交互，执行仿真控制、传感器管理、交通流管理等功能。该文档主要介绍 CARLA 客户端相关类的功能和使用方式。

## 主要类与功能

### 1. **Client 类**

`Client` 类是与 CARLA 模拟器交互的核心类，提供了连接仿真器、加载世界、设置物理参数、控制传感器等功能。
在 Carla 自动驾驶仿真平台中，Client 类是开发者与 Carla 服务器（服务端）交互的核心接口。它负责建立客户端与服务器之间的连接，并通过网络通信管理仿真世界中的实体（如车辆、传感器、行人等）。
通过 Client 类，开发者可以灵活控制 Carla 仿真环境，实现从简单的车辆控制到复杂的多智能体协同仿真。
1. 核心作用
功能	说明
连接管理	建立/断开与 Carla 服务器的 TCP/IP 连接，指定 IP 和端口（默认 localhost:2000）。
世界（World）管理	获取当前仿真世界的引用（World 对象），用于操作场景中的实体。
Actor 生命周期控制	创建（Spawn）、销毁（Destroy）车辆、传感器、行人等 Actor。
同步/异步模式控制	设置仿真步长（Timestep）、控制仿真运行模式（同步或异步）。
数据获取与订阅	接收传感器数据（如摄像头图像、激光雷达点云）、交通信息、地图数据等。
#### 构造函数

- `Client(const std::string &host, uint16_t port, size_t worker_threads = 0u)`
  - 构建一个与 CARLA 仿真器的客户端连接。
  - `host`：模拟器所在主机的 IP 地址。
  - `port`：与模拟器的连接端口。
  - `worker_threads`：使用的异步工作线程数，默认为 0（使用所有可用线程）。

#### 核心方法

- `std::string GetClientVersion() const`

  - 获取当前客户端的版本。

- `std::string GetServerVersion() const`

  - 获取当前连接的模拟器的版本。

- `std::vector<std::string> GetAvailableMaps() const`

  - 获取所有可用的地图名称。

- `World LoadWorld(std::string map_name, bool reset_settings = true, rpc::MapLayer map_layers = rpc::MapLayer::All) const`

  - 加载指定的仿真地图。

- `World ReloadWorld(bool reset_settings = true) const`

  - 重新加载当前仿真世界。

- `void SetTimeout(time_duration timeout)`

  - 设置与模拟器的网络超时时间。

- `TrafficManager GetInstanceTM(uint16_t port = TM_DEFAULT_PORT) const`

  - 获取当前模拟器中的交通管理实例（TrafficManager）。

- `void StartRecorder(std::string name, bool additional_data = false)`

  - 启动仿真数据记录器。

- `void StopRecorder()`

  - 停止仿真数据记录器。

- `World GenerateOpenDriveWorld(std::string opendrive, const rpc::OpendriveGenerationParameters &params, bool reset_settings = true) const`
  - 使用 OpenDrive 文件生成仿真世界。

### 2. `Actor` 类

`Actor` 类代表仿真中的任何实体（例如：车辆、行人等）。通过该类，用户可以控制行为体的位置、速度、碰撞、物理状态等。

#### 核心方法

- `geom::Location GetLocation() const`

  - 获取行为体的当前位置。

- `geom::Transform GetTransform() const`

  - 获取行为体的变换（位置与方向）。

- `void SetLocation(const geom::Location& location)`

  - 设置行为体的位置。

- `void SetTransform(const geom::Transform& transform)`

  - 设置行为体的变换（位置与方向）。

- `void AddImpulse(const geom::Vector3D& impulse)`

  - 向行为体添加冲量。

- `void AddForce(const geom::Vector3D& force)`

  - 向行为体施加力。

- `void SetSimulatePhysics(bool enabled = true)`

  - 启用或禁用行为体的物理模拟。

- `bool Destroy()`
  - 销毁行为体。

### 3. `ClientSideSensor` 类

`ClientSideSensor` 类继承自 `Sensor` 类，代表 CARLA 中的客户端传感器（如摄像头、激光雷达等）。

#### 核心方法

- `void Enable()`

  - 启用传感器。

- `void Disable()`

  - 禁用传感器。

- `bool IsEnabled() const`
  - 检查传感器是否启用。

### 4. 交通管理器（`TrafficManager`）

`TrafficManager` 类用于管理仿真中的交通流，控制交通行为，包括交通灯的状态、车辆的行驶路线、交通规则等。

#### 核心方法

- `void SetGlobalDistanceToLeadingVehicle(float distance)`

  - 设置车辆与前车的最小安全距离。

- `void SetVehicleDistanceToLeadingVehicle(float distance)`

  - 设置单个车辆与前车的最小安全距离。

- `void SetSynchronousMode(bool enabled)`
  - 启用或禁用同步模式，在同步模式下仿真步进是由用户控制的。

---

## 类之间的调用关系

### 以下是 CARLA 客户端部分类之间的调用关系图，展示了各个类如何交互以及它们之间的依赖关系。
![类关系](https://github.com/LangJing23/nuanxin_volunteer/blob/master/lei.png)


##### 调用关系说明

Client 类与其他类的交互：

> Client 类是与模拟器交互的主要接口，它通过方法如 LoadWorld()、GetAvailableMaps()、StartRecorder() 等来控制仿真世界。
> 
> Client 类使用 World 对象来表示仿真世界并操作其中的行为体（Actor 类）。
> 
> Actor 类与 Client 类的关系：
> 
> Actor 类表示仿真中的个体实体（如车辆、行人等），Client 类通过 Actor 对象控制仿真中的行为体（例如获取位置、速度，设置物理属性等）。
> 
> Actor 类也与传感器和交通管理相关联，可以通过 ClientSideSensor 和 TrafficManager 与交通流和传感器交互。
> 
> TrafficManager 类与 Client 类的关系：
> 
> TrafficManager 管理仿真中的交通流，Client 类通过 TrafficManager 提供的接口（如 SetSynchronousMode()）来控制仿真中的交通规则、车辆行为等。

ClientSideSensor 类与 Actor 类的关系：

> ClientSideSensor 类用于模拟仿真中的传感器，能够附加到 Actor 上，例如摄像头、激光雷达等。
> 
> 传感器可以用来获取 Actor 周围的环境数据（例如图像、点云等），以支持自动驾驶算法的训练与验证。

World 类与其他类的关系：

> World 类表示仿真中的一个世界，它与 Client 和 Actor 类紧密关联。World 提供对仿真环境的访问，可以控制仿真世界中的所有行为体和传感器。
> 
> World 还负责管理交通流、物理环境、天气等仿真环境因素。

## 总结

CARLA 客户端部分提供了与仿真环境交互的丰富 API，支持开发者控制仿真世界中的各种元素，包括行为体（Actor）、传感器（ClientSideSensor）和交通流（TrafficManager）等。通过 Client 类，开发者能够加载地图、获取世界状态、管理传感器、操作物理环境等。类之间的交互使得开发者能够在仿真环境中进行精确的控制和验证。

参考文档：[点击此处跳转](https://openhutb.github.io/carla_cpp/dir_64d7bb605f27024d49af86070bd3f0b6.html)
