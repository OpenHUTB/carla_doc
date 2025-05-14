


## **概述**

CARLA 是一个开源的自动驾驶模拟平台，提供了丰富的 API 以支持仿真环境的控制、车辆的管理以及自动驾驶算法的验证。客户端部分的 API 主要通过 `Client` 类与模拟器进行交互，执行仿真控制、传感器管理、交通流管理等功能。该文档主要介绍 CARLA 客户端相关类的功能和使用方式。

## **主要类与功能**

### 1. `Client` 类

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
  
### 5. `BlueprintLibrary` 与 `ActorBlueprint` 类
在 CARLA 客户端中，`BlueprintLibrary` 类用于提供创建行为体（Actor）的模板集合，而 `ActorBlueprint` 则描述具体实体的构造配置参数。
#### 主要功能

- `BlueprintLibrary`

  - 获取可创建的实体蓝图列表（如车辆、传感器等）
  - 支持通过关键词过滤蓝图
  
- `ActorBlueprint`
  - 表示单个实体的构造参数（如车辆类型、颜色，传感器分辨率等）
  - 可设置和查询各类属性值

#### 示例

```
auto blueprint_library = world.GetBlueprintLibrary();

// 选择一辆特斯拉车型的 blueprint
auto vehicle_blueprint = (*blueprint_library)->Find("vehicle.tesla.model3");

// 设置车辆颜色
vehicle_blueprint->SetAttribute("color", "255,0,0");

// 从地图上获取一个出生点
auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
auto transform = spawn_points[0];

// 生成车辆 Actor
auto vehicle_actor = world.SpawnActor(*vehicle_blueprint, transform);

```

### 6. `Sensor` 类与传感器使用（以摄像头为例）
CARLA 支持多种传感器类型，例如摄像头、激光雷达、碰撞传感器等，传感器均继承自 `ClientSideSensor` 类。

#### 创建与附加传感器的步骤

 - 从 BlueprintLibrary 获取传感器蓝图

 - 设置传感器参数（如图像分辨率、视场角等）

 - 附加到目标 Actor（如车辆）

 - 注册数据回调处理函数
 
### 示例（以摄像头为例）

```
auto camera_bp = blueprint_library->Find("sensor.camera.rgb");
camera_bp->SetAttribute("image_size_x", "800");
camera_bp->SetAttribute("image_size_y", "600");
camera_bp->SetAttribute("fov", "90");

// 定义相对于车辆的位置（车顶前部）
Transform camera_transform(Location(1.5f, 0.0f, 2.4f));

// 创建摄像头并附加到车辆
auto camera = world.SpawnActor(*camera_bp, camera_transform, vehicle_actor);

// 注册数据处理回调
std::static_pointer_cast<ClientSideSensor>(camera)->Listen([](auto data) {
    auto image = std::static_pointer_cast<sensor::data::Image>(data);
    image->SaveToDisk("output/%06d.png", image->GetFrame());
});

```

### 7. `World` 类

World 类表示一个仿真世界实例，它封装了场景中的地图、实体、天气、环境参数等。通过 `Client::LoadWorld()` 可获得 `World` 对象。

#### 常用方法

 - `GetActors()`：获取当前世界中所有实体

 - `GetBlueprintLibrary()`：获取实体蓝图列表

 - `SpawnActor()`：创建新实体

 - `Tick()`：推进世界一帧（同步模式）

 - `SetWeather()`：设置天气（如雨、雾、阳光）

 - `GetMap()`：获取当前地图信息

#### 示例（以设置天气为例）
```
carla::rpc::WeatherParameters weather;
weather.cloudiness = 80.0f;
weather.precipitation = 30.0f;
weather.sun_altitude_angle = 45.0f;
world.SetWeather(weather);
```

---
## **扩展板块**
### 1、DebugHelper 模块：CARLA 客户端调试可视化工具

`DebugHelper` 是 CARLA `client` 部分的重要组件之一，主要用于**在仿真世界中可视化各种调试信息**，帮助开发者直观地了解仿真环境中各类实体和交互的运行状态。

该模块基于 `rpc::DebugShape` 实现了多个图形绘制方法，包括点、线、箭头、边界框和文本等。所有形状通过调用统一的模板函数 `DrawShape` 实现绘制，绘图内容会被添加到当前模拟剧集中（`EpisodeProxy`）。

#### 功能概览

`DebugHelper` 提供以下绘图接口，每个函数内部都调用 `DrawShape` 模板函数生成具体的 `rpc::DebugShape`：

- `DrawPoint`：绘制普通点
- `DrawHUDPoint`：绘制 HUD 点（投影到 HUD 层的点）
- `DrawLine`：绘制线段
- `DrawHUDLine`：绘制 HUD 线段
- `DrawArrow`：绘制带箭头的线
- `DrawHUDArrow`：绘制 HUD 箭头
- `DrawBox`：绘制三维边界框
- `DrawHUDBox`：绘制 HUD 边界框
- `DrawString`：在空间中绘制文字信息

所有方法均支持设置：

- **颜色（Color）**
- **生命周期（life_time）**
- **是否为持久绘图（persistent_lines）**

### 内部实现结构

所有绘图方法都通过调用如下统一模板函数实现绘制：

```
template <typename T>
static void DrawShape(detail::EpisodeProxy &episode,
                      const T &primitive,
                      rpc::Color color,
                      float life_time,
                      bool persistent_lines) {
  const Shape shape{primitive, color, life_time, persistent_lines};
  episode.Lock()->DrawDebugShape(shape);
}
```
该函数接收一个图形 primitive（如点、线、框等），并封装为 DebugShape 后添加到当前仿真剧集中。
#### 使用场景
 - 在开发过程中验证 AI 行为（例如绘制导航路线）

 - 调试地图元素（如车道线、交通标志检测）

 - 实时可视化传感器数据结果

 - 在科研或演示过程中进行辅助说明

#### 小结
DebugHelper 是 CARLA 提供的轻量级可视化调试接口，极大提升了开发效率。它通过统一封装和生命周期控制，使用户可以灵活地向仿真场景中添加临时的可视图形，方便排查问题、验证逻辑或演示系统能力。


### 2、FileTransfer 模块：CARLA 客户端的文件读写与缓存机制

`FileTransfer` 是 CARLA 客户端中的静态类模块，专用于**文件的本地读写、缓存管理以及路径设置**等操作。其设计初衷是为了在客户端环境中缓存或传输所需的文件数据，如地图片段、传感器结果或其他模拟资源，减少不必要的重复下载或传输。

该模块主要由两个文件组成：

- `FileTransfer.h`：定义了该类的接口及核心成员变量。
- `FileTransfer.cpp`：实现了各个接口的具体功能。

---

#### 核心功能接口

| 接口名 | 功能描述 |
|--------|----------|
| `SetFilesBaseFolder(path)` | 设置本地文件缓存基础目录（必须是非空路径），自动添加尾部斜杠 |
| `GetFilesBaseFolder()` | 获取当前设置的基础路径 |
| `FileExists(file)` | 检查指定文件是否存在于当前版本的缓存目录中 |
| `WriteFile(path, content)` | 将二进制内容写入指定路径文件中，并自动创建所需目录结构 |
| `ReadFile(path)` | 从指定路径读取二进制文件，返回字节向量 |

---

#### 文件路径结构与平台兼容性

模块中根据操作系统自动设置默认的缓存根目录 `_filesBaseFolder`：

- Windows 系统：`%USERPROFILE%/carlaCache/`
- Linux / macOS：`$HOME/carlaCache/`

所有操作文件的路径会被扩展为如下格式：
<基础路径>/<CARLA版本>/<目标文件>

其中 `<CARLA版本>` 通过调用 `carla::version()` 获取。

#### 路径验证与创建逻辑

在 `WriteFile` 函数中，使用了 CARLA 内部的 `FileSystem::ValidateFilePath()` 方法来确保路径存在并合法，如不存在会自动创建相应目录。这一机制增强了程序的健壮性与用户的易用性。


#### 示例：写入与读取缓存文件

```
std::vector<uint8_t> data = {0x01, 0x02, 0x03};
FileTransfer::WriteFile("sensor/lidar.bin", data);

std::vector<uint8_t> result = FileTransfer::ReadFile("sensor/lidar.bin");

```
该段代码将一个小型字节向量写入缓存路径，并随后读取回来，展示了 FileTransfer 简洁易用的接口风格。

#### 设计特点
 - 所有函数为静态方法（类不能被实例化）

 - 内部缓存路径根据系统平台自动设定

 - 所有路径操作均以当前 CARLA 版本为中间层级，便于区分不同版本的数据

 - 支持二进制文件的高效读写

 - 利用 std::vector<uint8_t> 管理数据，兼容性好

#### 总结
`FileTransfer `模块提供了 CARLA 客户端中重要的本地文件操作机制，适合用于缓存地图、传感器数据、AI模型结果等静态资源。通过该模块可以显著提高数据复用效率、降低文件操作出错率，并为系统性能和用户体验带来明显提升。

### 3、Junction 模块：CARLA 道路交叉口（Junction）建模与访问

`Junction` 是 CARLA 客户端地图模块中的一个关键类，用于**表示道路网络中的交叉口区域**。该类提供对交叉口 ID、边界范围和交叉区域内道路点（Waypoint）信息的访问能力，通常被用于路径规划、交通规则模拟与可视化渲染等功能中。

该模块由以下两个文件组成：

- `Junction.h`：定义了类的结构与接口。
- `Junction.cpp`：实现了其功能逻辑。


#### 类结构与设计

| 成员变量 | 说明 |
|----------|------|
| `_id` | 交叉口在道路拓扑中的唯一标识 |
| `_bounding_box` | 表示交叉口在世界坐标中的空间范围 |
| `_parent` | 指向所属 `Map` 的共享指针，便于访问地图全局信息 |

此外，`Junction` 类继承了 `EnableSharedFromThis`，并被标记为 `NonCopyable`，以确保对象管理的正确性与唯一性。


#### 构造函数与访问控制

构造函数 `Junction(SharedPtr<const Map>, const road::Junction *)` 被设为 `private`，仅允许 `Map` 类作为友元进行构造。这种设计符合“只允许地图构建交叉口对象”的意图，防止外部模块随意构造非法交叉口。

---

### 提供的接口函数

| 函数名 | 功能 |
|--------|------|
| `GetId()` | 返回当前交叉口的 ID |
| `GetBoundingBox()` | 返回交叉口所覆盖的三维边界框（`geom::BoundingBox`） |
| `GetWaypoints(type)` | 返回与该交叉口相关的所有 `Waypoint` 对，支持根据车道类型过滤，如 `Driving`、`Shoulder` 等 |

其中 `GetWaypoints` 默认仅返回行驶车道（`Driving`）的相关路径点对，调用时可按需调整车道类型参数。


#### 示例：获取交叉口路径点对

```
auto junction_list = map->GetJunctions();
for (auto& junction : junction_list) {
    auto wp_pairs = junction->GetWaypoints(road::Lane::LaneType::Driving);
    for (auto& pair : wp_pairs) {
        auto entry = pair.first;
        auto exit = pair.second;
        // 可用于路径规划或交通仿真
    }
}
```
#### 应用场景
 - 获取交叉口覆盖区域，用于渲染、碰撞检测或自动驾驶决策；

 - 获取进入与离开交叉口的路径点对，实现行为规划（如左转、右转、直行）；

 - 通过 Junction::GetId() 与道路拓扑结构联动，解析交通节点关系。

#### 总结
`Junction` 类为 CARLA 中交叉口建模提供了清晰、面向对象的封装方式。它作为 Map 类的组成部分，隐藏了底层拓扑数据的复杂性，同时提供了丰富的 API 支持，是交通仿真与路径分析不可或缺的模块。








---

## **类之间的调用关系**

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
