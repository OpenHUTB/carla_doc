# CARLA 客户端文档目录

## 一、[概述](#概述)

## 二、[主要类与功能](#主要类与功能)
- [2.1 Client 类](#21-client-类)
- [2.2 Actor 类](#22-actor-类)
- [2.3 ClientSideSensor 类](#23-clientsidesensor-类)
- [2.4 TrafficManager 类](#24-trafficmanager-类)
- [2.5 BlueprintLibrary 与 ActorBlueprint 类](#25-blueprintlibrary-与-actorblueprint-类)
- [2.6 Sensor 类与传感器使用（以摄像头为例）](#26-sensor-类与传感器使用以摄像头为例)
- [2.7 World 类](#27-world-类)

## 三、[扩展模块](#扩展模块)
- [3.1 DebugHelper 模块](#31-debughelper-模块carla-客户端调试可视化工具)
- [3.2 FileTransfer 模块](#32-filetransfer-模块carla-客户端的文件读写与缓存机制)
- [3.3 Junction 模块](#33-junction-模块carla-道路交叉口junction建模与访问)
- [3.4 LaneInvasion 模块](#34-laneinvasion-模块carla-车道入侵lane-invasion检测与处理)
- [3.5 Light 模块](#35-light-模块carla-中的交通灯与照明灯控制接口)
- [3.6 Vehicle 模块](#36-vehicle-模块carla-车辆实体控制与状态管理)

## 四、[类之间的调用关系](#类之间的调用关系)

## 五、[总结](#总结)

## 六、[参考链接](#参考链接)



# **概述**

CARLA 是一个开源的自动驾驶模拟平台，提供了丰富的 API 以支持仿真环境的控制、车辆的管理以及自动驾驶算法的验证。客户端部分的 API 主要通过 `Client` 类与模拟器进行交互，执行仿真控制、传感器管理、交通流管理等功能。该文档主要介绍 CARLA 客户端相关类的功能和使用方式。

## **主要类与功能**

### 1. `Client` 类

`Client` 类是与 CARLA 模拟器交互的核心类，提供了连接仿真器、加载世界、设置物理参数、控制传感器等功能。在 Carla 自动驾驶仿真平台中，Client 类是开发者与 Carla 服务器（服务端）交互的核心接口。它负责建立客户端与服务器之间的连接，并通过网络通信管理仿真世界中的实体（如车辆、传感器、行人等）。通过 Client 类，开发者可以灵活控制 Carla 仿真环境，实现从简单的车辆控制到复杂的多智能体协同仿真。

#### 核心作用

| 功能                 | 说明                                                                 |
|----------------------|--------------------------------------------------------------------|
| 连接仿真器           | 建立客户端与Carla服务器的TCP/IP连接，支持指定IP地址和端口号              |
| 加载/切换世界        | 动态加载不同地图（如Town01-Town10），支持世界状态重置和场景热更新          |
| 时间同步控制         | 设置固定时间步长或可变时间步长，控制仿真时钟与真实时间的同步比例            |
| 物理参数配置         | 调整重力系数、空气密度等物理参数，模拟不同环境条件下的车辆动力学特性        |
| 交通管理器控制       | 启停交通流，设置行人/车辆密度，配置NPC行为模式（保守/激进驾驶等）           |
| 传感器管理           | 生成/销毁摄像头、LiDAR、雷达等传感器，配置传感器参数（分辨率、FOV等）       |
| 天气系统控制         | 动态调整降水、积水、雾霾、太阳角度等天气参数，支持天气渐变过渡效果           |
| 蓝图库访问           | 提供车辆、传感器、行人的蓝图模板库，支持自定义属性过滤和参数化生成           |
| 快照(Snapshot)获取   | 捕获当前帧的世界状态信息，包含所有实体的位置、速度和物理特性                 |
| 录制与回放           | 支持场景录制（.log格式）和实时回放，可用于事故复现或测试用例自动化                 

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

### 4、LaneInvasion 模块：CARLA 车道入侵（Lane Invasion）检测与处理

`LaneInvasion` 是 CARLA 传感器系统中的一个事件检测模块，用于**监测车辆是否越过车道线**。该模块广泛用于驾驶策略评估、车辆行为监控以及安全规则验证，是自动驾驶测试中的重要工具之一。

#### 该模块的主要组成包括：

- `LaneInvasionSensor.h`：定义了传感器的基本结构与回调接口；
- `LaneInvasionSensor.cpp`：实现了车道线检测的逻辑；
- `LaneInvasionEvent.h / .cpp`：封装了每次触发的事件信息。

---

#### 类结构与设计

| 成员变量 | 说明 |
|----------|------|
| `_parent_actor` | 被绑定的车辆 Actor，用于检测其车道状态 |
| `_callback` | 用户注册的回调函数，在触发车道入侵时调用 |
| `_lane_markings` | 当前被侵入的车道线类型（如实线、虚线等） |

传感器作为 `Sensor` 的派生类，通过 CARLA 的 Actor 系统被附着到车辆上。每当车辆穿越车道标线时，该传感器便会生成一次事件，并触发回调。


#### 提供的接口函数

| 函数名 | 功能 |
|--------|------|
| `Listen(callback)` | 注册回调函数以处理车道入侵事件 |
| `Stop()` | 停止传感器监听 |
| `GetActor()` | 返回绑定的车辆 Actor |
| `GetLaneMarkings()` | 获取当前入侵事件中涉及的车道线类型列表 |

其中，`Listen` 是最关键的接口，用户可通过传入 Lambda 函数或回调对象，自定义入侵响应逻辑。

---

#### 示例：注册车道入侵事件监听器

```
auto sensor = world.SpawnActor(sensor_bp, transform, vehicle);
auto lane_invasion_sensor = boost::static_pointer_cast<carla::sensor::data::LaneInvasionEvent>(sensor);

sensor->Listen([](auto data) {
    auto event = boost::static_pointer_cast<carla::sensor::data::LaneInvasionEvent>(data);
    for (auto mark : event->GetCrossedLaneMarkings()) {
        std::cout << "Lane invasion detected: " << mark.TypeToString() << std::endl;
    }
});
```
#### 应用场景
 - 驾驶员违规行为监控：识别是否压线行驶，评估驾驶行为合规性；

 - 路径偏移检测：在路径规划中判断车辆是否偏离既定路线；

 - 规则执行引擎支持：结合交通规则仿真，在违反道路线规则时进行惩罚或报警；

 - 系统调试与可视化：在自动驾驶开发中用于调试控制策略，结合可视化界面标注压线轨迹。

#### 事件结构（LaneInvasionEvent）

每次入侵会生成一个 `LaneInvasionEvent` 对象，包含以下信息：

| 字段名                 | 类型                         | 说明                 |
|------------------------|------------------------------|----------------------|
| `crossed_lane_markings` | `std::vector<LaneMarking>`   | 被穿越的车道线集合   |
| `actor`                | `Actor`                      | 触发事件的车辆       |
| `timestamp`            | `Time`                       | 事件发生时间戳       |

#### 总结
`LaneInvasion` 模块为 CARLA 中提供了高效、可扩展的车道线检测机制，是自动驾驶测试流程中的重要一环。通过该模块，开发者可方便地监控车辆是否遵循车道规范，并可将入侵事件集成至评估、报警或控制逻辑中，显著提升仿真测试的准确性与现实性。

### 5、Light 模块：CARLA 中的交通灯与照明灯控制接口

`Light` 模块是 CARLA 客户端中用于操作和管理灯光对象（包括交通灯、街道灯、车灯等）的类。它通过封装 `LightManager` 的控制接口，允许用户**读取和设置灯光属性、状态、分组以及开关控制**，在交通仿真与可视化系统中扮演关键角色。

该模块主要由以下文件组成：

- `Light.h`：声明 `Light` 类的接口；
- `Light.cpp`：实现灯光状态的读取和设置功能；
- 依赖 `LightManager` 实现实际的灯光控制。

#### 类结构与设计

| 成员变量         | 说明 |
|------------------|------|
| `_id`            | 当前灯光的唯一标识符 |
| `_light_manager` | 指向 `LightManager` 的弱引用指针，确保灯光操作的上下文一致性 |

所有灯光状态操作均通过 `_light_manager` 执行，确保集中管理和同步执行。

#### 提供的接口函数

| 函数名                | 功能说明 |
|------------------------|-----------------------------|
| `GetColor()`           | 获取当前灯光颜色 |
| `GetIntensity()`       | 获取当前灯光亮度（强度） |
| `GetLightGroup()`      | 获取灯光所属分组 |
| `GetLightState()`      | 获取灯光完整状态（如颜色、开关、组别等） |
| `IsOn()` / `IsOff()`   | 判断灯光是否已开启或关闭 |
| `SetColor(Color)`      | 设置灯光颜色 |
| `SetIntensity(float)`  | 设置灯光亮度 |
| `SetLightGroup(group)` | 设置灯光所属分组 |
| `SetLightState(state)` | 设置完整灯光状态 |
| `TurnOn()` / `TurnOff()` | 控制灯光开关 |

#### 示例：设置灯光颜色与状态

```
auto light = some_actor.GetLight();

// 设置灯光颜色为红色
light.SetColor(carla::Color(255, 0, 0));

// 设置强度为5.0
light.SetIntensity(5.0f);

// 设置状态为开启状态并指定组别
carla::rpc::LightState state;
state.active = true;
state.intensity = 5.0f;
state.color = carla::Color(255, 255, 0);
state.group = carla::rpc::LightState::LightGroup::Street;

light.SetLightState(state);
```
#### 应用场景
 - 交通灯仿真控制：控制红绿灯的开关状态与颜色变换；
 - 街道照明模拟：控制夜间场景下街灯的亮度与颜色；
 - 视觉感知测试：模拟不同灯光环境用于测试自动驾驶感知算法；
 - 动态灯光效果渲染：在交互式仿真场景中创建逼真的灯光效果变化。

#### 模块特性与设计优势
 - 所有接口均通过 LightManager 管理，集中控制所有灯光对象；
 - 利用 weak_ptr 提高资源管理效率，避免循环引用；
 - 支持细粒度的灯光操作，如单独控制颜色、强度、分组等；
 - 可作为传感器、视觉系统或控制模块的辅助工具。

#### 总结
`Light `模块为 CARLA 中所有可控灯光对象提供了统一、灵活的控制接口。它结合 `LightManager `实现了强大的灯光仿真能力，
为真实感可视化和智能交通仿真提供关键支持。该模块对提升自动驾驶仿真环境的真实性和交互性具有重要意义。

### 6、Vehicle 模块：CARLA 车辆实体控制与状态管理

`Vehicle` 是 CARLA 模拟器中用于**表示和操作模拟场景中的车辆 Actor** 的基本类。该类接口完善，支持车辆的控制操作、自动驾驶切换、物理参数调用以及交通灯交互，是自动驾驶仿真中的核心组件之一。

该模块包括两部分文件：

- `Vehicle.h`：定义车辆类的基本结构和操作接口。
- `Vehicle.cpp`：实现了车辆控制逻辑和状态查询功能。

#### 类设计与主要成员

| 成员变量 | 功能说明 |
|--------------|----------------|
| `_control` | 存储最后一次应用的车辆控制指令 |
| `_is_control_sticky` | 表示是否使用“粘性控制”（即无需重复上传一样的指令） |

`Vehicle` 类继承自 `Actor`，并使用多类型符合 CARLA 控制器标准。

#### 公共接口函数

| 函数名 | 功能说明 |
|-----------|----------------------------|
| `SetAutopilot(bool, tm_port)` | 设置是否使用自动驾驶 (与 TrafficManager 联动) |
| `ApplyControl()` | 应用基础车辆控制指令 (加速、转向、刹车) |
| `ApplyAckermannControl()` | 应用 Ackermann 转向控制 |
| `GetControl()` | 获取当前车辆控制指令 |
| `ApplyPhysicsControl()` | 设置车辆物理参数 (重量、碳体积等) |
| `GetTelemetryData()` | 获取车辆遥测数据 (位置、速度等) |
| `OpenDoor()` / `CloseDoor()` | 操作车门打开或关闭 |
| `SetLightState()` / `GetLightState()` | 设置和查询车灯状态 |
| `SetWheelSteerDirection()` / `GetWheelSteerAngle()` | 设置/获取车轮转向角 |
| `EnableCarSim()` / `UseCarSimRoad()` | 使用 CarSim 车辆模型和道路 |
| `EnableChronoPhysics()` / `RestorePhysXPhysics()` | 切换车辆物理引擎 (Chrono 或 PhysX) |
| `IsAtTrafficLight()` / `GetTrafficLight()` | 判断是否被交通灯影响，并获取当前交通灯对象 |
| `GetSpeedLimit()` | 获取当前车辆速度限制 |
| `GetFailureState()` | 获取车辆故障状态 |


#### 示例：启用自动驾驶
```
// 车辆初始化
carla::client::Vehicle vehicle = ...;

// 启用自动驾驶
vehicle.SetAutopilot(true);
```
#### 应用场景
 - 自动驾驶模拟控制：接入 TrafficManager 实现汽车流量自动控制
 - 车辆系统调用分析：通过接口获取操作、物理和状态数据
 - 高级路径控制实验：配合 Ackermann 控制器进行路径跳转与车轮方向控制

#### 总结
`Vehicle` 模块是 CARLA 中表示和操作车辆的核心类。它包括了驾驶、控制、物理反应和交通环境交互等方面，
是完成实时交通模拟与自动驾驶系统组装的基石。


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
