# Traffic Manager

## 什么是 Traffic Manager？

Traffic Manager（简称 TM）是 CARLA 模拟器中用于控制自动驾驶车辆行为的模块。它通过模拟真实的城市交通环境，生成符合交通规则的车辆行为，包括路径规划、避障、交通信号响应等。用户可以通过 TM 实现对车辆行为的细粒度控制，以满足特定的训练或测试需求。

## 架构概览

TM 的架构由多个独立的阶段组成，每个阶段在不同的线程中运行，确保高效的并行处理。主要组件包括：

- **代理生命周期与状态管理（ALSM）**：每帧扫描仿真环境中所有 Agent（如车辆和行人），记录其 ID、坐标、朝向、速度等信息。新增的车辆将被注册为 TM 控制对象，已销毁的车辆将被移除。

  ALSM 模块的核心是 `ALSM::Update()` 方法，它每帧执行以下任务：

  - 获取世界时间戳和参与者列表。
  - 判断已注册和未注册的参与者是否已经销毁，并移除其状态（通过 `IdentifyDestroyedActors()` 和 `RemoveActor()` 实现）。
  - 检查是否有新车辆或行人加入（`IdentifyNewActors()`），并注册新的对象到系统。
  - 对于注册车辆，更新其状态、位置、速度、交通灯状态等，并按是否在“混合物理模式”中启用物理仿真（`UpdateRegisteredActorsData()` + `UpdateData()`）。
  - 若某车辆长时间静止、非英雄角色且时间间隔满足条件，执行自动销毁。
  - 若处于 OSM 模式，按需销毁“marked_for_removal”中的参与者。
  - 更新所有未注册对象的状态并写入仿真状态系统（`UpdateUnregisteredActorsData()`）。
  - 附加判断函数 `IsVehicleStuck()` 用于判断车辆是否卡住。

  ALSM 模块通过维护 `idle_time`、`hero_actors`、`unregistered_actors` 等状态表，结合定位、碰撞、交通灯、运动规划等阶段模块，形成了一个生命周期完整、响应实时的 Agent 管理系统。

- **车辆注册表**：是 TM 的核心索引系统，用于记录所有被 Traffic Manager 控制的车辆及其行为配置，如目标速度、车距、变道偏好等。它确保每辆车都能在后续计算阶段正确访问和更新状态。
- **模拟状态缓存**：为了提升性能，TM 并不会在每个阶段实时从仿真世界中提取状态，而是将所有车辆和行人的信息缓存下来，使得各计算阶段可以共享一致且高效的数据快照。
- **控制循环（主调度器）**：驱动 TM 的主循环机制，协调定位、碰撞检测、交通规则判断、运动控制等阶段，并在每一步完成后统一调度进入下一阶段，保证各线程的时序一致性。
- **内存地图与网格路径**：将仿真地图转换为更高效的拓扑结构，包括道路连接关系、车道几何等，并在该结构上生成稀疏网格路径（waypoints）供车辆导航。
- **路径缓存与车辆轨迹（PBVT）**：为每辆车维护一个未来若干秒的路径轨迹队列（Trajectory Buffer），其中的每个点都是基于当前位置规划出的可达路径，用于预测、避障和控制计算。
- **PID 控制器**：每辆车都通过其 PID 控制器根据期望路径点计算出实际控制命令（油门、刹车和方向盘），以最小化当前运动状态与路径目标之间的误差。
- **命令数组控制器**：在控制阶段结束后，所有车辆的控制指令被批量组织为一个控制命令数组，通过高效通道（如 client.apply_batch()）发送到 CARLA 服务器，实现高帧率控制。

## 控制循环的阶段

1. **定位阶段**：根据每辆车当前的位置信息，查找其前方预定路径上的一组路点（Waypoints）。路径长度根据车速动态调整，确保路径点能够覆盖车辆数秒钟内的预期移动范围。该阶段还会处理路径重建请求，以适应新注册车辆或突发路径变更。

2. **碰撞检测阶段**：分析当前路径点与其他动态对象（车辆、行人）的可能交叉情况，构建临时的局部避障图。若检测到潜在碰撞，会在车辆控制中插入速度限制、停车或绕行建议。

3. **交通灯阶段**：识别路径中出现的红绿灯、停车标志、优先通行规则等信息。此阶段依据道路拓扑结构判断当前车道是否被交通信号所控制，并根据灯色状态决定是否停车或等待。

4. **运动规划阶段**：使用 PID 控制器将车辆当前位置与路径点之间的差值转化为控制命令（油门、刹车、方向盘）。控制器不断调整以最小化路径误差，同时结合速度限制和行为参数（如期望速度、是否变道）进行动态调整。

5. **车灯控制阶段**：根据运动规划的结果（如是否刹车、转向）设置车辆灯光状态。例如：当车辆计划左转时，激活左转向灯；减速时激活刹车灯；在夜间或低光照区域自动打开前照灯等。

## 使用 Traffic Manager

### 创建和配置

```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()

# 设置同步模式
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 获取 Traffic Manager 实例
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)
```

### 控制车辆行为

```python
# 获取车辆蓝图
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*')[0]

# 获取地图上的生成点
spawn_points = world.get_map().get_spawn_points()

# 生成车辆
vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])

# 启用自动驾驶
vehicle.set_autopilot(True)

# 设置车辆行为参数
traffic_manager.ignore_lights_percentage(vehicle, 30.0)  # 30% 的概率忽略红灯
traffic_manager.set_desired_speed(vehicle, 20.0)         # 设置期望速度为 20 m/s
```

## 高级功能

- **确定性模式**：通过设置随机种子，确保每次模拟结果一致，便于测试和验证。
- **混合物理模式**：在车辆远离 Ego 车辆时，使用简化的物理模型，提高模拟效率。
- **多 Traffic Manager 实例**：支持在同一模拟中运行多个 TM 实例，分别控制不同的车辆组。
- **同步模式**：确保所有车辆在每个模拟步长中同步更新，适用于需要严格时间控制的场景。

## 总结

Traffic Manager 是 CARLA 模拟器中强大的工具，提供了对自动驾驶车辆行为的全面控制。通过合理配置 TM，可以模拟各种复杂的交通场景，满足自动驾驶系统的训练和测试需求。
