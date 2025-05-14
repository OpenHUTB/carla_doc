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

**车辆注册表（Vehicle Registry）**：是 Traffic Manager（TM）中用于管理所有被控制车辆的核心数据结构。该模块维护一个由车辆 ID 映射到车辆属性与控制参数的索引表，确保在整个仿真过程中 TM 能够高效、准确地读取和更新每辆车的状态。其主要职责包括：

- **车辆注册与注销**：当新车辆被识别并纳入 TM 控制时，其 ID 和初始配置被写入车辆注册表；当车辆被销毁或移除时，其信息也从注册表中清除。
- **行为参数存储与访问**：每辆车都关联一组行为参数，例如期望速度、跟车距离、变道偏好、交通信号响应概率等。这些参数在运行时可通过接口动态修改，支持个性化行为建模。
- **高效索引机制**：通过哈希表维护车辆 ID 与车辆对象/状态的映射关系，提供常数时间复杂度的读写操作，保证在大规模仿真中依然具有良好的性能。
- **线程安全支持**：通过原子操作或加锁机制，确保在并发读取或更新过程中数据一致性，支持多线程环境下的 TM 控制循环。
- **与阶段模块的数据同步**：注册表中车辆的 ID 是后续各阶段（如定位、碰撞检测、运动规划）访问数据的入口，确保车辆状态在各处理阶段的一致性与连贯性。
- **车辆状态快照支持**：部分实现中，车辆注册表还可缓存车辆的静态信息（如尺寸、类型）和部分动态状态，供控制器或外部模块使用。

示例行为配置接口（伪代码）：

```cpp
vehicle_registry.SetDesiredSpeed(vehicle_id, 15.0f);      // 设置目标速度
vehicle_registry.EnableAutoLaneChange(vehicle_id, true);  // 启用自动变道
vehicle_registry.SetIgnoreLightsProbability(vehicle_id, 0.3f); // 设置30%的概率忽视红灯
```

总之，车辆注册表在 TM 中扮演类似“车辆数据库”的角色，是连接外部控制逻辑与内部状态维护的桥梁，其设计直接影响 Traffic Manager 的效率与行为灵活性。

**模拟状态缓存（Simulation State Cache）**：该模块是 Traffic Manager 各阶段之间共享交通参与者状态的核心机制。为了避免每个模块重复从 CARLA 世界读取状态信息，模拟状态缓存提供了一种轻量、线程安全、时间一致的状态存取方式。其主要作用包括：

- **状态快照机制**：每帧开始时，ALSM 或状态采集模块会遍历所有车辆与行人，提取其位置、朝向、速度、交通灯状态等动态数据，并记录为快照。该快照在当前帧内固定不变，供各阶段模块读取。
- **静态属性记录**：除了动态状态，系统还会记录每个对象的类型（如车辆或行人）、边界尺寸等静态属性，用于后续阶段的行为建模与碰撞预测。
- **物理仿真模式支持**：缓存中记录了每个参与者是否启用了物理仿真（EnablePhysics 标记），该信息由 ALSM 根据混合物理模式判断决定，供运动控制或轨迹预测模块使用。
- **交通灯状态缓存**：车辆是否处于红灯控制、是否正在红灯前停车等状态也被同步缓存，避免重复查询 Vehicle API，提升性能。
- **接口丰富**：提供高效接口用于：
  - 获取车辆当前位置、速度、朝向（`GetLocation()`、`GetVelocity()`、`GetTransform()`）
  - 判断是否启用物理（`IsPhysicsEnabled()`）
  - 更新/添加参与者状态（`UpdateKinematicState()`、`AddActor()`）
  - 获取交通灯状态（`GetTLS()`）

示例接口使用（伪代码）：

```cpp
if (state_cache.ContainsActor(vehicle_id)) {
    auto loc = state_cache.GetLocation(vehicle_id);
    auto vel = state_cache.GetVelocity(vehicle_id);
    if (!state_cache.IsPhysicsEnabled(vehicle_id)) {
        // 使用估算速度进行运动规划
    }
}
```

总体而言，模拟状态缓存作为 TM 的“感知中枢”，为系统提供高一致性、高并发的状态访问支持，是实现高性能并行仿真的关键基础模块。

**控制循环（主调度器）**：主调度器是 Traffic Manager 的核心执行引擎，它以固定频率驱动各阶段模块协同工作，完成每一帧的交通逻辑处理流程。调度器以流水线方式组织模块调用，确保数据流与控制流的时序一致性，并协调多线程处理过程，提升整体仿真效率。其主要职责包括：

- **阶段调度机制**：主调度器以帧为单位执行控制流程，每帧包含一系列固定顺序的阶段模块（如定位、碰撞检测、交通灯判断、运动控制、车灯控制等）。每个阶段均在独立线程运行，通过 barrier 或任务队列机制统一调度。
- **模块执行顺序保障**：确保每个阶段开始前，其依赖的上游阶段已经完成。例如必须先完成定位模块，后续阶段才能获取车辆的路径信息。
- **状态同步与传递**：调度器负责在阶段之间传递必要的车辆状态与路径缓存数据，如 TrajectoryBuffer、KinematicState 等，保证数据流的完整性。
- **异常处理与故障恢复**：在任意阶段出现异常（如车辆数据缺失、路径断裂等）时，主调度器会根据设定策略降级处理或跳过当前帧，以保证仿真连续性与稳定性。
- **性能统计与帧控制**：记录每阶段的执行耗时用于性能监控，并根据同步模式（sync mode）或固定步长设定统一推进仿真时间步长（delta time）。

示例调度流程（伪代码）：

```cpp
for (each simulation frame) {
    ALSM.Update();                 // 更新参与者生命周期与状态缓存
    LocalizationStage.Run();      // 定位模块执行
    CollisionStage.Run();         // 碰撞检测
    TrafficLightStage.Run();      // 交通灯判断
    MotionPlannerStage.Run();     // 路径跟踪与运动规划
    VehicleLightStage.Run();      // 根据规划设置灯光
    ApplyControlCommands();       // 汇总控制命令，应用至服务器
}
```
控制循环作为 Traffic Manager 的主心骨，串联起整个行为决策链条，其效率与鲁棒性直接决定了多车自动驾驶仿真的实时性与稳定性。
 
**内存地图与网格路径（Local Map & Waypoint Grid）**：该模块负责将 CARLA 原始地图数据转化为高效结构化的拓扑图与稀疏导航路径点（Waypoints），用于支持定位、路径规划与避障等模块的快速查询与插值计算。它是构建交通规则感知与路径跟踪基础的核心组件之一，主要功能包括：

- **地图解析与拓扑构建**：读取 CARLA 世界地图并转换为图结构，节点表示车道中的关键位置，边表示车道的连接关系，形成有向图用于路径搜索与行为建模。
- **路点稀疏采样与数据封装**：使用固定间隔（例如 1 米）采样方式在道路中心线生成稀疏路点（SimpleWaypoint），每个路点对象包含：
  - 三维位置与朝向（Transform）
  - 所属车道与道路 ID
  - 限速、车道宽度、导航标签（如是否为交叉口、是否可变道）
  - 前向/后向邻接路点、相邻车道路点（左右变道）
- **导航路径构建与裁剪**：支持基于全局导航目标生成主路径（Global Route Plan）与局部路径（Local Trajectory），并根据车辆实时状态动态裁剪前向 N 秒或 N 米的路径段用于控制模块跟踪。
- **交通语义融合**：为路点附加交通灯、停车线、优先级等规则信息，在进行红灯判定、停车控制时使用，提升交通规则遵从性。
- **变道与交叉口支持**：
  - 查询当前路点是否为交叉口 `CheckJunction()`
  - 判断左右车道是否存在 `GetLeftLane()`, `GetRightLane()`
  - 检查是否允许变道 `IsLaneChangeAllowed()`
- **效率与线程安全设计**：路点以索引方式管理，支持高效空间查找（基于坐标 KD-Tree 或哈希表），同时接口可在多线程中并发调用，支持定位、轨迹生成和控制模块同时访问。

示例调用逻辑（伪代码）：

```cpp
SimpleWaypointPtr current_wp = local_map->GetWaypoint(vehicle_location);
std::vector<SimpleWaypointPtr> forward_path = current_wp->GetNextWaypointN(20);
for (auto wp : forward_path) {
    if (wp->HasTrafficLight()) {
        auto light_state = wp->GetBoundTrafficLight()->GetState();
        if (light_state == Red) {
            // 触发停车或减速逻辑
        }
    }
}
```

**路径缓存与车辆轨迹（PBVT, Path Buffer & Vehicle Trajectory）**：PBVT 是 Traffic Manager 中用于记录与更新每辆车未来运动轨迹的关键模块。它通过对局部路径的持续预测与缓存，为运动规划和避障模块提供决策支撑，确保系统在复杂场景中具备前瞻性和响应性。其主要功能包括：

- **轨迹缓冲区构建**：为每辆车维护一个轨迹缓冲队列（Trajectory Buffer），该队列包含从当前位置出发、沿参考路径前向若干米或若干秒的路径点，通常为 2-4 秒时域。
- **基于路点的路径预测**：轨迹点通常来源于内存地图生成的稀疏网格路点（Waypoints），并结合当前速度和加速度动态裁剪路径长度。
- **缓冲更新策略**：
  - 在车辆进入新位置或帧推进后，检测是否需要重建轨迹缓冲区。
  - 如果路径偏差或交通规则变化（如交通灯、障碍物出现）导致路径失效，则触发轨迹重规划。
- **轨迹点结构**：每个轨迹点不仅包含位置坐标，还记录目标速度、限速信息、变道指令、是否避障等高阶信息，供运动控制模块直接使用。
- **状态预判与多阶段使用**：
  - 碰撞检测阶段使用路径中的前向点进行障碍预测。
  - 运动控制阶段基于轨迹点生成 PID 控制输入。
  - 车灯控制模块判断是否需转向灯激活。
- **线程安全实现**：PBVT 模块支持并发访问，不同阶段可读取对应车辆的路径缓冲，同时支持局部重建。

示例调用逻辑（伪代码）：

```cpp
if (!trajectory_buffer.HasEnoughWaypoints(vehicle_id)) {
    auto new_path = planner.GeneratePathFromWaypoint(current_wp);
    trajectory_buffer.Update(vehicle_id, new_path);
}
auto next_target = trajectory_buffer.GetNextTarget(vehicle_id);
motion_controller.Track(next_target);
```



**PID 控制器（PID Controller）**：PID 控制器是 Traffic Manager 中将高层路径点转换为底层控制指令（油门、刹车、方向盘）的关键模块。每辆被控制的车辆都关联一个 PID 控制器实例，用于以闭环方式将目标轨迹点转化为精确的速度与方向控制，从而实现平滑、稳定的轨迹跟踪。其主要功能包括：

- **控制目标设定**：根据路径缓存模块提供的下一个期望轨迹点，设定车辆目标位置与速度作为控制目标。

- **误差计算**：

  - **纵向误差**：期望速度与当前速度之差，用于调节油门和刹车。
  - **横向误差**：车辆当前位置与期望轨迹中心线之间的横向偏移，用于调节方向盘角度。

- **PID 运算公式**：

  - 控制器使用标准 PID 结构（比例 P、积分 I、微分 D）进行误差反馈控制，避免控制抖动与震荡，同时可适配不同车辆模型参数。

  - 示例纵向控制：

    ```math
    a = Kp_v * e_v + Ki_v * ∫e_v dt + Kd_v * de_v/dt
    ```

  - 示例横向控制：

    ```math
    δ = Kp_d * e_d + Ki_d * ∫e_d dt + Kd_d * de_d/dt
    ```

- **输出控制命令**：将纵向输出映射为 throttle/brake，将横向输出映射为 steering，封装为控制指令发送至 CARLA 服务端。

- **特殊情况处理**：

  - 当路径点突然消失或预测点不连续时，控制器会平滑制动至停止。
  - 当限速调整或交通灯状态影响目标速度时，控制器根据反馈动态更新目标值。

- **自适应参数机制**（可选）：部分实现支持根据车速自动调整 PID 参数（如增益缩放），提升在高速或低速工况下的控制精度与响应性。

示例控制过程（伪代码）：

```cpp
auto target = trajectory_buffer.GetNextTarget(vehicle_id);
float throttle, brake, steer;
pid_controller.RunStep(target.location, target.speed, current_state, &throttle, &brake, &steer);
ControlCommand cmd{throttle, brake, steer};
```

PID 控制器通过对运动误差的持续反馈修正，为 Traffic Manager 提供了平稳、可调的控制机制，是连接路径规划与车辆执行的核心桥梁模块。
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
