@@ -0,0 +1,200 @@
 # CARLA 导航模块技术文档 (Navigation.cpp)
 
 ---
 
 ## 目录  
 1. [模块概述](#模块概述)  
 2. [核心功能](#核心功能)  
 3. [类与方法详解](#类与方法详解)  
 4. [坐标系统与数据结构](#坐标系统与数据结构)  
 5. [关键流程](#关键流程)  
 6. [配置参数](#配置参数)  
 7. [注意事项](#注意事项)  
 8. [示例代码](#示例代码)  
 9. [附录](#附录)  
 
 ---
 
 ## 模块概述  
 `Navigation` 是 CARLA 模拟器中实现智能体（行人、车辆）导航的核心模块，基于 **Recast/Detour** 库实现以下功能：  
 - **导航网格加载**：支持多瓦片（Tile）动态加载  
 - **路径规划**：基于区域类型（道路/草地）的路径搜索与优化  
 - **人群模拟**：支持500+代理的动态管理（避障、转向预测、分离行为）  
 - **动态障碍物处理**：车辆包围盒（OBB）的实时更新  
 - **多线程安全**：通过 `std::mutex` 保证关键操作原子性  
 
 ---
 
 ## 核心功能  
 ### 1. 导航数据加载  
 - **二进制文件解析**  
   ```cpp
   bool Load(const std::string &filename);  // 从文件加载
   bool Load(std::vector<uint8_t> content); // 从内存加载
   ```  
   - 支持多瓦片动态加载（`NavMeshSetHeader` 结构体）  
   - 内存管理：使用 `dtAllocNavMesh` 和 `dtFreeNavMesh` 分配/释放资源  
 
 ### 2. 路径规划  
 - **三步查询流程**  
   1. **最近多边形查询**：`findNearestPoly()`  
   2. **路径搜索**：`findPath()` 获取多边形序列  
   3. **路径优化**：`findStraightPath()` 转换为直线路径点  
   ```cpp
   bool GetPath(Location from, Location to, dtQueryFilter* filter, vector<Location>& path, vector<byte>& area);
   ```
 
 ### 3. 人群管理  
 | 方法                | 功能描述                               |
 |---------------------|----------------------------------------|
 | `AddWalker()`       | 添加行人代理，配置半径/高度/最大速度  |
 | `AddOrUpdateVehicle()` | 更新车辆OBB，标记为动态障碍物        |
 | `UpdateCrowd()`     | 每帧更新代理状态（位置/速度/避障）     |
 | `RemoveAgent()`     | 移除代理并清理映射关系               |
 
 ### 4. 动态避障  
 - **车辆包围盒计算**  
   ```cpp
   // 计算旋转后的OBB角点
   Vector3D box_corner = RotatePointOnOrigin2D(corner, yaw_angle) + vehicle.location;
   ```
 - **避障参数配置**  
   ```cpp
   params.obstacleAvoidanceType = 3; // 避障品质（0-3对应低到高）
   ```

 ---
 
 ## 类与方法详解  
 ### `Navigation` 类  
 #### 关键方法  
 | 方法                      | 参数说明                              | 返回值  |
 |--------------------------|---------------------------------------|---------|
 | `CreateCrowd()`          | 初始化人群管理器，配置过滤器和避障参数 | void    |
 | `GetRandomLocation()`    | 生成可行走区域的随机位置              | bool    |
 | `SetPedestriansCrossFactor()` | 设置行人过马路概率（0.0-1.0）      | void    |
 
 #### 内部状态管理  
 - **映射表**  
   ```cpp
   unordered_map<ActorId, int> _mapped_walkers_id;  // 行人ID到索引
   unordered_map<ActorId, int> _mapped_vehicles_id; // 车辆ID到索引
   ```
 
 ---
 
 ## 坐标系统与数据结构  
 ### 坐标系转换  
 | 系统        | 坐标顺序 | 转换示例                     |
 |-------------|----------|------------------------------|
 | CARLA (UE)  | (X, Y, Z)| `from.x, from.y, from.z`     |
 | Recast      | (X, Z, Y)| `float pos[3] = {x, z, y};`  |
 
 ### 数据结构  
 - **导航网格头**  
   ```cpp
   struct NavMeshSetHeader {
     int magic;             // 魔术字 'MSET'
     int version;           // 版本号
     dtNavMeshParams params;// 网格参数（原点/瓦片尺寸）
   };
   ```
 
 ---
 
 ## 关键流程  

### 1. 路径规划流程

1. **初始化查询环境**

   ```cpp
   // 创建并初始化 NavMeshQuery 实例
   dtNavMeshQuery* query = dtAllocNavMeshQuery();
   query->init(navmesh, MAX_POLYS);
   // 配置过滤器：设置区域成本、禁行区域等
   dtQueryFilter filter;
   filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL);
   filter.setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);
   filter.setAreaCost(AREA_ROAD, AREA_ROAD_COST);
   filter.setAreaCost(AREA_GRASS, AREA_GRASS_COST);
   ```

2. **最近多边形查询**

   ```cpp
   dtPolyRef start_ref, end_ref;
   float start_pos[3] = {from.x, from.y, from.z};
   float end_pos[3]   = {to.x,   to.y,   to.z};

   // 在 navmesh 中找到距离起点和终点最近的多边形
   if (dtStatusFailed(query->findNearestPoly(start_pos, extents, &filter, &start_ref))) {
       // 处理找不到起点多边形的情况
       return false;
   }
   if (dtStatusFailed(query->findNearestPoly(end_pos, extents, &filter, &end_ref))) {
       // 处理找不到终点多边形的情况
       return false;
   }
   ```

   * `extents` 为搜索半径，一般设置为 `{2,2,2}`。

3. **多边形路径搜索**

   ```cpp
   dtPolyRef polys[MAX_POLYS];
   int poly_count = 0;
   if (dtStatusFailed(query->findPath(start_ref, end_ref, start_pos, end_pos, &filter, polys, &poly_count, MAX_POLYS))) {
       // 记录日志：路径搜索失败或超过最大多边形数
   }
   ```

   * 返回的 `polys` 数组包含从起始到目标的多边形序列，长度为 `poly_count`。

4. **路径点优化转换**

   ```cpp
   float straight_path[MAX_POLYS*3];
   unsigned char straight_flags[MAX_POLYS];
   dtPolyRef straight_polys[MAX_POLYS];
   int path_count = 0;

   query->findStraightPath(
       start_pos, end_pos,
       polys, poly_count,
       straight_path, straight_flags, straight_polys,
       &path_count, MAX_POLYS,
       DT_STRAIGHTPATH_ALL_CROSSINGS
   );

   // 将转换后的路径点填充到 vector<Location>
   path.clear();
   for (int i = 0; i < path_count; ++i) {
       Location pt{straight_path[i*3], straight_path[i*3+1], straight_path[i*3+2]};
       path.push_back(pt);
   }
   ```

   * `path_count` 为最终路径点数量。

5. **清理与返回**

   ```cpp
   dtFreeNavMeshQuery(query);
   return !path.empty();
   ```

---

### 2. 动态障碍物处理流程

1. **更新车辆包围盒**

   ```cpp
   void Navigation::AddOrUpdateVehicle(ActorId id, const Location& loc, const Rotator& rot) {
       std::lock_guard<std::mutex> guard(_mutex);
       // 计算旋转后的 OBB 八个角
       std::array<Vector3D, 8> corners = ComputeOBBCorners(loc, rot.yaw, vehicle_extent);
       // 更新 dtTileCache 或 crowdObstacles
       _crowd->removeObstacle(_vehicle_obstacles[id]);
       _vehicle_obstacles[id] = _crowd->addObstacle(corners.data(), 8);
   }
   ```

2. **动态切割（TileCache）**

   ```cpp
   // 若启用 TileCache，则调用以下接口实现局部切割
   dtTileCacheUpdate* tc_update = dtAllocTileCacheUpdate(...);
   tc_update->opcode = OPCODE_CUT_OUT;
   tc_update->poly = new_poly;
   tile_cache->addTileCacheUpdate(tc_update);
   ```

3. **重建切片**

   ```cpp
   dtStatus status = tile_cache->buildTile(x, y);
   if (dtStatusFailed(status)) {
       // 记录错误并重试
   }
   ```
---

### 3. 人群模拟与避障更新流程

1. **批量添加/移除代理**

   ```cpp
   for (auto& walker : new_walkers) {
       _crowd->addAgent(walker.start, params);
   }
   for (auto& id : removed_walkers) {
       _crowd->removeAgent(_mapped_walkers_id[id]);
   }
   ```

2. **每帧避障与位置更新**

   ```cpp
   void Navigation::UpdateCrowd(float delta_time) {
       std::lock_guard<std::mutex> guard(_mutex);
       _crowd->update(delta_time, nullptr);
       // 同步位置到 CARLA 代理
       for (int i = 0; i < _crowd->getAgentCount(); ++i) {
           dtCrowdAgent* ag = _crowd->getAgent(i);
           Location pos{ag->npos[0], ag->npos[1], ag->npos[2]};
           UpdateCARLAAgent(_agent_ids[i], pos, ag->vel);
       }
   }
   ```

3. **分离与预测**

   ```cpp
   // 设置分离权重和预测时间
   params.separationWeight = 0.5f;
   params.predictTime = 1.0f;
   ```
## 高级定制

### 1. 自定义查询过滤器

在默认过滤器基础上，通过设置不同的区域标志和成本，实现对特定地形的偏好或避让：

```cpp
struct CustomFilter : public dtQueryFilter {
    CustomFilter() {
        // 排除水体区域
        setExcludeFlags(SAMPLE_POLYFLAGS_WATER);
        // 道路成本最低，草地成本略高
        setAreaCost(AREA_ROAD, 1.0f);
        setAreaCost(AREA_GRASS, 5.0f);
        // 增加人行道优先级
        setAreaCost(AREA_SIDEWALK, 0.5f);
    }
};

// 使用示例
CustomFilter filter;
query->findPath(startRef, endRef, startPos, endPos, &filter, polys, &polyCount, MAX_POLYS);
```

**说明**：可以继承 `dtQueryFilter` 并在构造函数中修改 flag 和 cost，以便针对不同场景进行优化。

### 2. Off-Mesh Links 处理

支持跳跃、攀爬等场景，通过 Off-Mesh Links 定义特殊连接：

```cpp
// 添加 Off-Mesh Link
dtMeshTile* tile = navmesh->getTile(0);
const int verts[2] = {startVertexId, endVertexId};
const float linkRadius = 0.2f;
const dtPolyRef ref = tile->header->polyCount;
dtOffMeshConnection conn = {};
conn.radius = linkRadius;
conn.poly = ref;
conn.flags = SAMPLE_POLYFLAGS_JUMP;
conn.userId = 1234;
conn.startPosition[0] = sx; conn.startPosition[1] = sy; conn.startPosition[2] = sz;
conn.endPosition[0]   = ex; conn.endPosition[1]   = ey; conn.endPosition[2]   = ez;

// 将连接写入网格
navmesh->addOffMeshConnection(&conn);
```

调用 `findStraightPath` 时，返回的 `straight_flags` 数组中含 `DT_STRAIGHTPATH_OFFMESH_CONNECTION`，可据此在行走逻辑中处理跳跃或爬升。

### 3. 异步加载与多线程优化

当场景较大时，可在后台线程逐瓦片加载/卸载导航数据，同时主线程运行路径查询：

```cpp
// 后台加载任务
std::future<void> loadTask = std::async(std::launch::async, [&](){
    Navigation::LoadTileAsync(x, y);
});

// 在主线程检查完成并合并
if (loadTask.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    std::lock_guard<std::mutex> guard(_mutex);
    navmesh->mergeTile(pendingTile);
}
```

**注意**：所有对 `navmesh` 和 `dtCrowd` 的修改需在同一锁保护下完成，避免并发冲突。
### 4. 日志与可视化支持

内置可将导航网格和路径可视化输出到 CARLA 的调试渲染：

```cpp
// 绘制网格边界
for (int t = 0; t < tileCount; ++t) {
    auto* verts = navmesh->getTile(t)->verts;
    auto* polys = navmesh->getTile(t)->polys;
    for (int i = 0; i < navmesh->getTile(t)->header->polyCount; ++i) {
        RenderDebug::DrawPolygon(verts, polys[i].verts, polys[i].vertCount);
    }
}
// 高亮当前路径
for (size_t i = 1; i < path.size(); ++i) {
    RenderDebug::DrawLine(path[i-1], path[i], Color::Green);
}
```

日志接口集成 `spdlog`：

```cpp
SPDLOG_INFO("Loaded NavMesh tile {} at ({}, {})", tileId, x, y);
SPDLOG_DEBUG("Computed path with {} points", path.size());
```

通过在 CI 中捕获渲染输出和日志，可对路径正确性和性能进行可视化回归测试。
以下是“社交驾驶行为建模”功能的详细介绍及示例伪代码，供文档中直接引用或改写。

---

###5. 社交驾驶行为建模（Social Driving Behavior Modeling）

社交驾驶行为建模旨在让自动驾驶主体（车辆或行人）在多-agent环境中展现更加“自然”“人性化”的行为，例如礼让行人、智能变道、遵守交通礼仪等。核心思路可以基于以下两种方法之一或结合：

1. **社会力模型（Social Force Model）**

   * 将周围每个交通参与者视作“力源”——吸引力（desire force）驱动车辆沿目标前进，斥力（social force）促使车辆与他人保持安全距离。
   * 在每个仿真步，计算合力并更新速度与方向。

2. **深度强化学习（Deep Reinforcement Learning）**

   * 将车辆状态（位置、速度、周边环境）作为观测，动作空间包括加减速、转向和变道；
   * 通过奖励函数（如“安全”“效率”“礼让”）训练政策网络，实现多样化行为。

以上两者可混合使用：先用社会力模型快速逼近安全、礼让行为，再用RL微调细节。

### 算法流程
```cpp
#include <vector>
#include <cmath>

// 简单的二维向量类
struct Vector2 {
    double x;
    double y;

    Vector2(double _x = 0, double _y = 0) : x(_x), y(_y) {}

    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
    Vector2 operator*(double scalar) const {
        return Vector2(x * scalar, y * scalar);
    }
    Vector2& operator+=(const Vector2& other) {
        x += other.x; y += other.y; return *this;
    }

    double length() const {
        return std::sqrt(x*x + y*y);
    }
    Vector2 normalized() const {
        double len = length();
        if (len > 1e-6) return *this * (1.0 / len);
        return Vector2(0, 0);
    }
};

// Agent 类
struct Agent {
    Vector2 position;
    Vector2 velocity;
    Vector2 goal_pos;
    double desired_speed;
    double mass;
    int lane;

    Agent(const Vector2& pos, const Vector2& vel, const Vector2& goal,
          double speed, double m)
        : position(pos), velocity(vel),
          goal_pos(goal), desired_speed(speed), mass(m), lane(0) {}
};

// 计算道路约束力（示例 stub，可根据实际情况补充）
Vector2 computeRoadConstraints(const Agent& agent) {
    // 例如车道边界、交通信号灯等影响，这里先返回零向量
    return Vector2(0, 0);
}

// 是否需要变道（规则或RL策略 stub）
bool needLaneChange(const Agent& agent, const std::vector<Agent>& neighbors) {
    // 简单示例：速度低于某阈值则考虑变道
    return agent.velocity.length() < agent.desired_speed * 0.8;
}

// 选取最佳车道（规则或RL策略 stub）
int selectBestLane(const Agent& agent, const std::vector<Agent>& neighbors) {
    // 简单示例：切换到下一车道
    return agent.lane + 1;
}

// 是否存在潜在冲突（规则或RL策略 stub）
bool potentialConflict(const Agent& agent, const std::vector<Agent>& neighbors) {
    for (const auto& other : neighbors) {
        double dist = (agent.position - other.position).length();
        if (dist < 1.0) {
            return true;
        }
    }
    return false;
}

// 每个仿真步更新函数
void updateAgentBehavior(Agent& agent,
                         const std::vector<Agent>& neighbors,
                         double dt) {
    // 参数
    const double A = 5.0;       // 斥力强度
    const double B = 1.0;       // 斥力衰减系数
    const double k = 2.0;       // 吸引力比例
    const double D_safe = 2.0;  // 舒适距离

    // 1. 计算吸引力 toward goal
    Vector2 dir_to_goal = (agent.goal_pos - agent.position).normalized();
    Vector2 F_att = (dir_to_goal * agent.desired_speed - agent.velocity) * k;

    // 2. 计算斥力 against each neighbor
    Vector2 F_rep_total(0, 0);
    for (const auto& other : neighbors) {
        Vector2 d_vec = agent.position - other.position;
        double dist = std::max(d_vec.length(), 1e-3);
        Vector2 n_ij = d_vec * (1.0 / dist);
        double exp_term = std::exp((D_safe - dist) / B);
        Vector2 F_rep_ij = n_ij * (A * exp_term);
        F_rep_total += F_rep_ij;
    }

    // 3. 考虑道路和交通规则约束
    Vector2 F_road = computeRoadConstraints(agent);

    // 4. 合力与动力学更新
    Vector2 F_total = F_att + F_rep_total + F_road;
    Vector2 acceleration = F_total * (1.0 / agent.mass);
    agent.velocity += acceleration * dt;
    agent.position += agent.velocity * dt;

    // 5. 变道或减速决策（可选 RL 微调）
    if (needLaneChange(agent, neighbors)) {
        agent.lane = selectBestLane(agent, neighbors);
    }
    if (potentialConflict(agent, neighbors)) {
        agent.velocity = agent.velocity * 0.5;  // 紧急减速
    }
}
```

 ---
 
 ## 配置参数  
 ### 静态常量  
 ```cpp
 static const int MAX_AGENTS = 500;     // 最大代理数
 static const float AGENT_RADIUS = 0.3f; // 碰撞半径
 static const float AREA_ROAD_COST = 10.0f; // 道路区域路径成本
 ```
 
 ### 避障参数  
 | 参数                  | 值    | 描述                |
 |-----------------------|-------|---------------------|
 | `adaptiveDivs`        | 7     | 采样细分次数        |
 | `separationWeight`   | 0.5f  | 分离行为权重        |
 
 ---
 
在 CARLA 的导航模块中，使用 `std::mutex` 对关键操作加锁的主要目的是确保多线程环境下的数据一致性和线程安全性。以下是对加锁必要性的详细说明，可补充至“注意事项”部分：

---

## 注意事项

1. **内存管理**

   * 显式调用 `dtFreeNavMesh()` 释放资源，避免内存泄漏。

2. **线程安全**

   * 所有修改 `dtCrowd` 状态的操作需加锁：

     ```cpp
     std::lock_guard<std::mutex> lock(_mutex);
     ```
   * **加锁的必要性**：
     * **避免数据竞争**：在多线程环境中，多个线程可能同时访问和修改共享资源（如导航网格、代理状态等）。如果不加锁，可能导致数据竞争，产生不可预测的行为。
     * **确保操作的原子性**：加锁可以确保对共享资源的操作是原子的，即操作在执行过程中不会被其他线程打断，从而保持数据的一致性。
     * **防止死锁和资源泄漏**：通过合理使用锁机制，可以避免死锁的发生，并确保资源在使用完毕后被正确释放。
     * **提高程序的稳定性和可靠性**：加锁可以防止由于并发访问导致的程序崩溃或异常，提高系统的稳定性。
     * **参考资料**：关于多线程加锁的详细解释，请参阅 [CSDN博客](https://blog.csdn.net/ke1ying/article/details/109531062)。

3. **性能限制**

   * `MAX_POLYS=256` 和 `MAX_AGENTS=500` 影响计算复杂度。

 ---
| 注意事项 | 对应资源类型           | 具体对象/结构                                               |
| ---- | ---------------- | ----------------------------------------------------- |
| 内存管理 | 导航网格与查询对象        | `dtNavMesh*`、`dtNavMeshQuery*`                        |
| 线程安全 | 路径查询与人群管理        | `dtNavMesh`、`dtNavMeshQuery`、`dtCrowd`、`dtCrowdAgent` |
| 性能限制 | 网格查询多边形数、人群代理数限制 | `DT_MAX_POLYS`、`dtCrowd(maxAgents=…)`                 |

---

## 内存管理

Recast/Detour 在 CARLA 中负责导航网格的加载、切割与查询，需手动释放相应资源以防泄漏。

* **导航网格对象 (`dtNavMesh*`)**

  * CARLA 在 `Navigation::Load(...)` 中调用 `dtAllocNavMesh()` 分配网格实例，结束后必须调用 `dtFreeNavMesh(navmesh)` 释放所有瓦片数据（仅释放标记 `DT_TILE_FREE_DATA` 的瓦片）([recastnav.com][1])。
* **查询对象 (`dtNavMeshQuery*`)**

  * 用于执行 `findNearestPoly()`、`findPath()`、`findStraightPath()` 等操作，分配方式同 `dtAllocNavMeshQuery()`，完成后应调用 `dtFreeNavMeshQuery(query)` 以回收节点池和开放列表内存([recastnav.com][1])。
* **节点池与开放列表**


上述资源均在 CARLA 源码的 `LibCarla/Navigation` 模块中通过 Recast/Detour API 管理，必须成对使用 `dtAlloc…` 与 `dtFree…`。

---

## 线程安全

CARLA 支持同时更新数百个行人和车辆，底层依赖 Detour 的并发模型，但自身并未对 Detour 做线程封装，需在应用层自行加锁。

* **Detour 本身非线程安全**

  * Detour 将数据分为“导航数据”与“查询数据”两部分。导航数据（`dtNavMesh`）可多线程只读；查询数据（包含节点池、开放列表）在执行过程中会被修改，存在并发访问冲突([Google 群组][2])。
* **CARLA 的 `dtCrowd` 管理**

  * `dtCrowd` 维护一组 `dtCrowdAgent`，负责避障与本地寻路。多线程更新时，若多个线程同时调用 `update()`、`addAgent()`、`removeAgent()`，会竞态修改内部容器与过滤器参数，导致未定义行为([rwindegger.github.io][3], [GitHub][4])。
* **`std::mutex` 加锁示例**

  ```cpp
  void Navigation::UpdateCrowd(const EpisodeState& state) {
      std::lock_guard<std::mutex> guard(_mutex);
      _crowd->update(dt, agents, agentCount);
  }
  ```

  * 该锁确保在一次 `update()` 调用中，无其他线程能同时修改 `_crowd` 或映射表 `_mapped_walkers_id`/`_mapped_vehicles_id`([Google 群组][2])。
* **加锁**

  1. **避免数据竞争**：并发读写会导致节点池或代理容器处于不一致状态，触发崩溃或路径错误([Google 群组][5])。
  2. **保证原子性**：一系列操作（如先移除旧代理再添加新代理）要么全部完成，要么全部不生效，防止中间状态被其他线程观察到而出错([Google 群组][2])。
  3. **防止资源泄漏**：锁可确保在异常抛出时，析构保护会释放 `_mutex` 并允许后续清理逻辑正确执行。
  4. **提升可靠性**：虽有性能开销，但对关键操作加锁能极大降低生产环境下的随机崩溃风险([Google 群组][2])。

---

## 性能限制

CARLA 在导航模块中对路径搜索与人群模拟规模做了上限，以控制查询与避障计算复杂度。

* **最大多边形数 (`DT_MAX_POLYS`)**

  * Detour 查询时会遍历多边形网格，`DT_MAX_POLYS=256`（默认值）限定 `findPath()` 等函数的多边形序列最大长度，超限将提前退出；过大可支持长路径但压力激增([recastnav.com][1])。
* **最大代理数 (`dtAllocCrowd(maxAgents)`)**

  * 在 `Navigation::CreateCrowd()` 中，`dtAllocCrowd(MAX_AGENTS)` 分配代理数组，CARLA 默认 `MAX_AGENTS=500`，即一次性只能管理 500 个行人/车辆避障；超出后 `addAgent()` 返回失败([GitHub][4])。
* **避障质量 (`obstacleAvoidanceType`)**

  * 可在 0–3 间切换采样分辨率与代价计算复杂度，较高质量下 CPU 采样更密集，影响单帧性能；CARLA 默认 `3`，可针对性能需求下调✱。
* **多线程并行限度**

  * 虽可为每线程创建独立 `dtNavMeshQuery` 实例并发运行查询，但若导航网格需动态更新（`addTile()`/`removeTile()`），更新时仍需全局锁定以防查询中断([Google 群组][5])。

---


 ## 示例代码  

```cpp
// ===== NavigationExample.cpp =====
#include "Navigation.h"        // 包含 Navigation 类声明
#include "geom/Location.h"     // CARLA 几何库
#include "geom/Rotator.h"
#include "Logging.h"           // 假设有统一的日志接口

int main(int argc, char** argv)
{
    // 1. 创建导航实例并加载 NavMesh
    Navigation nav;
    const std::string navmesh_file = "city_navmesh.bin";
    if (!nav.Load(navmesh_file)) {
        logging::error("导航数据加载失败: %s", navmesh_file.c_str());
        return -1;
    }
    logging::info("成功加载导航网格：%s", navmesh_file.c_str());

    // 2. 创建人群管理器，配置避障和路径过滤参数
    nav.CreateCrowd();
    // 设置行人越过道路的概率
    nav.SetPedestriansCrossFactor(0.2f);
    logging::info("人群管理器初始化完成");

    // 3. 添加行人代理
    const uint64_t walker_id = 1001;
    geom::Location walker_spawn{10.0f, 20.0f, 0.5f}; 
    if (!nav.AddWalker(walker_id, walker_spawn)) {
        logging::warn("添加行人 %llu 失败", walker_id);
    } else {
        logging::info("行人 %llu 已添加，初始位置 (%.2f, %.2f, %.2f)",
                      walker_id,
                      walker_spawn.x, walker_spawn.y, walker_spawn.z);
    }

    // 4. 设置行人目标点
    geom::Location walker_target{50.0f, 30.0f, 0.5f};
    if (!nav.SetWalkerTarget(walker_id, walker_target)) {
        logging::warn("设置行人 %llu 目标失败", walker_id);
    } else {
        logging::info("行人 %llu 目标设置为 (%.2f, %.2f, %.2f)",
                      walker_id,
                      walker_target.x, walker_target.y, walker_target.z);
    }

    // 5. 添加一辆动态障碍物（车辆）
    const uint64_t vehicle_id = 2001;
    geom::Location vehicle_loc{15.0f, 25.0f, 0.0f};
    geom::Rotator vehicle_rot{0.0f, 90.0f, 0.0f};  // 车辆朝向 90°
    if (!nav.AddOrUpdateVehicle(vehicle_id, vehicle_loc, vehicle_rot)) {
        logging::warn("添加/更新车辆 %llu 失败", vehicle_id);
    } else {
        logging::info("车辆 %llu 已注册为动态障碍物，位置 (%.2f, %.2f, %.2f), 偏航角 %.2f°",
                      vehicle_id,
                      vehicle_loc.x, vehicle_loc.y, vehicle_loc.z,
                      vehicle_rot.yaw);
    }

    // 6. 主循环：在每个模拟帧调用更新函数
    const float fixedDeltaTime = 1.0f / 30.0f;  // 30 FPS
    for (int frame = 0; frame < 1000; ++frame) {
        // 假设获取当前车辆列表和行人列表的逻辑已在外部实现
        std::vector<VehicleState> vehicle_list = GetCurrentVehicleStates();
        EpisodeState sim_state = GetCurrentEpisodeState();

        // 更新人群（行人）避障和路径跟踪
        nav.UpdateCrowd(fixedDeltaTime);

        // 更新所有车辆动态障碍物（位置与旋转）
        nav.UpdateVehicles(vehicle_list);

        // 可以在此处插入渲染或其他逻辑
    }

    // 7. 退出前清理资源（析构时自动释放 NavMesh、Crowd 等）
    logging::info("导航模拟结束，开始清理资源");
    return 0;
}
```

**要点说明：**

1. **错误检查与日志**
   每一步操作后都进行了返回值判断，并通过 `logging` 模块输出信息或警告，便于调试与排查问题。
2. **配置细节**

   * 调用 `SetPedestriansCrossFactor()` 演示了如何动态调整行人过马路的概率。
   * `fixedDeltaTime` 固定为 30 帧每秒，仅供示例，可根据实际仿真引擎时间步长调整。
3. **车辆动态障碍**

   * 使用 `AddOrUpdateVehicle()` 注册车辆包围盒，示例中展示了位置和朝向的设置。
   * 在主循环中，每帧都调用 `UpdateVehicles()` 同步车辆状态。
4. **主循环结构**

   * 假设外部提供 `GetCurrentVehicleStates()` 与 `GetCurrentEpisodeState()`，实际可根据用户项目自行实现。
   * 将行人更新和车辆更新放在同一循环中，保证多代理场景下一致性。


 ---
 
 ## 附录  
 ### 依赖项  
 - **Recast/Detour**：导航网格与路径规划  
 - **CARLA 几何库**：`geom::Location`, `geom::Math`  
 
 ### 调试支持  
 - **断言检查**：  
   ```cpp
   DEBUG_ASSERT(_nav_query != nullptr); // 确保查询对象已初始化
   ```  
 - **日志输出**：  
   ```cpp
   logging::log("Nav: failed to create crowd");
   ```
## 附录

### 依赖项

* **Recast/Detour**：导航网格与路径规划
* **CARLA 几何库**：`geom::Location`, `geom::Math`

### 调试支持

* **断言检查**：

  ```cpp
  DEBUG_ASSERT(_nav_query != nullptr); // 确保查询对象已初始化
  ```
* **日志输出**：

  ```cpp
  logging::log("Nav: failed to create crowd");
  ```

**补充说明**：增加调试支持说明，帮助定位运行时错误和状态异常。

---

## 扩展功能

### 1. 运行时导航网格切割与TileCache

* **动态障碍物切割**：基于 `dtTileCache` 实现对动态物体的局部切割，无需重建整个网格，可显著提升性能。
* **离线编辑支持**：使用 CARLA UE4 导出器生成 `.OBJ` 并调用 RecastBuilder 生成 `.BIN`，支持自定义参数（如瓦片大小、层高度）([carla.readthedocs.io](https://carla.readthedocs.io/en/0.9.7/how_to_generate_pedestrians_navigation/?utm_source=chatgpt.com))。

### 2. 离散导航链接（Off-Mesh Links）

* **跳跃与爬升链接**：允许通过手动或算法方式在 NavMesh 上添加“跳跃”、“梯子”等自定义连接，增强导航灵活性([digestingduck.blogspot.com](https://digestingduck.blogspot.com/2009/07/recast-and-detour-roadmap.html?utm_source=chatgpt.com))。
* **运行时增删链接**：可通过 `dtMeshTile` 动态管理 Off-Mesh 连接，无需重建瓦片；推荐使用哈希表存储链接索引以加速查询。

### 3. 可视化与调试

* **NavMesh调试渲染**：CARLA 提供的 No-Rendering 工具可在 2D 地图上可视化导航网格、交通信号和行人路径([carla.org](https://carla.org/2019/01/31/release-0.9.3/?utm_source=chatgpt.com))。
* **调试UI插件**：在 UE4 编辑器中使用 Carla Exporter 导出 OBJ，并在场景中加载调试材质查看不同区域成本；可通过过滤器调节 `AREA_ROAD_COST` 实时预览路径变化。

### 4. 性能剖析与优化

* **Query缓存**：对频繁调用的 `dtNavMeshQuery` 对象进行复用，避免重复分配，减少 GC 开销。
* **多线程分桶调度**：将 `UpdateCrowd` 分成多个分区并行执行，每个线程处理一组代理，合并结果时加锁保障一致性。
* **代理批量更新**：对相邻或相似路径的代理进行合并查询，利用局部空间相似性降低调用次数。

---

## 典型应用案例

### 异步获取路径

```cpp
std::future<PathResult> async_get_path = std::async(std::launch::async, [&nav,from,to,filter](){
    PathResult result;
    nav.GetPath(from,to,filter,result.path,result.area);
    return result;
});
```

* **补充说明**：使用异步接口可避免主线程阻塞，适用于高并发路径请求场景。

### 自定义区域成本示例

```cpp
filter->setAreaCost(AREA_ROAD, 5.0f);
filter->setAreaCost(AREA_GRASS, 20.0f);
```

* **补充说明**：通过动态调节 `dtQueryFilter` 中的区域成本，可实现行人或车辆对不同地形偏好的模拟。


 