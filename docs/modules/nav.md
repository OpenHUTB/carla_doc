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
 | `RemoveAgent()`     | 移除代理并清理映射关系                |
 
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
 ### 路径规划流程  
 1. **输入**：起点 `from` 和终点 `to`  
 2. **查询最近多边形**：  
    ```cpp
    findNearestPoly(start_pos, filter, &start_ref);
    ```  
 3. **多边形路径搜索**：  
    ```cpp
    findPath(start_ref, end_ref, polys, &num_polys);
    ```  
 4. **直线路径优化**：  
    ```cpp
    findStraightPath(polys, num_polys, straight_path);
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
 
 ## 注意事项  
 1. **内存管理**  
    - 显式调用 `dtFreeNavMesh()` 释放资源，避免内存泄漏  
 2. **线程安全**  
    - 所有修改 `dtCrowd` 状态的操作需加锁：  
      ```cpp
      std::lock_guard<std::mutex> lock(_mutex);
      ```  
 3. **性能限制**  
    - `MAX_POLYS=256` 和 `MAX_AGENTS=500` 影响计算复杂度  
 
 ---
 
 ## 示例代码  
 ### 初始化导航系统  
 ```cpp
 Navigation nav;
 nav.Load("city_navmesh.bin");
 nav.CreateCrowd();
 
 // 添加行人
 geom::Location spawn_point(10.0f, 20.0f, 0.5f);
 nav.AddWalker(1001, spawn_point);
 
 // 设置目标点
 geom::Location target(50.0f, 30.0f, 0.5f);
 nav.SetWalkerTarget(1001, target);
 ```
 
 ### 每帧更新  
 ```cpp
 void OnSimulationTick(EpisodeState state) {
   nav.UpdateCrowd(state);  // 更新代理状态
   nav.UpdateVehicles(vehicle_list); // 同步车辆位置
 }
 ```
 
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


 