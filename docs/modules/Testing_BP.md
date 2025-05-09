## 测试蓝图

* 点光源旋转 [PointLightRotator](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Testing/PointLightRotator.uasset) ：点光源是从单一位置向各个方向发射光线的光源

# CARLA 测试说明文档

## 一、项目概述
**CARLA 测试** 基于开源自动驾驶仿真平台，旨在通过可重复的仿真环境验证自动驾驶算法、传感器性能及交通场景逻辑。

- **核心目标** ：
  - 验证车辆在复杂场景（如避障、路口交互、恶劣天气）中的决策能力。
  - 评估传感器（LiDAR、摄像头、雷达）数据质量与实时性。
  - 测试交通管理器（Traffic Manager）的车辆行为控制效果。

- **支持测试类型** ：功能测试、性能测试、回归测试、压力测试。

## 二、模块依赖与集成

Testing 模块与 CARLA 的众多模块紧密相关：

### （一）依赖模块

  * **核心模拟引擎** ：提供物理引擎、场景渲染等基本功能，为测试场景的创建和运行提供了基础。
  * **传感器模块（sensor）** ：获取车辆、行人等物体的各种传感器数据，如摄像头图像、激光雷达点云等，用于测试传感器的准确性和性能。
  * **交通管理器（trafficmanager）** ：控制交通流量和交通规则的执行，模拟不同的交通场景。

### （二）集成方式

通过调用和组合这些依赖模块的功能，构建出复杂的测试流程和场景，实现对 CARLA 各个方面的全面测试。

## 三、核心组件

| 组件                | 功能描述                                                                 |
|---------------------|--------------------------------------------------------------------------|
| **场景管理器**      | 加载 / 卸载场景，控制天气、光照、NPC 行为等参数。                        |
| **传感器系统**      | 管理摄像头、LiDAR、IMU 等传感器数据采集与同步。                          |
| **交通管理器**      | 控制 NPC 车辆的路径规划、速度调整、避障逻辑。                            |
| **数据记录器**      | 将仿真数据（传感器输出、车辆状态）保存为 `.log` 或 `.csv` 格式。          |
| **测试评估模块**    | 根据预设规则（如碰撞次数、路径偏差）自动判定测试结果。                    |


## 四、关键功能实现

### （一）场景测试（避障验证）

```python
# 生成障碍物与测试车辆
obstacle = world.spawn_actor(barrier_bp, carla.Transform(location))
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

# 检测碰撞事件
def on_collision(event):
    if event.actor == vehicle:
        print("避障失败！碰撞发生。")
world.on_tick(on_collision)
```

### （二）传感器数据验证（LiDAR 点云）

```python
# 绑定 LiDAR 并检查点云密度
lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('points_per_second', '100000')
lidar.listen(lambda data: np.save("pointcloud.npy", np.frombuffer(data.raw_data, dtype=np.float32))

# 分析点云
points = np.load("pointcloud.npy").reshape(-1, 4)
print(f"有效点数：{len(points)}，范围：{np.max(points[:, :3])} 米")
```

### （三）交通流压力测试

```python
tm = client.get_trafficmanager()
tm.set_global_speed_limit(30)  # 全局限速 30km/h
tm.set_hybrid_physics_mode(True)  # 混合物理模式（远距离车辆低精度模拟）

# 生成 100 辆 NPC 车辆
for i in range(100):
    vehicle = world.spawn_actor(bp, spawn_points[i])
    tm.ignore_lights_percentage(vehicle, 100)  # 所有车辆忽略红灯
```

```