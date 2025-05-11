# 行人蓝图

* 行人工厂 [WalkerFactory](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerFactory.uasset)
* 行人功能 [WalkerFunctions](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerFunctions.uasset)
* 行人选择器 [WalkerSelector](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerSelector.uasset)

# Carla虚幻场景中行人（Walkers）技术说明文档

## 1. 概述
在Carla虚幻场景中，**行人（Walkers）**是场景的关键参与者之一，用于模拟城市环境中的人类行为。行人不仅丰富了模拟场景的多样性，还对自动驾驶系统的感知、决策与规划提出更高要求。本技术说明文档旨在介绍Walker的创建、管理、属性配置以及与环境的交互，为开发者提供详细的操作指南。

## 2. 行人（Walkers）简介
- **定义**：在Carla中，Walker代表模拟环境中的人类实体，具有自主运动能力，可设定行为路径、速度和状态。
- **用途**：
  - 模拟城市中复杂的交通场景。
  - 训练感知算法识别和理解行人行为。
  - 测试自动驾驶车辆在不同人流状态下的表现。
- **特点**：
  - 可以动态生成、控制与销毁。
  - 支持各种行为模式（随机、预定义路径等）。
  - 可与传感器、交通规则等模块互动。

## 3. Walker的基本操作流程
### 3.1 获取蓝图库
- **作用**：提供所有可用的Walker资产（蓝图）集合。
- **操作**：
```python
blueprint_library = world.get_blueprint_library()
walkers_blueprints = blueprint_library.filter('walker.*')
```

### 3.2 生成Walker
- **步骤一**：选择合适的Walker蓝图
```python
walker_bp = random.choice(walkers_blueprints)
```
- **步骤二**：定义起始位置和朝向
```python
spawn_point = world.get_random_location_from_navigation()
transform = carla.Transform(spawn_point, carla.Rotation(yaw=random.uniform(0, 360)))
```
- **步骤三**：调用`spawn_actor()`生成Walker实例
```python
walker_actor = world.spawn_actor(walker_bp, transform)
```
- **异常处理**：使用`try_spawn_actor()`避免异常
```python
walker_actor = world.try_spawn_actor(walker_bp, transform)
```

### 3.3 控制Walker行为
- 可以通过设置Walker的行为参数（如行走速度、路径等）实现复杂行为。
- **示例：设定步行速度**
```python
walker_actor.set_attribute('speed', '1.0')  # 速度单位为m/s
```
- **控制行走路径（路径点）**：
  - 使用导航点（waypoints）或预定义路径控制Walker的运动。

### 3.4 监听与数据交互
- **监听Walker状态**：
  ```python
  def walker_callback(data):
      # 处理Walker状态信息
      pass
  
  walker_sensor.listen(walker_callback)
  ```
- **行为模拟**：根据监听的数据调整Walker状态或触发场景事件。

### 3.5 销毁Walker
- **调用销毁方法**：
```python
walker_actor.destroy()
```
- **批量销毁示例**：
```python
for actor in walkers_list:
    actor.destroy()
```

