# 行人蓝图

* 行人工厂 [WalkerFactory](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerFactory.uasset)
* 行人功能 [WalkerFunctions](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerFunctions.uasset)
* 行人选择器 [WalkerSelector](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Walkers/WalkerSelector.uasset)

会更好

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

## 4. Walker属性配置
- **属性访问**：
```python
speed_attr = walker_bp.get_attribute('speed')
walker_bp.set_attribute('speed', '1.5')  # 设置速度
```
- **常用属性**：
  - **speed**：行走速度（m/s）
  - **gender**：性别（一般非必须）
  - **pose**：姿态信息
  - **behavior**：行为类型（静止、随机走动、特定路线）
  
- **自定义行为**：
  - 可以结合路径点、行为脚本控制Walker模拟更真实的动作。

## 5. 复杂行为与路径规划
- 支持通过路径点或导航网格引导Walker
- 可结合交通信号、障碍物等动态环境调整Walker路径
- 使用OpenDRIVE地图数据进行更复杂的导航定义

## 6. 与传感器和环境的交互
- **传感器检测**：如相机、激光雷达检测Walker
- **行人行为响应**：可编写逻辑（如避让、等待）应对算法训练
- **行为模拟**：配合交通管理器调节Walker在不同交通场景中的表现

## 7. 资源管理
- **蓝图库**：通过`world.get_blueprint_library()`获取
- **多Walker场景优化**：批量生成与销毁以节省资源
- **自定义资产添加**：贡献者可制定自定义Walker资产，加入蓝图库

## 8. 发展与定制
- 支持自定义行为脚本编写（Python、蓝图）
- 可集成复杂的AI行为模型
- 支持与交通信号等场景元素的协调动作

## 9. 总结
Carla中的Walker为模拟环境提供了高度灵活与可定制的人类行为模型。通过合理配置蓝图、路径和行为参数，开发者可以构建多样化且逼真的人流场景，强化自动驾驶系统的训练和验证效果。

---

**注意事项**：
- 在生成Walker时，注意位置避免冲突。
- 结合交通规则和场景需求调整Walker行为，使模拟更贴近现实。
- 详细参数和属性请参考官方文档及Python API帮助。

---

