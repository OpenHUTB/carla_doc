# 载具蓝图

## 新两轮基座 [BP_Base2wheeledNew](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/2Wheeled/BP_Base2wheeledNew.uasset)

## 车辆棋子基座 [BaseVehiclePawn](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/BaseVehiclePawn.uasset)
其中`事件图表`中的`中断VehicleControl`模块接受`Vehicle/VehicleControl.h`中所定义的车辆控制变量：
```cpp
UPROPERTY(Category = "Vehicle Control", EditAnywhere, BlueprintReadWrite)
float Brake = 0.0f;
```

## [N轮车辆棋子基座](../tuto_A_add_vehicle.md#add-a-n-wheeled-vehicle) [BaseVehiclePawnNW](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/BaseVehiclePawnNW.uasset)
## 车辆工厂 [VehicleFactory](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/VehicleFactory.uasset)


## 两轮载具 [2Wheeled](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/2Wheeled/)
# Carla虚幻场景中的载具蓝图（Vehicle Blueprints）技术说明

## 1. 概述
在Carla的虚幻引擎场景中，车辆蓝图（Vehicle Blueprints）是定义和管理各种汽车模型及其属性的核心元素。它们不仅包含车辆的几何模型和动画信息，还包括车辆的物理特性、行为控制参数以及附属传感器等，是实现主动交互和自动驾驶仿真的基础。

## 2. 车辆蓝图的结构与组成

### 2.1 蓝图库（Blueprint Library）
- **定义：** 车辆蓝图存放在蓝图库中，通过`carla.BlueprintLibrary`类提供访问。
- **内容：** 包含一系列可用的车辆类型ID（如`vehicle.tesla.model3`、`vehicle.audi.tt`等），同时支持过滤和自定义属性设定。
- **操作：** 可以随机抽取、精准查找，也支持根据特定属性筛选。

### 2.2 车辆蓝图（ActorBlueprint）
- **定义：** 代表具体车辆模型的蓝图对象，类似于虚幻引擎的Actor蓝图。
- **属性：**
  - **ID:** 标识模型类型，关联至具体的3D模型和动画资源。
  - **可修改属性：** 包括颜色、轮子类型、动力参数、灯光状态等。
  - **自定义属性：** 允许用户根据实验需求调节车辆性能和外观。

### 2.3 车辆属性（Attributes）
- **可修改属性：** 如`color`、`engine_power`、`vehicle_physics`等。
- **限制：** 一些属性可能由模型预设、不允许修改。
- **推荐值：** 属性通常配有建议范围或默认值，便于快速配置。

## 3. 车辆蓝图的操作流程

### 3.1 获取蓝图库
```python
blueprint_library = world.get_blueprint_library()
```

### 3.2 查询和筛选
```python
# 查找特定车辆模型
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

# 选择所有车辆蓝图并随机选择
vehicle_blueprints = blueprint_library.filter('vehicle.*')
chosen_bp = random.choice(vehicle_blueprints)
```

### 3.3 修改属性
```python
# 设置车辆颜色为红色
vehicle_bp.set_attribute('color', '255,0,0')

# 调整发动机功率
vehicle_bp.set_attribute('engine_power', '300')
```

### 3.4 生成车辆实例
```python
transform = carla.Transform(carla.Location(x=230, y=195, z=40), carla.Rotation(yaw=180))
vehicle_actor = world.spawn_actor(chosen_bp, transform)
```

### 3.5 自定义功能
- 用户可以扩展车辆蓝图（如添加特殊传感器、动画或自定义参数）。
- 支持导入用户自制的车辆资产。


