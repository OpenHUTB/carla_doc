# [关卡设计蓝图](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/)

* 建筑蓝图 [BP_Building](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Building.uasset)
* 将建筑转化为代码 [BP_BuildingConverterToCode](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_BuildingConverterToCode.uasset)
* 护栏 [BP_GuardRail](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_GuardRail.uasset)
* 程序化生成建筑 [BP_Procedural_Bulding](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Procedural_Bulding.uasset)
* 程序化窗口的零件 [BP_Procedural_WindowPieces](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Procedural_WindowPieces.uasset)
* 道具 [BP_Prop](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Prop.uasset)
* [Rep样条](../tuto_M_custom_weather_landscape.md#bp_repspline) [BP_RepSpline](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_RepSpline.uasset) 
* [样条](../tuto_M_custom_weather_landscape.md#bp_spline) [BP_Spline](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Spline.uasset) 
* 河流 [BP_River](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_River.uasset)
* 地形 [BP_Terrain](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Terrain.uasset)
* 植被 [BP_Vegetation](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Vegetation.uasset)
* [墙](../tuto_M_custom_weather_landscape.md#bp_wall) [BP_Wall](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BP_Wall.uasset)
* 通用建筑类 [BuildingMaster](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/BuildingMaster.uasset) ：在建筑网格上分配和/或随机分配材料、堆叠地板、将道具放置在随机位置
* 贴花图集生成器 [DecalAtlasGenerator](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/DecalAtlasGenerator.uasset)
* 道路画家 [RoadPainter](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/RoadPainter.uasset)
* 道路画家预设 [RoadPainterPreset](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/LevelDesign/RoadPainterPreset.uasset)
* # Carla虚幻场景中关卡设计（LevelDesign）技术说明

## 1. 概述
在Carla虚幻引擎模拟器环境中，**关卡设计（LevelDesign）**是创建和配置各类模拟场景的基础。它涉及场景的布局、元素布局、环境配置，以及各种静态和动态资产的搭配。良好的关卡设计可以提高模拟的真实感和多样性，为自动驾驶的训练、测试提供丰富的环境条件。

## 2. 关卡设计的核心要素

### 2.1 场景布局
- **地形地貌**：包括道路、城市、郊区、乡村等不同类型的环境。设计依据通常采用OpenDRIVE标准定义道路网络。
- **道路网络**：由OpenDRIVE文件定义，标明车道、路口、交汇点、停车场等元素。
- **建筑及静态资产**：如住宅、商业建筑、桥梁、隧道、交通标志、信号灯等，作为场景的静态元素。
- **静态资产（Static）**：固定在场景中的元素，比如路灯、树木、交通标志、路面涂装等。

### 2.2 气候与环境条件
- **天气参数**：太阳位置、云量、风速、能见度、雾、雨雪等，影响场景的视觉效果和驾驶表现。
- **时间设置**：白天、夜晚或黄昏等时间段，可以调节光照和环境状态。

### 2.3 地标和关键点
- **交通标志和信号灯**：定义道路的交通规则和信号变化。
- **关卡目标位置**：设定起点、终点、检测点、休息区等关键位置。

### 2.4 其他环境元素
- **植被**：树木、灌木、草地等自然元素。
- **道具和景观**：休息区、建筑物、广告牌、障碍物等。

---






