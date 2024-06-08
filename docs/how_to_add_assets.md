# 如何添加资产
*本文档尚在编写中，可能不完整。*

## 添加车辆
按照 [艺术指南](https://docs.unrealengine.com/latest/INT/Engine/Physics/Vehicles/VehcileContentCreation/index.html) 创建骨架网格体和物理资产。 其余部分请参阅 [车辆用户指南](https://docs.unrealengine.com/latest/INT/Engine/Physics/Vehicles/VehicleUserGuide/) 。

- 将 fbx 作为 Skelletal Mesh 导入到其自己的文件夹中 `Content/Static/Vehicles`。物理资产和骨架应自动创建，并将三者链接在一起。
- 调整物理资产。删除自动创建的资产，并为与`Vehicle_Base`形状匹配的骨骼添加框，确保启用了生成撞击事件。为每个轮子添加一个球体，并将其“物理类型”设置为“运动学”。
- 在该文件夹中创建一个“动画蓝图”，创建时选择“VehicleAnimInstance”作为父类，并以此汽车模型的骨架作为目标骨架。添加动画图，如上面给出的链接所示（或在其他汽车的动画中查找，如 Mustang）。
- 创建文件夹`Content/Blueprints/Vehicles/<vehicle-model>`
- 在该文件夹中，创建两个从“VehicleWheel”类派生的蓝图类。将它们命名为`<vehicle-model>_FrontWheel`和`<vehicle-model>_RearWheel`。将它们的“形状半径”设置为与网格轮半径完全匹配（注意，是半径而不是直径）。将它们的“轮胎配置”设置为“CommonTireConfig”。在前轮上取消选中“受手刹影响”，在后轮上将“转向角”设置为零。
- 在同一个文件夹中创建一个蓝图类，从中派生出并`BaseVehiclePawn`调用它`<vehicle-model>`。打开它进行编辑并选择组件“Mesh”，将“Skeletal Mesh”和“Anim Class”设置为相应的。然后选择组件“VehicleMovement”，在“Vehicle Setup”下展开“Wheel Setups”，设置每个车轮

* 0：轮子类别= `<vehicle-model>_FrontWheel`，骨骼名称=`Wheel_Front_Left`
* 1：轮子类别= `<vehicle-model>_FrontWheel`，骨骼名称=`Wheel_Front_Right`
* 2：轮子类别= `<vehicle-model>_RearWheel`，骨骼名称=`Wheel_Rear_Left`
* 3：轮子类别= `<vehicle-model>_RearWheel`，骨骼名称=`Wheel_Rear_Right`

- 向其中添加一个名为“Box”的盒子组件（参见 Mustang 蓝图），并将大小设置为从上方覆盖车辆区域。确保设置了以下内容
* 隐藏在游戏中
* 模拟物理已禁用
* 禁用生成重叠事件
* 碰撞预设设置为“NoCollision”

测试一下，进入 CarlaGameMode 蓝图并将“Default Pawn Class”更改为新创建的汽车蓝图。

## 地图生成
对于道路生成，预计会找到以下网格
```
# Enum                            Filepath
RoadTwoLanes_LaneLeft          at "Content/Static/Road/St_Road_TileRoad_RoadL.uasset"
RoadTwoLanes_LaneRight         at "Content/Static/Road/St_Road_TileRoad_RoadR.uasset"
RoadTwoLanes_SidewalkLeft      at "Content/Static/SideWalk/St_Road_TileRoad_SidewalkL.uasset"
RoadTwoLanes_SidewalkRight     at "Content/Static/SideWalk/St_Road_TileRoad_SidewalkR.uasset"
RoadTwoLanes_LaneMarkingSolid  at "Content/Static/RoadLines/St_Road_TileRoad_LaneMarkingSolid.uasset"
RoadTwoLanes_LaneMarkingBroken at "Content/Static/RoadLines/St_Road_TileRoad_LaneMarkingBroken.uasset"

Road90DegTurn_Lane0            at "Content/Static/Road/St_Road_Curve_Road1.uasset"
Road90DegTurn_Lane1            at "Content/Static/Road/St_Road_Curve_Road2.uasset"
Road90DegTurn_Lane2            at "Content/Static/Road/St_Road_Curve_Road3.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road4.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road5.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road6.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road7.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road8.uasset"
Road90DegTurn_Lane3            at "Content/Static/Road/St_Road_Curve_Road9.uasset"
Road90DegTurn_Sidewalk0        at "Content/Static/SideWalk/St_Road_Curve_Sidewalk1.uasset"
Road90DegTurn_Sidewalk1        at "Content/Static/SideWalk/St_Road_Curve_Sidewalk2.uasset"
Road90DegTurn_Sidewalk2        at "Content/Static/SideWalk/St_Road_Curve_Sidewalk3.uasset"
Road90DegTurn_Sidewalk3        at "Content/Static/SideWalk/St_Road_Curve_Sidewalk4.uasset"
Road90DegTurn_LaneMarking      at "Content/Static/Road/St_Road_Curve_LaneMarking.uasset"

RoadTIntersection_Lane0        at "Content/Static/Road/St_Road_TCross_Road1.uasset"
RoadTIntersection_Lane1        at "Content/Static/Road/St_Road_TCross_Road2.uasset"
RoadTIntersection_Lane2        at "Content/Static/Road/St_Road_TCross_Road3.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road4.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road5.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road6.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road7.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road8.uasset"
RoadTIntersection_Lane3        at "Content/Static/Road/St_Road_TCross_Road9.uasset"
RoadTIntersection_Sidewalk0    at "Content/Static/SideWalk/St_Road_TCross_Sidewalk1.uasset"
RoadTIntersection_Sidewalk1    at "Content/Static/SideWalk/St_Road_TCross_Sidewalk2.uasset"
RoadTIntersection_Sidewalk2    at "Content/Static/SideWalk/St_Road_TCross_Sidewalk3.uasset"
RoadTIntersection_Sidewalk3    at "Content/Static/SideWalk/St_Road_TCross_Sidewalk4.uasset"
RoadTIntersection_LaneMarking  at "Content/Static/RoadLines/St_Road_TCross_LaneMarking.uasset"

RoadXIntersection_Lane0        at "Content/Static/Road/St_Road_XCross_Road1.uasset"
RoadXIntersection_Lane1        at "Content/Static/Road/St_Road_XCross_Road2.uasset"
RoadXIntersection_Lane2        at "Content/Static/Road/St_Road_XCross_Road3.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road4.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road5.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road6.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road7.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road8.uasset"
RoadXIntersection_Lane3        at "Content/Static/Road/St_Road_XCross_Road9.uasset"
RoadXIntersection_Sidewalk0    at "Content/Static/SideWalk/St_Road_XCross_Sidewalk1.uasset"
RoadXIntersection_Sidewalk1    at "Content/Static/SideWalk/St_Road_XCross_Sidewalk2.uasset"
RoadXIntersection_Sidewalk2    at "Content/Static/SideWalk/St_Road_XCross_Sidewalk3.uasset"
RoadXIntersection_Sidewalk3    at "Content/Static/SideWalk/St_Road_XCross_Sidewalk4.uasset"
RoadXIntersection_LaneMarking  at "Content/Static/RoadLines/St_Road_XCross_LaneMarking.uasset"
```
