## OpenSCENARIO 支持

scenario_runner 提供对 [OpenSCENARIO](http://www.openscenario.org/) 标准的支持。当前的实现涵盖了对机动动作、条件、故事和故事板的初步支持。如果您想使用场景的评估标准来评估通过/失败结果，可以将这些标准实现为 StopTriggers（见下文）。然而，并非这些元素的所有功能均可用。如果有疑问，请参阅 srunner/tools/openscenario_parser.py 中的模块文档。

[此处](https://github.com/carla-simulator/scenario_runner/blob/master/srunner/examples/FollowLeadingVehicle.xosc) 提供了基于 OpenSCENARIO 的受支持场景的示例。

此外，建议查看 [此处](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html) 和 [此处](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html) and [here](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword) 提供的官方文档。

### 将 OpenSCENARIO 0.9.x 迁移到 1.0
将旧的 OpenSCENARIO 示例转换为官方标准 1.0 的最简单方法是使用 _xsltproc_ 和位于 openscenario 文件夹中的迁移方案。例子：

```bash
xsltproc -o newScenario.xosc migration0_9_1to1_0.xslt oldScenario.xosc
```


### 支持程度
下面列出了 OpenSCENARIO 属性及其当前支持状态。

#### 常规的 OpenSCENARIO 设置

这涵盖了 OpenSCENARIO Storyboard 外部定义的所有部分

| 属性                          | 支持                            | 笔记                                                                                                                        |
| ---------------------------------------- | ---------------------------------------- |---------------------------------------------------------------------------------------------------------------------------|
| `FileHeader`                       | &#9989;                              | 在描述的开头使用“CARLA:”即可使用 CARLA 坐标系。                                                                                           |
| `CatalogLocations`<br>`ControllerCatalog`                                      | &#9989;                              | 虽然支持目录，但参考/使用可能不起作用。                                                                                                      |
| `CatalogLocations`<br>`EnvironmentCatalog`                                     | &#9989;                              |                                                                                                                           |
| `CatalogLocations`<br>`ManeuverCatalog`                                        | &#9989;                              |                                                                                                                           |
| `CatalogLocations`<br>`MiscObjectCatalog`                                      | &#9989;                              |                                                                                                                           |
| `CatalogLocations`<br>`PedestrianCatalog`                                      | &#9989;                              |                                                                                                                           |
| `CatalogLocations`<br>`RouteCatalog`                                           | &#9989;                              | 虽然支持目录，但参考/使用可能不起作用。                                                                                                      |
| `CatalogLocations`<br>`TrajectoryCatalog`                                      | &#9989;                              | 虽然支持目录，但参考/使用可能不起作用。                                                                                                      |
| `CatalogLocations`<br>`VehicleCatalog`                                         | &#9989;                              |                                                                                                                           |
| `ParameterDeclarations`            | &#9989;                              |                                                                                                                           |
| `RoadNetwork`<br>`LogicFile`                                                   | &#9989;                              | 可以直接使用 Carla 级别（例如LogicFile=Town01）。还可以提供任何 OpenDRIVE 路径。                                                                 |
| `RoadNetwork`<br>`SceneGraphFile`                                              | &#10060;                               | 所提供的信息未被使用。                                                                                                               |
| `RoadNetwork`<br>`TafficSignals`                                               | &#10060;                               | 所提供的信息未被使用。                                                                                                               |
| `Entities`<br>`EntitySelection`                                                | &#10060;                               | 所提供的信息未被使用。                                                                                                               |
| `Entities` `ScenarioObject`<br>`CatalogReference`                               | &#9989;                              | 所提供的信息未被使用。                                                                                                               |
| `Entities` `ScenarioObject`<br>`MiscObject`                                     | &#9989;                              | 该名称应与 CARLA 车辆型号匹配，否则将使用基于vehicleCategory 的默认车辆。 BoundingBox 条目将被忽略。                                                      |
| `Entities` `ScenarioObject`<br>`ObjectController`                               | &#10060;                               | 所提供的信息未被使用。                                                                                                               |
| `Entities` `ScenarioObject`<br>`Pedestrian`                                     | &#9989;                              | 该名称应与 CARLA 车辆型号匹配，否则将使用基于vehicleCategory 的默认车辆。 BoundingBox 条目将被忽略。                                                      |
| `Entities` `ScenarioObject`<br>`Vehicle`                                        | &#9989;                              | 该名称应与 CARLA 车辆型号匹配，否则将使用基于vehicleCategory 的默认车辆。颜色可以通过属性设置（'属性名称=“颜色”值=“0,0,255”'）。 Axles、Performance、BoundingBox 条目将被忽略。 |

<br>


#### OpenSCENARIO 故事板

##### OpenSCENARIO 操作

OpenSCENARIO 操作可用于两个不同的目的。首先，动作可用于定义某些事物（例如交通参与者）的初始行为。因此，Actions 可以在 OpenSCENARIO Init 中使用。此外，OpenSCENARIO 故事中也使用了动作。下面列出了这两个应用领域的支持状态。如果操作包含未列出的子模块，则支持状态适用于所有子模块。 

###### GlobalAction



| GlobalAction                          | 初始化支持                            | Story support                              | 笔记                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `EntityAction`                | &#9989;                          | &#9989;                          |                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `EnvironmentAction`           | &#9989;                         | &#10060;                          |                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `ParameterAction`             | &#9989;                          | &#9989;                          |                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficAction`                                               | &#10060;                          | &#10060;                          |                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficSignalControllerAction`                               | &#10060;                          | &#9989;                          |                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `InfrastructureAction` `TrafficSignalAction`<br>`TrafficSignalStateAction`                                    | &#10060;                          | &#9989;                         | 由于 CARLA 城镇中的交通信号灯没有唯一 ID，因此必须通过提供其位置来设置它们（示例：TrafficSignalStateAction name="pos=x,y" state="green"）。该 ID 也可用于非 CARLA 城镇（示例：TrafficSignalStateAction name="id=n" state="green"）  |

<br>



###### 用户定义的动作



| 用户定义的动作                          | 初始化支持                            | Story support                              | 笔记                                                                                                                                                                                   |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `CustomCommandAction`                       | &#10060;                                        | &#9989;                                       | 此操作当前用于触发附加脚本的执行。示例：type="python /path/to/script args"。  |

<br>


###### PrivateAction


| PrivateAction                          | 初始化支持                            | Story support                              | 笔记                                                                                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |-------------------------------------------------------------------------------------------------|
| `ActivateControllerAction`                          | &#10060;             | &#9989;            | 可用于激活/停用 CARLA 自动驾驶仪。                                                                           |
| `ControllerAction`                                  | &#9989;            | &#9989;            | 支持AssignControllerAction，但必须为控制器实现提供Python 模块，并且在OverrideControllerValueAction 中所有值都需要为`False`。 |
| `LateralAction`<br> `LaneChangeAction`             | &#10060;             | &#9989;            | 目前所有车道变化都有一个线性的dynamicShape，dynamicDimension被定义为距离并且相对于参与者本身（RelativeTargetLane）。               |
| `LateralAction`<br>`LaneOffsetAction`             | &#10060;             | &#9989;             | 目前，所有类型的动态形状都被忽略并取决于控制器。如果偏移量足够高以使车辆退出车道，则此操作可能无法按预期工作。                                         |
| `LateralAction`<br> `LateralDistanceAction`        | &#9989;             | &#9989;             |                                                                                                 |
| `LongitudinalAction`<br> `LongitudinalDistanceAction`| &#10060;             | &#9989;              | 不支持`timeGap`属性                                                                                  |
| `LongitudinalAction`<br> `SpeedAction`             | &#9989;            | &#9989;            |                                                                                                 |
| `SynchronizeAction`                                 | &#10060;             | &#10060;             |                                                                                                 |
| `TeleportAction`                                    | &#9989;            | &#9989;            |                                                                                                 |
| `VisibilityAction`                                  | &#10060;             | &#10060;             |                                                                                                 |
| `RoutingAction`<br> `AcquirePositionAction`        | &#10060;             | &#9989;            |                                                                                                 |
| `RoutingAction`<br> `AssignRouteAction`            | &#10060;             | &#9989;            | 支持路线选项（最短/最快/等）。最短路径表示 A 和 B 之间的直接路径，其他所有路径将使用 A 和 B 之间的路网中的最短路径。                               |
| `RoutingAction`<br> `FollowTrajectoryAction`       | &#10060;             | &#9989;             | 目前仅支持没有时间约束的折线，轨迹跟随模式由控制器决定。                                                                    |

<br>



##### OpenSCENARIO 条件

OpenSCENARIO 中的条件可以定义为 ByEntityCondition 或 ByValueCondition。两者均可用于 StartTrigger 和 StopTrigger 条件。以下两个表列出了每个表的支持状态。

###### ByEntityCondition


| EntityCondition                                | 支持                                        | 笔记                                          |
| ---------------------------------------------- | ---------------------------------------------- | ---------------------------------------------- |
| `AccelerationCondition`                        | &#9989;                                          |                                                |
| `CollisionCondition`                           | &#9989;                                          |                                                |
| `DistanceCondition`                            | &#9989;                                          | \*freespace\* 属性仍然不受支持 |
| `EndOfRoadCondition`                           | &#9989;                                          |                                                |
| `OffroadCondition`                             | &#9989;                                          |                                                |
| `ReachPositionCondition`                       | &#9989;                                          |                                                |
| `RelativeDistanceCondition`                    | &#9989;                                          |                                                |
| `RelativeSpeedCondition`                       | &#9989;                                          |                                                |
| `SpeedCondition`                               | &#9989;                                          |                                                |
| `StandStillCondition`                          | &#9989;                                          |                                                |
| `TimeHeadwayCondition`                         | &#9989;                                          |                                                |
| `TimeToCollisionCondition`                     | &#9989;                                          |                                                |
| `TraveledDistanceCondition`                    | &#9989;                                          |                                                |

<br>

###### ByValueCondition


| ValueCondition            | 支持                   | 笔记                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------------------------------------------- | -------------------------------------------------------- |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ParameterCondition`         | &#9989;                     | 支持级别取决于参数。如果可能的话，建议使用其他条件。另请考虑下面的注释。                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `SimulationTimeCondition`                                    | &#9989;                     |                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `StoryboardElementStateCondition`                            | &#9989;                     | 目前支持 startTransition、stopTransition、endTransition 和completeState。                                                                                                                                                                                                                                                                                                                                                                                        |
| `TimeOfDayCondition`         | &#9989;                     |                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `TrafficSignalCondition`                                     | &#9989;                     | 由于 CARLA 城镇中的交通信号灯没有唯一 ID，因此必须通过提供其位置来设置它们（示例：TrafficSignalCondition name="pos=x,y" state="green"）。该 ID 也可用于非 CARLA 城镇（示例：TrafficSignalCondition name="id=n" state="green"）  |
| `TrafficSignalControllerCondition`                           | &#9989;                      |                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `UserDefinedValueCondition`                                  | &#10060;                      |                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

<br>

!!! 注意
     在 OpenSCENARIO 1.0 标准中，没有定义测试/评估标准的定义。为此，您可以将 StopTrigger 条件与 CARLA 重复使用。通过提供条件的条件名称，通过 ParameterConditions 支持评估条件的以下 StopTrigger 条件：

     * criteria_RunningStopTest
     * criteria_RunningRedLightTest
     * criteria_WrongLaneTest
     * criteria_OnSideWalkTest
     * criteria_KeepLaneTest
     * criteria_CollisionTest
     * criteria_DrivenDistanceTest


##### OpenSCENARIO Positions

在 OpenSCENARIO 中定义位置的方法有多种。下面我们列出了每种定义格式的当前支持状态。

| 位置                          | 支持                            | 笔记                              |
| ---------------------------------------- | ---------------------------------------- | ---------------------------------------- |
| `LanePosition`           | &#9989;                    |                          |
| `RelativeLanePosition`   | &#9989;                    |                          |
| `RelativeObjectPosition` | &#9989;                    |                          |
| `RelativeRoadPosition`   | &#9989;                     |                          |
| `RelativeWorldPosition`  | &#9989;                    |                          |
| `RoadPosition`           | &#9989;                     |                          |
| `RoutePosition`          | &#10060;                     |                          |
| `WorldPosition`          | &#9989;                    |                          |

<br>
