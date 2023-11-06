# 地图和导航

在讨论了世界及其参与者之后，是时候将所有内容都放在适当的位置并了解地图以及参与者如何导航。

- [__地图__](#the-map)  
	- [改变地图](#changing-the-map)  
	- [地标](#landmarks)  
	- [车道](#lanes)  
	- [路口](#junctions)  
	- [航点](#waypoints)  
	- [环境对象](#environment-objects)
- [__在 CARLA 中导航__](#navigation-in-carla)  
	- [通过航路点导航](#navigating-through-waypoints)  
	- [生成地图导航](#generating-a-map-navigation)  
- [__CARLA 地图__](#carla-maps)  
	- [非分层地图](#non-layered-maps)
	- [分层地图](#layered-maps)
- [__自定义地图__](#custom-maps)
	- [概述](tuto_M_custom_map_overview.md)
	- [道路涂装 painting](tuto_M_custom_road_painter.md)
	- [定制建筑](tuto_M_custom_buildings.md) 
	- [生成地图](tuto_M_generate_map.md)
	- [添加地图包](tuto_M_add_map_package.md)
	- [添加地图源](tuto_M_add_map_source.md)
	- [替代方法](tuto_M_add_map_alternative.md)


---
## 地图

地图包括城镇的三维模型及其道路定义。地图的道路定义基于 OpenDRIVE 文件，这是一种标准化的带注释的道路定义格式。[OpenDRIVE 1.4 标准](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf) 定义道路、车道、路口等方式决定了 Python API 的功能以及决策背后的推理。

Python API 充当高级查询系统来导航这些道路。它不断发展以提供更广泛的工具集。


### 改变地图

__要改变地图，世界也必须改变__。将从头开始重新创建仿真。您可以在新世界中使用相同的地图重新启动，也可以更改地图和世界：

- `reload_world()` 使用相同的地图创建世界的新实例。
- `load_world()` 改变当前地图并创建一个新世界。

```py
world = client.load_world('Town01')
```

每个地图都有一个与当前加载的城市名称匹配的`name`属性，例如 _Town01_ 。要获取可用地图的列表：

```py
print(client.get_available_maps())
```

### 地标

OpenDRIVE 文件中定义的交通标志将转换为 Carla，作为可从 API 查询的地标对象。以下方法和类可用于操作和使用地标对象：

- __[`carla.Landmark`](https://carla.readthedocs.io/en/latest/python_api/#carla.Landmark)__ 对象代表 OpenDRIVE 信号。该类的属性和方法描述了地标及其影响区域。
	- [`carla.LandmarkOrientation`](https://carla.readthedocs.io/en/latest/python_api/#carla.LandmarkOrientation) 说明地标相对于道路几何定义的方向。
	- [`carla.LandmarkType`](https://carla.readthedocs.io/en/latest/python_api/#carla.LandmarkType) 包含常见的地标类型，以便于转换为 OpenDRIVE 类型。
- __[`carla.Waypoint`](https://carla.readthedocs.io/en/latest/python_api/#carla.Waypoint)__ 可以获得位于其前方一定距离的地标。可以指定要获取的地标类型。
- __[`carla.Map`](https://carla.readthedocs.io/en/latest/python_api/#carla.Map)__ 检索地标集。它可以返回地图中的所有地标，或者具有共同 ID、类型或组的地标。
- __[`carla.World`](https://carla.readthedocs.io/en/latest/python_api/#carla.World)__ 充当地标、仿真中代表他们的`carla.TrafficSign` 和 `carla.TrafficLight` 之间的中间。

```py
my_waypoint.get_landmarks(200.0,True)
``` 

### 路径点

[`carla.Waypoint`](python_api.md#carla.Waypoint) 是 Carla 世界中的三维有向点，对应于 OpenDRIVE 车道。与路径点相关的所有事情都发生在客户端；只需与服务器通信一次即可获取包含航点信息的[地图对象](python_api.md#carlamap)。

每个路径点都包含一个 [`carla.Transform`](python_api.md#carla.Transform) ，它说明其在地图上的位置以及包含该航路点的车道的方向。变量`road_id`,`section_id`,`lane_id` 和 `s`对应 OpenDRIVE 道路。路径点`id`的是由这四个值的哈希组合构造的。


!!! 注意
    __同一道路内距离小于 2cm__ 的路点共享相同的路点 `id`。

路径点保存有关包含该路径点的 __车道__ 的信息。此信息包括车道的左车道和右 __车道标记__、用于确定其是否位于路口内的布尔值、车道类型、宽度和车道变更权限。

```py
# 从路径点访问车道信息
inside_junction = waypoint.is_junction()
width = waypoint.lane_width
right_lm_color = waypoint.right_lane_marking.color
```

### 车道


[OpenDRIVE 1.4 标准](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf) 定义的通道类型将 [carla.LaneType](python_api.md#carla.LaneType) 作为一系列枚举值转换为 API 。


车道周围的车道标记可通过 [`carla.LaneMarking`](python_api.md#carla.LaneMarking) 访问。车道标记由一系列变量定义：

- __color:__ [`carla.LaneMarkingColor`](python_api.md#carla.LaneMarkingColor) 是定义标记颜色的枚举值。
- __lane_change:__ [`carla.LaneChange`](python_api.md#carla.LaneChange) 说明车道是否允许左转、右转、两者都允许或不允许。
- __type:__ [`carla.LaneMarkingType`](python_api.md#carla.LaneMarkingType) 是根据 OpenDRIVE 标准定义标记类型的枚举值。
- __width:__ 定义标记的厚度。

下面的示例显示了获取有关特定路径点的车道类型、车道标记和车道变更权限的信息：

```py
# 获得路径点车道类型
lane_type = waypoint.lane_type

# 获得左车道标记的类型
left_lanemarking_type = waypoint.left_lane_marking.type()

# 获得该路径点可得的车道改变
lane_change = waypoint.lane_change
```

### 路口

[`carla.Junction`](python_api.md#carla.Junction) 代表 OpenDRIVE 连接点。此类包含带有边界框的交汇处，用于识别其中的车道或车辆。

该 `carla.Junction` 类包含 `get_waypoints` 为交汇处的每个车道返回一对路点的方法。每对位于交界处边界的起点和终点。

```py
waypoints_junc = my_junction.get_waypoints()
```

### 环境对象

Carla 地图上的每个对象都有一组关联的变量，可以在 [此][env_obj] 处找到这些变量。这些变量中包含一个 [唯一 ID][env_obj_id]，可用于切换该对象在地图上的可见性。您可以使用 Python API 根据每个环境对象的[语义标签]([semantic_tag]) [获取][fetch_env_obj] 其 ID ：

		# 获得世界中的建筑
	    world = client.get_world()
		env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

		# 访问各个建筑 IDs 并保存在集合当中
		building_01 = env_objs[0]
		building_02 = env_objs[1]
		objects_to_toggle = {building_01.id, building_02.id}

		# 切换建筑为不可见
		world.enable_environment_objects(objects_to_toggle, False)
		# 切换建筑为可见
		world.enable_environment_objects(objects_to_toggle, True)

See an example of distinct objects being toggled:

![toggle_objects_gif](img/objects_small.gif)

[env_obj]: https://carla.readthedocs.io/en/latest/python_api/#carla.EnvironmentObject
[env_obj_id]: https://carla.readthedocs.io/en/latest/python_api/#carla.EnvironmentObject.id
[toggle_env_obj]: https://carla.readthedocs.io/en/latest/python_api/#carla.World.enable_environment_objects
[fetch_env_obj]: https://carla.readthedocs.io/en/latest/python_api/#carla.World.get_environment_objects
[semantic_tag]: https://carla.readthedocs.io/en/latest/python_api/#carla.CityObjectLabel


---
## Navigation in CARLA

Navigation in CARLA is managed via the Waypoint API, a combination of methods from [`carla.Waypoint`](python_api.md#carla.Waypoint) and [`carla.Map`](python_api.md#carla.Map).

The client must initially communicate with the server to retrieve the map object containing the waypoint information. This is only required once, all subsequent queries are performed on the client side.

### Navigating through waypoints

The Waypoint API exposes methods that allow waypoints to connect to each other and construct a path along a road for vehicles to navigate:

- `next(d)` creates a list of waypoints within an approximate distance, `d`, __in the direction of the lane__. The list contains one waypoint for each possible deviation.
- `previous(d)` creates a list of waypoints waypoint within an approximate distance, `d`, __in the opposite direction of the lane__. The list contains one waypoint for each possible deviation.
- `next_until_lane_end(d)` and `previous_until_lane_start(d)` return a list of waypoints a distance `d` apart. The lists go from the current waypoint to the end and beginning of its lane, respectively.
- `get_right_lane()` and `get_left_lane()` return the equivalent waypoint in an adjacent lane, if one exists. A lane change maneuver can be made by finding the next waypoint to the one on its right/left lane, and moving to it.

```py
# Find next waypoint 2 meters ahead.
waypoint = waypoint.next(2.0)
```

### Generating map navigation

The client needs to make a request to the server to get the `.xodr` map file and parse it to a [`carla.Map`](python_api.md#carla.Map) object. This only needs to be done once.

To get the map object:

```py
map = world.get_map()
```

The map object contains __recommended spawn points__ for the creation of vehicles. You can get a list of these spawn points, each one containing a [`carla.Transform`](python_api.md#carlatransform), using the method below. Bear in mind that the spawn points may be occupied already, resulting in failed creation of vehicles due to collisions.

```py
spawn_points = world.get_map().get_spawn_points()
```

You can get started with waypoints by __[getting](python_api.md#carla.Map.get_waypoint) the closest waypoint__ to a specific location or to a particular `road_id`, `lane_id` and `s` value in the map's OpenDRIVE definition:

```py
# Nearest waypoint in the center of a Driving or Sidewalk lane.
waypoint01 = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

#Nearest waypoint but specifying OpenDRIVE parameters. 
waypoint02 = map.get_waypoint_xodr(road_id,lane_id,s)
```

The below example shows how to __generate a collection of waypoints__ to visualize the city lanes. This will create waypoints all over the map, for every road and lane. All of them will approximately 2 meters apart:

```py
waypoint_list = map.generate_waypoints(2.0)
```

To __generate a minimal graph of road topology__, use the example below. This will return a list of pairs (tuples) of waypoints. The first element in each pair connects with the second element and both define the start and end points of each lane in the map. More information on this method is found in the [PythonAPI](python_api.md#carla.Map.get_topology).

```py
waypoint_tuple_list = map.get_topology()
```

The example below __converts a `carla.Transform` to geographical latitude and longitude coordinates,__ in the form of a [`carla.GeoLocation`](python_api.md#carla.GeoLocation):

```py
my_geolocation = map.transform_to_geolocation(vehicle.transform)
```

Use the following example to __save road information__ in OpenDRIVE format to disk:

```py
info_map = map.to_opendrive()
```

---
## CARLA maps

There are eight towns in the CARLA ecosystem and each of those towns have two kinds of map, non-layered and layered. [Layers][layer_api] refer to the grouped objects within a map and consist of the following:

- NONE
- Buildings
- Decals
- Foliage
- Ground
- ParkedVehicles
- Particles
- Props
- StreetLights
- Walls
- All

[layer_api]: https://carla.readthedocs.io/en/latest/python_api/#carlamaplayer

### Non-layered maps

Non-layered maps are shown in the table below (click the town name to see an overhead image of the layout). All of the layers are present at all times and cannot be toggled on or off in these maps. Up until CARLA 0.9.11, these were the only kinds of map available.

!!! Note
    Users can [customize a map](tuto_A_map_customization.md) or even [create a new map](tuto_M_custom_map_overview.md) to be used in CARLA.

| Town       | Summary |
| -----------| ------  |
| **[Town01](img/Town01.jpg)** | A basic town layout consisting of "T junctions".|
| **[Town02](img/Town02.jpg)** | Similar to **Town01**, but smaller.|
| **[Town03](img/Town03.jpg)** | The most complex town, with a 5-lane junction, a roundabout, unevenness, a tunnel, and more.|
| **[Town04](img/Town04.jpg)** | An infinite loop with a highway and a small town.|
| **[Town05](img/Town05.jpg)** | Squared-grid town with cross junctions and a bridge. It has multiple lanes per direction. Useful to perform lane changes.  |
| **[Town06](img/Town06.jpg)** | Long highways with many highway entrances and exits. It also has a [**Michigan left**](<https://en.wikipedia.org/wiki/Michigan_left>). |
| **[Town07](img/Town07.jpg)** | A rural environment with narrow roads, barns and hardly any traffic lights. |
| **[Town10](img/Town10.jpg)** | A city environment with different environments such as an avenue or promenade, and more realistic textures.|

### Layered maps

The layout of layered maps is the same as non-layered maps but it is possible to toggle off and on the layers of the map. There is a minimum layout that cannot be toggled off and consists of roads, sidewalks, traffic lights and traffic signs. Layered maps can be identified by the suffix `_Opt`, for example, `Town01_Opt`. With these maps it is possible to [load][load_layer] and [unload][unload_layer] layers via the Python API:

		# Load layered map for Town 01 with minimum layout plus buildings and parked vehicles
		world = client.load_world('Town01_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles)

		# Toggle all buildings off
		world.unload_map_layer(carla.MapLayer.Buildings)

		# Toggle all buildings on	
		world.load_map_layer(carla.MapLayer.Buildings)

[load_layer]: https://carla.readthedocs.io/en/latest/python_api/#carla.World.load_map_layer
[unload_layer]: https://carla.readthedocs.io/en/latest/python_api/#carla.World.unload_map_layer

See an example of all layers being loaded and unloaded in sequence:

![map-layers](img/sublevels.gif)


---

## Custom maps

CARLA is designed to be extensible and highly customisable for specialist applications. Therefore, in addition to the many maps and assets already avaiable in CARLA out of the box, it is possible to create and import new maps, road networks and assets to populate bespoke environments in a CARLA simulation. The following documents detail the steps needed to build and integrate custom maps:  

* [__Overview__](tuto_M_custom_map_overview.md)
* [__Road painting__](tuto_M_custom_road_painter.md)
* [__Custom buildings__](tuto_M_custom_buildings.md) 
* [__Generate map__](tuto_M_generate_map.md)
* [__Add map package__](tuto_M_add_map_package.md)
* [__Add map source__](tuto_M_add_map_source.md)
* [__Alternative methods__](tuto_M_add_map_alternative.md)
