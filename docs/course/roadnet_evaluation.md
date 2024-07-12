# 路网评价

#### **介绍**

[该代码](https://github.com/OpenHUTB/carla_doc/blob/master/course/roadnet_evaluation.py) 用于评估 Carla 仿真环境中的路网质量。评估的指标包括路网覆盖率、路口数量、交通灯数量和道路平均宽度。评估结果可以帮助了解仿真环境中的道路网络结构和配置。
#### **环境配置**

要运行该代码，需要以下软件和库：

- [CARLA仿真器](https://pan.baidu.com/s/15T1hGoWJ70tVmsTX7-zcSw?pwd=hutb)
- Python 3.x
- `carla` Python API
- `geopandas`库
- `shapely`库
- `scikit-learn`库

安装所需库的命令：

```
pip install carla==0.9.15 
pip install geopandas shapely scikit-learn
```

#### **功能结构**

该功能包含以下主要部分：

1. **计算路网覆盖率**
2. **计算路口数量**
3. **计算交通灯数量**
4. **计算道路平均宽度**
5. **仿真流量和真实流量相似性评估**

##### 计算路网覆盖率

将道路几何信息转换为GeoPandas数据框，并计算道路覆盖的总长度和感兴趣区域的总面积，最后计算覆盖率。

```
def road_coverage(world):
    road_geometries = get_road_geometries(world)
    road_lines = [LineString(road_geometries)]
    road_gdf = gpd.GeoDataFrame(geometry=road_lines)
    area_of_interest = Polygon([(-500, -500), (-500, 500), (500, 500), (500, -500)])
    area_gdf = gpd.GeoDataFrame(geometry=[area_of_interest])
    total_road_length = road_gdf.length.sum()
    total_area = area_gdf.area.sum()
    coverage_ratio = total_road_length / total_area
    return coverage_ratio
```

##### 计算路口数量

通过遍历所有路点，找到位于路口的路点并获取唯一的路口ID。

```
def intersection_num(world):
    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(distance=2.0)
    junction_ids = set()
    for waypoint in waypoints:
        if waypoint.is_junction:
            junction = waypoint.get_junction()
            junction_ids.add(junction.id)
    return len(junction_ids)
```

##### 计算交通灯数量

它通过获取所有交通灯的演员对象并计数来实现。

```
def traffic_num(world):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    return len(traffic_lights)
```

##### 计算道路平均宽度

它遍历所有路点，累计车道宽度并计算平均值。

```
def road_width(world):
    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(2.0)
    total_width = 0.0
    lane_count = 0
    for waypoint in waypoints:
        total_width += waypoint.lane_width
        lane_count += 1
    if lane_count > 0:
        average_width = total_width / lane_count
        return average_width
    else:
        return 0
```
##### 仿真流量和真实流量相似性评估<span id=predict_flow ></span>

给定10次真实交通流数据，与仿真获取得10次仿真交通流数据比较其相似性。

###### 获取仿真车流量

```
# 某个时刻西二环十字路口的车流量
def get_vehicles_in_intersection(world, intersection_location, intersection_radius):
    vehicles = world.get_actors().filter('vehicle.*')
    vehicles_in_intersection = []

    for vehicle in vehicles:
        distance = get_distance(vehicle.get_location(), intersection_location)
        if distance <= intersection_radius:
            vehicles_in_intersection.append(vehicle)

    return len(vehicles_in_intersection)
```

###### 计算相似性

```
def anal_similarity(real_flow, sim_flow):
    simulated_traffic = np.array(sim_flow)
    real_traffic = np.array(real_flow)
    # 数据标准化
    simulated_traffic = (simulated_traffic - simulated_traffic.mean()) / simulated_traffic.std()
    real_traffic = (real_traffic - real_traffic.mean()) / real_traffic.std()
    # 计算余弦相似性
    simulated_traffic = simulated_traffic.reshape(1, -1)
    real_traffic = real_traffic.reshape(1, -1)
    cosine_sim = cosine_similarity(simulated_traffic, real_traffic)
    return cosine_sim
```