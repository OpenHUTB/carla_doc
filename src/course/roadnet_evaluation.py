"""
     路网评价
"""
import math
import carla
import geopandas as gpd
import numpy as np
from shapely.geometry import LineString, Polygon
from sklearn.metrics.pairwise import cosine_similarity
# 车流量计算半径
intersection_radius = 100

# 假设该路口车流量数据如下
real_flow = [13, 14, 8, 3, 5, 1, 4, 2, 20, 6]
# 仿真车流量
sim_flow = []

# 路口中心位置
center_location = [
    carla.Location(x=434, y=-16, z=0)
]


def get_road_geometries(world):
    road_geometries = []
    for waypoint in world.get_map().generate_waypoints(2.0):
        transform = waypoint.transform
        location = transform.location
        road_geometries.append((location.x, location.y))
    return road_geometries


# 计算路网覆盖率
def road_coverage(world):
    # 获取道路几何信息
    road_geometries = get_road_geometries(world)

    # 创建GeoPandas GeoDataFrame
    road_lines = [LineString(road_geometries)]
    road_gdf = gpd.GeoDataFrame(geometry=road_lines)

    # 假设感兴趣区域为一个矩形（可以根据实际情况调整）
    area_of_interest = Polygon([(-500, -500), (-500, 500), (500, 500), (500, -500)])
    area_gdf = gpd.GeoDataFrame(geometry=[area_of_interest])

    # 计算道路覆盖的总长度（或总面积，如果道路宽度已知）
    total_road_length = road_gdf.length.sum()

    # 计算感兴趣区域的总面积
    total_area = area_gdf.area.sum()

    # 计算路网覆盖率
    coverage_ratio = total_road_length / total_area
    return coverage_ratio


# 计算路口数量
def intersection_num(world):
    carla_map = world.get_map()

    # 获取所有路点
    waypoints = carla_map.generate_waypoints(distance=2.0)

    # 存储路口唯一ID,保证每个路口只计算一次
    junction_ids = set()
    for waypoint in waypoints:
        if waypoint.is_junction:
            junction = waypoint.get_junction()
            junction_ids.add(junction.id)
    return len(junction_ids)


# 计算交通灯总数
def traffic_num(world):
    # 获取所有红绿灯
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    return len(traffic_lights)


# 计算道路平均宽度
def road_width(world):
    carla_map = world.get_map()
    # 获取道路几何信息
    waypoints = carla_map.generate_waypoints(2.0)
    total_width = 0.0
    lane_count = 0

    for waypoint in waypoints:
        transform = waypoint.transform
        location = transform.location

        total_width += waypoint.lane_width
        lane_count += 1

    # 计算平均值
    if lane_count > 0:
        average_width = total_width / lane_count
        return average_width
    else:
        return 0


# 某个时刻西二环十字路口的车流量
def get_vehicles_in_intersection(world, intersection_location, intersection_radius):
    vehicles = world.get_actors().filter('vehicle.*')
    vehicles_in_intersection = []

    for vehicle in vehicles:
        distance = get_distance(vehicle.get_location(), intersection_location)
        if distance <= intersection_radius:
            vehicles_in_intersection.append(vehicle)

    return len(vehicles_in_intersection)


def get_distance(location1, location2):
    return math.sqrt(
        (location1.x - location2.x) ** 2 + (location1.y - location2.y) ** 2 + (location1.z - location2.z) ** 2)


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


def main():
    # 连接到CARLA仿真服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 路网覆盖率
    coverage_ratio = road_coverage(world)
    print(f"路网覆盖率: {coverage_ratio:.2%}")

    # 路口个数
    junction_num = intersection_num(world)
    print(f"路口个数: {junction_num}")

    # 交通灯数量
    traffic = traffic_num(world)
    print(f"交通灯个数: {traffic}")

    # 道路平均宽度
    ave_width = road_width(world)
    print(f"道路平均宽度: {ave_width:.2f}")
    simple_time = 0

    while True:

        world.tick()
        simple_time += 1
        if simple_time % 50 == 0:
            sim_num = get_vehicles_in_intersection(world, center_location[0], intersection_radius)
            sim_flow.append(sim_num)
            if len(sim_flow) == 10:
                break
    # 分析仿真流量和真实流量的相似性
    sim_value = (anal_similarity(real_flow, sim_flow) + 1) / 2
    print(sim_flow)
    print(f"Cosine Similarity: {sim_value[0][0]}")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
