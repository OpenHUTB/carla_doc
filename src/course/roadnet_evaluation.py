"""
     路网评价
"""

import carla
import geopandas as gpd
from shapely.geometry import LineString, Polygon


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


def main():
    # 连接到CARLA仿真服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

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