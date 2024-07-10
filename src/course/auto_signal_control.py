"""
    根据路口交通流量的大小来修改路口红绿灯的时长
"""

import glob
import os
import sys
import carla
import argparse
import json
from flask import Flask, jsonify, request
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# 路口中心位置
center_location = [
    carla.Location(x=434, y=-16, z=0)
]
spectator_transform = [
     carla.Transform(carla.Location(x=387.824249, y=-16.431578, z=57.161552), carla.Rotation(pitch=-53.401241, yaw=-0.648651, roll=-0.000305))
]
intersection_radius = 100

traffics_in_junction = set()

GREEN_TIME = 10

app = Flask(__name__)


def get_distance(location1, location2):
    return math.sqrt(
        (location1.x - location2.x) ** 2 + (location1.y - location2.y) ** 2 + (location1.z - location2.z) ** 2)


def get_vehicles_in_intersection(world, intersection_location, intersection_radius):
    vehicles = world.get_actors().filter('vehicle.*')
    vehicles_in_intersection = []

    for vehicle in vehicles:
        distance = get_distance(vehicle.get_location(), intersection_location)
        if distance <= intersection_radius:
            vehicles_in_intersection.append(vehicle)

    return len(vehicles_in_intersection)


# 得到这路口对象
def get_junction_at_location(world, location):
    map = world.get_map()
    waypoint = map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    junction = waypoint.get_junction()
    return junction


# 获得这个路口的全部红绿灯
def get_traffic_lights_in_junction(world, junction):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in traffic_lights:
        if get_distance(traffic_light.get_location(), center_location[0]) <= 50:
            id = traffic_light.get_opendrive_id()
            traffics_in_junction.add(id)


def get_light(all_traffic_lights):
    drive_list = list(traffics_in_junction)
    traffic_lights = []
    for traffic_light in all_traffic_lights:
        if traffic_light.get_opendrive_id() in drive_list:
            traffic_lights.append(traffic_light)
    return traffic_lights


@app.route('/set_traffic_light', methods=['GET'])
def set_traffic_light():
    # data = request.json
    # 不接收参数
    # traffic_id = float(request.args.get('traffic_id'))
    # color_id = int(request.args.get('color_id'))
    # color_time = int(request.args.get('color_time'))

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    all_traffic_lights = world.get_actors().filter('traffic.traffic_light')

    # init_time = 0
    # color_name = None

    # 得到这个路口对象
    junction = get_junction_at_location(world, center_location[0])
    # 将该路口红绿灯添加到列表s
    get_traffic_lights_in_junction(world, junction)
    traffic_id = 0
    init_green_time = 0
    traffic_ids = []
    traffic_init_green_time = []
    # 输出这个路口红绿灯的信息

    for traffic_light in get_light(all_traffic_lights):
        traffic_id = traffic_light.get_opendrive_id()
        init_green_time = traffic_light.get_green_time()
        traffic_ids.append(traffic_id)
        traffic_init_green_time.append(init_green_time)
    # 得到路口流量
    junction_flow = get_vehicles_in_intersection(world, center_location[0], intersection_radius)
    print(junction_flow)
    # lights_setting = [
    #     [-6, carla.Transform(carla.Location(x=-220, y=-9, z=5), carla.Rotation(yaw=180))],
    #     [-5, carla.Transform(carla.Location(x=-260, y=35, z=5), carla.Rotation(yaw=-90))]
    # ]
    last_green_time = 0
    # 根据路口流量来设置红绿灯
    if junction_flow > 5:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 5)
            last_green_time = GREEN_TIME + 5
    elif junction_flow > 10:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 10)
            last_green_time = GREEN_TIME + 10
    elif junction_flow > 15:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 15)
            last_green_time = GREEN_TIME + 15
    else:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME)
            last_green_time = GREEN_TIME
    try:
        # spectator = world.get_spectator()
        # for setting in lights_setting:
        #     if setting[0] == traffic_id:
        #         spectator.set_transform(setting[1])
        #         break
        # response_data = {
        #     "id": traffic_id,
        #     "color": color_name,
        #     "init_time": init_time,
        #     "last_time": color_time
        # }
        print(len(traffics_in_junction))
        response_data = {
            "traffic_lights": [
                {
                    "id": traffic_ids,
                },
                {
                    "init_green_time": traffic_init_green_time,
                },
                {
                    "last_green_time": last_green_time
                }
            ]
        }

        # tick应用修改红绿灯
        world.tick()

        settings.synchronous_mode = False
        # settings.no_rendering_mode = False
        # settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        return jsonify(response_data)

    except Exception as e:
        print(f"Unexpected Error: {e}")
        return jsonify({"error": str(e)})


if __name__ == '__main__':
    app.run(debug=True, port=5000)
