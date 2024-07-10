"""
    湖工商场景智能信号控制
    传入参数格式【红绿灯opendrive_id,颜色id， 设置时长】
    红绿灯对应编号:[绿-1， 黄-2， 红-3]
    红绿灯设置时间:以s为单位
"""

import glob
import os
import sys
import carla
import argparse
import json
from flask import Flask, jsonify, request

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

app = Flask(__name__)


@app.route('/set_traffic_light', methods=['GET'])
def set_traffic_light():
    # data = request.json

    traffic_id = float(request.args.get('traffic_id'))
    color_id = int(request.args.get('color_id'))
    color_time = int(request.args.get('color_time'))

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    settings = world.get_settings()
    # settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_lights = world.get_actors().filter('traffic.traffic_light')

    init_time = 0
    color_name = None

    for traffic_light in traffic_lights:
        if float(traffic_light.get_opendrive_id()) == traffic_id:
            if color_id == 1:
                init_time = traffic_light.get_green_time()
            elif color_id == 2:
                init_time = traffic_light.get_yellow_time()
            elif color_id == 3:
                init_time = traffic_light.get_red_time()
    lights_setting = [
        [-6, carla.Transform(carla.Location(x=-220, y=-9, z=5), carla.Rotation(yaw=180))],
        [-5, carla.Transform(carla.Location(x=-260, y=35, z=5), carla.Rotation(yaw=-90))]
    ]
    try:
        spectator = world.get_spectator()
        for setting in lights_setting:
            if setting[0] == traffic_id:
                spectator.set_transform(setting[1])
                break

        for traffic_light in traffic_lights:
            if float(traffic_light.get_opendrive_id()) == traffic_id:
                if color_id == 1:
                    traffic_light.set_green_time(float(color_time))
                    color_name = "绿色"
                elif color_id == 2:
                    traffic_light.set_yellow_time(float(color_time))
                    color_name = "黄色"
                elif color_id == 3:
                    traffic_light.set_red_time(float(color_time))
                    color_name = "红色"

        response_data = {
            "id": traffic_id,
            "color": color_name,
            "init_time": init_time,
            "last_time": color_time
        }

        # tick应用修改红绿灯
        world.wait_for_tick()

        # settings.synchronous_mode = False
        # settings.no_rendering_mode = False
        # settings.fixed_delta_seconds = None
        # world.apply_settings(settings)

        return jsonify(response_data)

    except Exception as e:
        print(f"Unexpected Error: {e}")
        return jsonify({"error": str(e)})


if __name__ == '__main__':
    app.run(debug=True, port=5000)
