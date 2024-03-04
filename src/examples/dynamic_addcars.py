#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA 动态天气：

连接到 CARLA 仿真器实例并控制天气。随着时间的推移，太阳的位置会平稳地变化，偶尔会产生风暴。

"""

import glob
import os
import sys

import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import math


def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


def draw_waypoints(waypoints, road_id=None, life_time=50.0):
 
  for waypoint in waypoints:
     if(waypoint.road_id == road_id):
          world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)
 






def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = 0.1 / speed_factor

    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    spectator = world.get_spectator()
    # 获得当前客户端的交通管理器
    traffic_manager = client.get_trafficmanager()

    # 获得当前模拟世界的设定
    setting = world.get_settings()
    # 设定为异步模式
    setting.synchronous_mode = True
    # 将时间步长设定为固定的0.03秒
    setting.fixed_delta_seconds = 0.05
    # 应用设定
    world.apply_settings(setting)

    # 将交通管理器设置为同步模式
    traffic_manager.synchronous_mode = True
    # 通过交通管理器设置所有车辆相对于限速的差值，这里为负即为所有车辆都会i超速行驶
    traffic_manager.global_percentage_speed_difference(-30)
    elapsed_time = 0.0
    road_id=2
    filtered_waypoints = []
    # 获得整个的blueprint库并从中筛选出车辆
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    # 以距离为1的间距创建waypoints
    waypoints = world.get_map().generate_waypoints(distance=1.0)
    # life_time 为画出的辅助标志存活时间

    for waypoint in waypoints:
        if (waypoint.road_id == road_id):
            world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=20,
                                    persistent_lines=True)
    for waypoint in waypoints:
      if(waypoint.road_id == road_id):
        filtered_waypoints.append(waypoint)
    spawn_point = filtered_waypoints[0].transform
    spawn_point.location.z += 1
    #bp为blueprint制造出来的小车
    vehicle = world.spawn_actor(random.choice(vehicle_blueprints), spawn_point)
    vehicle.set_autopilot()
    while True:
        # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
        # world.get_spectator().set_transform(camera.get_transform())

        # 如果为同步模式设定
        if traffic_manager.synchronous_mode:
            # 更新模拟世界
            world.tick()
            # 从队列中读取传感器图像
            # image = image_queue.get()
            # 将图像存储到本地路径(同步模式)
            # image.save_to_disk(output_path % image.frame)
        # 如果为异步模式设定
        else:
            # 更新模拟世界
            world.wait_for_tick()

    finally:
    # 停止并销毁所有controller
    for controller in world.get_actors().filter('*controller*'):
        controller.stop()
    # 销毁所有车辆
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.destroy()
    # 销毁所有行人
    for walker in world.get_actors().filter('*walker*'):
        walker.destroy()

    # 获得当前模拟世界设定
    settings = world.get_settings()
    # 设定为异步模式
    settings.synchronous_mode = False
    # 设定为可变时间步长
    settings.fixed_delta_seconds = None
    # 应用设定
    world.apply_settings(settings)


if __name__ == '__main__':

    main()
