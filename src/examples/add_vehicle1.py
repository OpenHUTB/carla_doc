import math
import os
import queue
import random
import sys

import carla
import argparse
import logging
from numpy import random


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--vehiclenums',
        type=int,
        default=20,
        help='nums of vehicle')
    args = argparser.parse_args()
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        #client.load_world('Town01')
        #num_walkers = 20
        #num_vehicle = 20
        # 获取我们client所对应的world
        world = client.get_world()
        # 获得这个world中的观察者
        spectator = world.get_spectator()

        # 获得当前客户端的交通管理器
        traffic_manager = client.get_trafficmanager()

        # 获得观察者的方位信息
        transform = spectator.get_transform()

        # 根据观察者默认方位设置新的方位
        location = transform.location + carla.Location(x=-30, z=20)
        rotation = carla.Rotation(pitch=-20, yaw=-20, roll=0)
        new_transform = carla.Transform(location, rotation)

        # 将观察者设置到新方位上
        spectator.set_transform(new_transform)

        # 获得整个的blueprint库并从中筛选出车辆
        vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')



        # 通过world获得map并获得所有可以生成车辆的地点
        vehicle_spawn_points = world.get_map().get_spawn_points()

        # 在地图上随机生成num_vehicle辆车，每辆车为车辆蓝图库中的随机车辆
        for i in range(0, args.vehiclenums):
            world.try_spawn_actor(random.choice(vehicle_blueprints),
                                  random.choice(vehicle_spawn_points))

        # 从world中获取车辆，并将每辆车的autopilot模式设置为打开
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.set_autopilot()


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


        while True:

            # 如果为同步模式设定
            if traffic_manager.synchronous_mode:
                # 更新模拟世界
                world.tick()
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

        # 获得当前模拟世界设定
        settings = world.get_settings()
        # 设定为异步模式
        settings.synchronous_mode = False
        # 设定为可变时间步长
        settings.fixed_delta_seconds = None
        # 应用设定
        world.apply_settings(settings)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
