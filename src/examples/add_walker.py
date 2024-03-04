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
        '--walkernums',
        type=int,
        default=10,
        help='nums of walkers')
    args = argparser.parse_args()
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        #client.load_world('Town01')
        # 设置跑步的行人比例
        percentage_pedestrians_running = 0.25
        # 设置横穿马路的行人比例
        percentage_pedestrians_crossing = 0.15

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

        # 获得整个的blueprint库并从中筛选出行人
        ped_blueprints = world.get_blueprint_library().filter('*pedestrian*')

        # 通过world获得所有可以生成行人的地点并存储
        ped_spawn_points = []
        for i in range(args.walkernums):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                ped_spawn_points.append(spawn_point)

        # 创建用来存储行人，行人速度设置和行人控制器的list
        walker_batch = []
        walker_speed = []
        walker_ai_batch = []

        # 在地图上随机生成num_walkers个行人，每个行人为行人蓝图库中的随机行人，并设定行人的移动速度
        for j in range(args.walkernums):
            walker_bp = random.choice(ped_blueprints)

            # 取消行人无敌状态
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # 设置行人的移动速度
            if walker_bp.has_attribute('speed'):
                if random.random() > percentage_pedestrians_running:
                    # 将对应行人速度设置为走路速度
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # 将对应行人速度设置为跑步速度
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            # 从可生成行人的生成点中随机选择并生成随机行人，之后将生成的行人添加到同一批中
            walker_batch.append(world.try_spawn_actor(walker_bp, random.choice(ped_spawn_points)))

        # 从蓝图库中寻找控制行人行动逻辑的控制器
        walker_ai_blueprint = world.get_blueprint_library().find('controller.ai.walker')

        # 为整批行人各自生成对应的控制器，并把控制器添加到代表批量控制器的列表中
        for walker in world.get_actors().filter('*pedestrian*'):
            walker_ai_batch.append(world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker))

        # 批量启动行人控制器，并设置控制器参数
        for i in range(len(walker_ai_batch)):
            # 启动控制器
            walker_ai_batch[i].start()
            # 通过控制器设置行人的目标点
            walker_ai_batch[i].go_to_location(world.get_random_location_from_navigation())
            # 通过控制器设置行人的行走速度
            walker_ai_batch[i].set_max_speed(float(walker_speed[i]))

        # 设置行人横穿马路的参数
        world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)

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

        # 令摄像头读取数据并存储(异步模式)
        #camera.listen(lambda image: image.save_to_disk(output_path % image.frame))

        while True:

            # 如果为同步模式设定
            if traffic_manager.synchronous_mode:
                # 更新模拟世界
                world.tick()
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

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
