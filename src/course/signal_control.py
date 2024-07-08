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
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
        '--traffic_id',
        metavar='I',
        default=-5,
        type=float,
        help='traffic light id')
    argparser.add_argument(
        '--color_id',
        metavar='C',
        default=1,
        type=int,
        help='traffic light color id')
    argparser.add_argument(
        '--color_time',
        metavar='T',
        default=20,
        type=int,
        help='set traffic light time')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    # 红绿灯的组内id和OpenDriveID不是一一对应的,组内id会发生变化

    # 获取所有红绿灯
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    # print(len(traffic_lights))
    # # 打印红绿灯的位置
    # for traffic_light in traffic_lights:
    #     transform = traffic_light.get_transform()
    #     print(traffic_light.get_opendrive_id())
    #     print(transform)
    #     world.debug.draw_string(transform.location, str(traffic_light.get_opendrive_id())
    #     , life_time=6000, color=carla.Color(255, 0, 0))

    lights_setting = [
        [-6, carla.Transform(carla.Location(x=-220, y=-9, z=5), carla.Rotation(yaw=180))],
        [-5, carla.Transform(carla.Location(x=-260, y=35, z=5), carla.Rotation(yaw=-90))]
            ]
    # 测试
    # spectator = world.get_spectator()
    # spectator.set_transform(carla.Transform(carla.Location(x=-260, y=35, z=5), carla.Rotation(yaw=-90)))
    # 传入参数格式【红绿灯id, 设置路灯颜色， 设置颜色时长】
    # [绿-1， 黄-2， 红-3]

    try:
        # 根据参数获取opendrive_id和设置红绿灯时长color_id, color_time
        # uni_id = lights_setting[0][0]
        # 获得视角
        spectator = world.get_spectator()
        # spectator.set_transform(lights_setting[0][1])

        for setting in lights_setting:
            if setting[0] == args.traffic_id:
                spectator.set_transform(setting[1])
                break

        # 遍历红绿灯找到对应路灯
        for traffic_light in traffic_lights:
            if float(traffic_light.get_opendrive_id()) == args.traffic_id:
                # 设置路灯时长
                if float(args.color_id) == 1:
                    traffic_light.set_green_time(float(args.color_time))
                    print("已设置编号为{}的绿灯时间为{}！".format(args.traffic_id, args.color_time))
                elif float(args.color_id) == 2:
                    traffic_light.set_yellow_time(float(args.color_time))
                    print("已设置编号为{}的黄灯时间为{}！".format(args.traffic_id, args.color_time))
                elif float(args.color_id) == 3:
                    traffic_light.set_red_time(float(args.color_time))
                    print("已设置编号为{}的红灯时间为{}！".format(args.traffic_id, args.color_time))
    except Exception as e:
        print(f"Unexpected Error: {e}")

    # tick应用修改红绿灯
    world.tick()

    settings.synchronous_mode = False
    settings.no_rendering_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)


if __name__ == '__main__':
    main()
