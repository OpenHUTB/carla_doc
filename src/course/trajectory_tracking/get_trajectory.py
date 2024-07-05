import glob
import os
import sys
import time
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import argparse
from numpy import random

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
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else 42)
    try:
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(args.tm_port)

        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True

        settings.fixed_delta_seconds = 0.02

        spawn_points = world.get_map().get_spawn_points()
        for ind, spawn_point in enumerate(spawn_points):
            world.debug.draw_string(spawn_point.location,str(ind), life_time=1000, color=carla.Color(255, 0, 0))
        vehicle_bp = world.get_blueprint_library().find('vehicle.audi.a2')

        # transform = carla.Transform(carla.Location(x=4624, y=561,z=0.3), carla.Rotation(yaw=180))
        vehicle_1 = world.try_spawn_actor(vehicle_bp, spawn_points[216])
        # print("yaw1:", vehicle_1.get_transform().rotation.yaw)
        vehicle_2 = world.try_spawn_actor(vehicle_bp, spawn_points[11])
        # print("yaw2:", vehicle_2.get_transform().rotation.yaw)
        vehicle_1.set_autopilot()
        vehicle_2.set_autopilot()
        initial_simulation_time = world.get_snapshot().timestamp.elapsed_seconds
        while True:
            world.tick()
            # print("yaw1:", vehicle_1.get_transform().rotation.yaw)
            # # 获取车辆的位置
            location = vehicle_1.get_location()
            x_1 = location.x
            y_1 = location.y
            #
            location = vehicle_2.get_location()
            x_2 = location.x
            y_2 = location.y
            #
            # 获取当前仿真时间
            current_simulation_time = world.get_snapshot().timestamp.elapsed_seconds
            # 计算相对于初始时间的经过时间
            elapsed_time = current_simulation_time - initial_simulation_time
            #
            # # 打印车辆位置和仿真时间
            print(f"{0}, {x_1}, {y_1}, {elapsed_time}")
            print(f"{1}, {x_2}, {y_2}, {elapsed_time}")
    finally:

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
