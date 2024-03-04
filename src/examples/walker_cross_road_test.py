
#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""闯红灯时其他车辆等待场景"""
import glob
import os
import random
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import math
import carla
import argparse
BRAKE_DISTANCE = 15


# 由刹车和距离来判断是否切换视角
def vehicle_coming(world, walker):
    for actor in world.get_actors().filter("vehicle.*"):
        actual_distance = math.sqrt((walker.get_location().x - actor.get_location().x) ** 2 + (
                walker.get_location().y - actor.get_location().y) ** 2 + (
                                            walker.get_location().z - actor.get_location().z) ** 2)
        control = actor.get_control()

        if control.brake > 0.8 and actual_distance < 10:
            return True

    return False


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


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
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprints_Walkers = get_actor_blueprints(world, args.filterw, args.generationw)

    transform = carla.Transform(carla.Location(x=93, y=93, z=0.3), carla.Rotation(yaw=0))
    walker = world.try_spawn_actor(random.choice(blueprints_Walkers), transform)
    # transform_1 = carla.Transform(carla.Location(x=-88, y=140, z=0.3), carla.Rotation(yaw=0))
    transform_1 = carla.Transform(carla.Location(x=-57, y=93, z=0.3), carla.Rotation(yaw=0))
    walker_1 = world.try_spawn_actor(random.choice(blueprints_Walkers), transform_1)

    # control walker
    control = carla.WalkerControl(carla.Vector3D(x=5, y=0, z=0), speed=0.5)
    walker.apply_control(control)
    control_1 = carla.WalkerControl(carla.Vector3D(x=5, y=-15, z=0), speed=0.5)
    # control_1 = carla.WalkerControl(carla.Vector3D(x=5, y=0, z=0), speed=0.5)
    walker_1.apply_control(control_1)

    # [walker, stop, respawn, transform,disappear_distance,sp_lo, speed]
    spectator_location = carla.Location(x=93, y=75, z=3)
    spectator_location_1 = carla.Location(x=-57, y=75, z=3)
    walker_list = [
        [walker, False, False, transform, 113, spectator_location]

    ]
    # [walker_1, False, False, transform_1, -38, spectator_location_1]
    display = False
    while True:
        for walker_list_item in walker_list:
            # 判断行人前面是否有车，是则停止
            for actor in world.get_actors().filter("vehicle.*"):
                w_x = walker_list_item[0].get_location().x
                _x = actor.get_location().x
                w_y = walker_list_item[0].get_location().y
                _y = actor.get_location().y
                wv_distance = math.sqrt((walker_list_item[0].get_location().x - actor.get_location().x) ** 2 + (
                        walker_list_item[0].get_location().y - actor.get_location().y) ** 2 + (
                                                walker_list_item[0].get_location().z - actor.get_location().z) ** 2)
                if w_y <= (_y + 2) and w_y >= (_y - 2) and wv_distance < 6:
                    walker_list_item[1] = True

            if walker_list_item[1]:
                walker_control = walker_list_item[0].get_control()
                walker_control.speed = 0
                walker_list_item[0].apply_control(walker_control)
                walker_list_item[1] = False
            else:
                walker_control = walker_list_item[0].get_control()
                walker_control.speed = 0.5
                walker_list_item[0].apply_control(control)

            if walker_list_item[0].get_location().x > walker_list_item[4]:
                walker_list_item[0].destroy()
                walker_list_item[2] = True

            #  respawn walker
            if walker_list_item[2]:
                walker_list_item[0] = world.try_spawn_actor(random.choice(blueprints_Walkers), walker_list_item[3])
                control = carla.WalkerControl(carla.Vector3D(x=5, y=0, z=0), speed=0.5)
                walker_list_item[0].apply_control(control)
                walker_list_item[2] = False

            if vehicle_coming(world, walker_list_item[0]) and not display:
                spectator = world.get_spectator()
                _x = walker_list_item[0].get_location().x
                _y = walker_list_item[0].get_location().y
                spectator_transform = carla.Transform(walker_list_item[5], carla.Rotation(yaw=80))
                spectator.set_transform(spectator_transform)
                display = True

        world.wait_for_tick()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
