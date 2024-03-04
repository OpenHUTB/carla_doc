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

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random



def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter) # 从carla仿真世界蓝图库中筛选蓝图

    if generation.lower() == "all":
        return bps                                     # 检查字符串是否==‘all’,是则返回所有符合得蓝图

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:                                   #蓝图数=1，返回这个蓝图
        return bps

    try:
        int_generation = int(generation)                # 将generation转换为整数类型，检查是否为有效的生成代数

        if int_generation in [1, 2]:                    # 检查生成代数在表的范围内
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
        description='CARLA Manual Control Client')
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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
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
        action='store_true',            #False
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
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)   # 多客户端模式和多交通管理器模式
    client.set_timeout(2000.0)                    # 设置超时
    world = client.get_world()                    # 获取世界对象
    synchronous_master = True
    vehicles_list = []

    try:

        ##  交通灯
        # 获取所有红绿灯
        traffic_lights = world.get_actors().filter('traffic.traffic_light')
        # 打印红绿灯的数量
        # print("Number of Traffic Lights: {}".format(len(traffic_lights)))
        # 遍历红绿灯,查看哪几个红绿灯是一组的
        # for traffic_light in traffic_lights:
        #    light_group = traffic_light.get_group_traffic_lights()
        #   for traffic_light in light_group:
        #        print("Traffic Light ID: {}".format(traffic_light.id))
        #   print("*************************")

        # 设置交通灯持续时间,只能修改一个组的，另外获取某一个红绿灯有问题,得得到lightID
        for traffic_light in traffic_lights:
            traffic_light.set_green_time(200)
            #traffic_light.set_yellow_time(200)
            # 获取交通灯持续时间
            # for traffic_light in traffic_lights:
            #     print("Traffic Light ID: {}".format(traffic_light.get_red_time()))
            #     print("Traffic Light ID: {}".format(traffic_light.get_green_time()))
            #     print("Traffic Light ID: {}".format(traffic_light.get_yellow_time()))
            # traffic_light.set_state（self， state)



            # 设置客户端同步
        if not args.sync:
            settings = world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        # 生成车辆
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']
        blueprints = sorted(blueprints, key=lambda bp: bp.id)  # 对蓝图按id升序排序
        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        hero = args.hero  # False
        batch = []
        blueprint = random.choice(blueprints)  # 从可用的车辆蓝图中随机选择一个来生成车辆。
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if hero:
            blueprint.set_attribute('role_name', 'hero')
            hero = False
        else:
            blueprint.set_attribute('role_name', 'autopilot')

        spawn_points = world.get_map().get_spawn_points()  # 获取生成点
        # 设置生成点坐标
        #transform = carla.Transform(carla.Location(x=-60.64484405517578,y=24.471010208129883,z=0.5999999642372131),carla.Rotation())
        transform = carla.Transform(spawn_points[0].location, carla.Rotation())
        # 设置路径
        route_indices = [93, 53, 56, 119, 59, 76, 136, 38, 39, 118]
        route = []

        for transform_loc_index in route_indices:
            route.append(spawn_points[transform_loc_index].location)

        for ind in route_indices:
            spawn_points[ind].location
            world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(255, 0, 0))

        # 生成车辆

        #vehicle = world.spawn_actor(blueprint , transform)
          # batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
          # # response=client.apply_batch_sync(batch, synchronous_master)
          # for response in client.apply_batch_sync(batch, synchronous_master):
          # if response.error:
          #       logging.error(response.error)
          # else:
          #    #将生成的车辆的 actor ID（response.actor_id）添加到 vehicles_list 列表中
          #    vehicles_list.append(response.actor_id)
        vehicle1 = world.try_spawn_actor(blueprint, carla.Transform(carla.Location(x=74.64484405517578,y=24.471010208129883,z=0.5999999642372131),carla.Rotation()))
        vehicle = world.try_spawn_actor(blueprint, transform)
        route = [
            carla.Location(x=-52.64484405517578, y=24.471010208129883, z=0.5999999642372131),
            carla.Location(x=-42.64484405517578, y=24.471010208129883, z=0.5999999642372131),
            carla.Location(x=-34.64484405517578, y=24.471010208129883, z=0.5999999642372131),
            carla.Location(x=-21.64484405517578, y=24.471010208129883, z=0.5999999642372131),
            carla.Location(x=-11.64484405517578, y=24.471010208129883, z=0.5999999642372131),
            carla.Location(x=-0.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=10.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=20.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=25.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=30.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=35.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=40.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=45.64484405517578, y=30.471010208129883, z=0.5999999642372131),
            carla.Location(x=50.64484405517578, y=30.471010208129883, z=0.5999999642372131),

        ]

        if vehicle is not None:
            vehicle.set_autopilot(True)
            traffic_manager.update_vehicle_lights(vehicle, True)
            # traffic_manager.random_left_lanechange_percentage(vehicle, 0)
            # traffic_manager.random_right_lanechange_percentage(vehicle, 0)
            traffic_manager.auto_lane_change(vehicle, False)
            # 定义车辆路线
            #route_instructions = ['Straight', 'Straight', 'Straight','Straight','Straight', 'Straight']

            # 使用 Traffic Manager 设置车辆路线
            #traffic_manager.set_route(vehicle, route_instructions)
            traffic_manager.set_path(vehicle, route)


        # 获取所有actors
        all_actors = world.get_actors()
        # 过滤得到所有车辆
        vehicles = [actor for actor in all_actors if actor.type_id.startswith('vehicle')]
        # 获取车辆数量
        num_vehicles = len(vehicles)
        # 打印车辆数量
        print("Number of vehicles: {}".format(num_vehicles))


        i=0
        while True:

            if not args.asynch and synchronous_master:
              #  if   i == 1000:
              #       client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
              #vehicle.destroy()
                world.tick()
            else:
                world.wait_for_tick()
            i+=1

    finally:

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)



        time.sleep(0.5)

if __name__ == '__main__':

    main()
