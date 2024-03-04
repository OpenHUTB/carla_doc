import carla
import time
import random
import argparse
import math
def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def spawn_vehicles(client, world, number_of_vehicles, blueprints, traffic_manager, synchronous_master,actor_list):
    vehicles_list = []
    spawn_points = world.get_map().get_spawn_points()
    if number_of_vehicles < len(spawn_points):
        random.shuffle(spawn_points)
    elif number_of_vehicles > len(spawn_points):
        print(f"请求了 {number_of_vehicles} 辆车，但只能找到 {len(spawn_points)} 个生成点")
        number_of_vehicles = len(spawn_points)

    # 生成车辆
    batch = []
    hero = True
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
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

        # 生成车辆并设置自动驾驶
        batch.append(
            carla.command.SpawnActor(blueprint, transform)
            .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port()))
        )

    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            print(response.error)
        else:
            vehicles_list.append(response.actor_id)
            actor_list.append(response.actor_id)

    return vehicles_list

def interpolate_weather(current_weather, target_weather, alpha):
    # 插值函数，用于平滑地从当前天气到目标天气
    interpolated_weather = carla.WeatherParameters()
    interpolated_weather.sun_altitude_angle = current_weather.sun_altitude_angle + alpha * (
                target_weather.sun_altitude_angle - current_weather.sun_altitude_angle)
    return interpolated_weather
def smooth_change_in_weather(world,current_weather, target_weather, steps=100):
    # 平滑地改变天气
    for step in range(steps):
        alpha = step / float(steps - 1)
        interpolated_weather = interpolate_weather(current_weather, target_weather, alpha)
        world.set_weather(interpolated_weather)
        #time.sleep(0.03)  # 等待一秒，可以调整等待时间
def destroy_vehicles(client,num_vehicles,vehicles_list):
    vehicles_to_remove = random.sample(vehicles_list, min(num_vehicles, len(vehicles_list)))
    print("qqqqqqqqqqqqqqqqqqqqqqqqqqqq",vehicles_to_remove)
    # 使用Carla命令删除选定的车辆
    client.apply_batch([carla.command.DestroyActor(actor_id) for actor_id in vehicles_to_remove])
    vehicles_list = [vehicle_id for vehicle_id in vehicles_list if vehicle_id not in vehicles_to_remove]

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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=9,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=4,
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

    args = argparser.parse_args()


    try:
        # ... (other argument parsing code)

        # Connect to CARLA server
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))

        # Get the world and traffic manager
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        # Get actor blueprints
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        # Spawn vehicles
        # 在 main 函数中
        # 获取观察者对象
        spectator = world.get_spectator()
        # 根据观察者默认方位设置新的方位
        location = carla.Location(x=-79.805443, y=176.765305, z=78.248627)
        rotation = carla.Rotation(pitch=-38.305176, yaw=-63.620777, roll=0.000002)
        new_transform = carla.Transform(location, rotation)
        # 将观察者设置到新方位上
        spectator.set_transform(new_transform)
        actor_list = []
        is_change = True
        hour=6
        is_add=True
        is_step=True
        step=180
        count_step=0
        while True:
            if step>-45:
                step = step - 0.4
                count_step=count_step+0.4
            if 6 <= hour <=8 and is_add:
                # 早上时段，生成一些车辆
                spawn_vehicles(client, world, 300, blueprints, traffic_manager,
                               synchronous_master,actor_list)
                traffic_manager.global_percentage_speed_difference(-30)
                is_add = False
            elif 11 <= hour <=16 and is_add:
                # 中午时段，删除一些车辆
                destroy_vehicles(client, 50, actor_list)
                traffic_manager.global_percentage_speed_difference(-30)
                is_add = False
            if hour > 16 and is_add:
                if True:
                    all_vehicle_actors = world.get_actors(actor_list)
                    for actor in all_vehicle_actors:
                        traffic_manager.update_vehicle_lights(actor, True)
                # 晚上时段，生成一些车辆
                spawn_vehicles(client, world, 200, blueprints, traffic_manager,
                               synchronous_master,actor_list)
                traffic_manager.global_percentage_speed_difference(-30)
                is_add = False
            if hour >=18:
                is_step=False
            if hour >=20 and is_add:
                is_add=False
            print(f"角度: {step:02}:00")
            print(f"时间: {hour:02}:00")
            # 获取目标天气
            if is_step:
              target_weather = carla.WeatherParameters(sun_altitude_angle=step)
              world.set_weather(target_weather)
            # 获取当前天气
            current_weather = world.get_weather()

            if count_step>15 and hour<21:
                hour=hour+1
                count_step=0
                is_add = True
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])


        time.sleep(0.5)


# ... (cleanup code)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
