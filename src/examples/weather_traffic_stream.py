import glob
import os
import sys
import time
import carla
import argparse
import logging
from numpy import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

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

def weather_traffic_stream():
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
        default=60,
        type=int,
        help='Number of vehicles (default: 30)')
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
        '-s', '--seed',
        metavar='S',
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--weather',
        metavar='W',
        default=0,
        type=int,
        help='type of weather 0 represent sun and 1 represent rain')
    argparser.add_argument(
        '--sun_altitude',
        metavar='W',
        default=150,
        type=int,
        help='sun altitude')





    try:
        args = argparser.parse_args()
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
        vehicles_list = []
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        synchronous_master = False
        random.seed(args.seed if args.seed is not None else int(time.time()))\


        world = client.get_world()
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)

        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        # 同步
        traffic_manager.set_synchronous_mode(True)
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.065
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, 'vehicle.*', 'all')

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        # blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        # 车辆数
        if (args.sun_altitude <= 70):
            args.number_of_vehicles = 155
            traffic_manager.global_percentage_speed_difference(50)
        elif(args.sun_altitude>=150):
            args.number_of_vehicles = 155
            traffic_manager.global_percentage_speed_difference(50)
        else:
            args.number_of_vehicles = 30
            traffic_manager.global_percentage_speed_difference(40)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # Spawn vehicles
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))


        # 加车灯
        vehicle_actors = world.get_actors(vehicles_list)
        for actor in vehicle_actors:
            traffic_manager.update_vehicle_lights(actor, True)



        # weather
        world = client.get_world()
        weather = Weather(world.get_weather())
        if args.weather == 0:
            weather._storm.wetness = 2
            weather.weather.wetness = weather._storm.wetness
            weather._storm.clouds = 1
            weather.weather.cloudiness = weather._storm.clouds
            weather._storm.rain = 0
            weather.weather.precipitation = weather._storm.rain
            weather._storm.fog = 0
            weather.weather.fog_density = weather._storm.fog
            weather._storm.wind = 2
            weather.weather.wind_intensity = weather._storm.wind
            weather._storm.puddles = 0
            weather.weather.precipitation_deposits = weather._storm.puddles
            weather._sun.altitude = args.sun_altitude
            weather.weather.sun_altitude_angle = weather._sun.altitude
            world.set_weather(weather.weather)

        elif(args.weather == 1):
            weather._storm.wetness = 100
            weather.weather.wetness = weather._storm.wetness
            weather._storm.clouds = 100
            weather.weather.cloudiness = weather._storm.clouds
            weather._storm.rain = 100
            weather.weather.precipitation = weather._storm.rain
            weather._storm.fog = 5
            weather.weather.fog_density = weather._storm.fog
            weather._storm.wind = 100
            weather.weather.wind_intensity = weather._storm.wind
            weather._storm.puddles = 100
            weather.weather.precipitation_deposits = weather._storm.puddles
            weather._sun.altitude = args.sun_altitude
            weather.weather.sun_altitude_angle = weather._sun.altitude
            world.set_weather(weather.weather)
            traffic_manager.global_percentage_speed_difference(60)


        spectator = world.get_spectator()
        transforms = spectator.get_transform()
        print(transforms)
        location = carla.Location(x=-146,y=17.7,z=62.6)
        rotation = carla.Rotation(pitch=-40.4,yaw=0.78,roll=0.0)
        new_transform = carla.Transform(location, rotation)
        spectator.set_transform(new_transform)

        flag = 0
        world.tick()
        while True:
            if not args.asynch and synchronous_master:
                if (flag != 1):
                    for response in client.apply_batch_sync(batch, synchronous_master):
                        if response.error:
                            logging.error(response.error)
                            flag = 1
                        else:
                            vehicles_list.append(response.actor_id)
                            flag = 1

                world.tick()
                id = world.get_actors().filter("vehicle.*")
            else:
                world.wait_for_tick()

    finally:
        return 1
        for vehicle in id:
            vehicle.destroy()

if __name__ == '__main__':
    weather_traffic_stream()