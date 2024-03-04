import carla
import argparse
import time
def cruise():
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
    argparser.add_argument(
        '-time',
        metavar='T',
        default=0.02,
        type=float,
        help='delay time(default: 0.02)')
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = 0.1 / speed_factor


    client = carla.Client(args.host, args.port)
    world = client.get_world()

    map1_name = "map_package/Maps/HutbCarlaCity/HutbCarlaCity"
    map2_name = "Carla/Maps/Town10HD_Opt"
    if world.get_map().name == map1_name:
        spectator = world.get_spectator()
        location = carla.Location(x=400, y=-15, z=50)
        rotation = carla.Rotation(pitch=0, yaw=180, roll=0)
        new_transform = carla.Transform(location, rotation)
        spectator.set_transform(new_transform)
        f=2/3
        t=0
        while True:
            time.sleep(args.time)
            t=t+1
            if t <= 280/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-2*f, y=0.06*f, z=0)
                rotation = carla.Rotation(pitch=0, yaw=180, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 135 + 280/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-2/3, y=-1/6, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw + 4/5, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 135 + 280/f + 150/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=0.7*f, y=-2*f, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 270 + 280/f + 150/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=1/6, y=-1/3, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw + 1/2, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 270 + 280/f + 150/f + 240/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=2*f, y=-0.05*f, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 405 + 280/f + 150/f + 240/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=1/2, y=1/6, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw + 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 405 + 280/f + 150/f + 240/f + 135/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=0.05*f, y=2*f, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 540 + 280/f + 150/f + 240/f + 135/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-1/6, y=1/2, z=0)
                rotation = carla.Rotation(pitch=0, yaw=rotation.yaw + 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            else:
                break

    if world.get_map().name == map2_name:
        # spectator = world.get_spectator()
        # location = carla.Location(x=-180, y=220, z=50)
        # rotation = carla.Rotation(pitch=-10, yaw=-45, roll=0)
        # new_transform = carla.Transform(location, rotation)
        # spectator.set_transform(new_transform)
        spectator = world.get_spectator()
        location = carla.Location(x=-100, y=135, z=40)
        rotation = carla.Rotation(pitch=-35, yaw=0, roll=0)
        new_transform = carla.Transform(location, rotation)
        spectator.set_transform(new_transform)
        f=1/5
        t = 0
        while True:
            time.sleep(args.time)
            t = t + 1
            if t <= 80/f:#80
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=2*f, y=0, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=0, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 135+80/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=1/3, y=-1/6, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=rotation.yaw - 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 135+80/f+68/f:#65
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=0, y=-2*f, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=-90, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 270+80/f+68/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-1/6, y=-1/3, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=rotation.yaw - 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 270+80/f+68/f+72/f:#75
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-2*f, y=0, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=-180.0, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 405+80/f+68/f+72/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=-1/3, y=1/6, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=rotation.yaw - 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 405+80/f+68/f+72/f+65/f:#65
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=0, y=2*f, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=-270, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            elif t <= 540+80/f+68/f+72/f+65/f:
                transform = spectator.get_transform()
                location = transform.location + carla.Location(x=1/6, y=1/3, z=0)
                rotation = carla.Rotation(pitch=-35, yaw=rotation.yaw - 2/3, roll=0)
                new_transform = carla.Transform(location, rotation)
                spectator.set_transform(new_transform)
            else :
                break

if __name__ == '__main__':
    cruise()