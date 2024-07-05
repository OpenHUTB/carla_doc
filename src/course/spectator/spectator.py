"""
  切换湖工商附近4个路口视角
"""
import time
import carla
import argparse


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    args = argparser.parse_args()
    # 连接到carla服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(10)
    world = client.get_world()
    try:
        # 设置
        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.05
        # world.apply_settings(settings)
        spectator_transform = [
            carla.Transform(carla.Location(x=-323, y=-15.7, z=108), carla.Rotation(pitch=-90)),
            carla.Transform(carla.Location(x=-359.6, y=391.3, z=94), carla.Rotation(pitch=-90)),
            carla.Transform(carla.Location(x=426.2, y=-33.4, z=91.6), carla.Rotation(pitch=-90)),
            carla.Transform(carla.Location(x=375.1, y=-558.4, z=91.6), carla.Rotation(pitch=-90))
        ]
        # 获取spectator对象
        spectator = world.get_spectator()
        spectator.set_transform(spectator_transform[0])

        start_time = time.time()
        state = 0
        while True:
            # 因为先运行generate_traffic.py，所以刷新操作由它完成即可
            # world.tick()
            elapsed_time = time.time() - start_time
            if elapsed_time >= 5:
                spectator.set_transform(spectator_transform[state])
                state += 1
                if state > 3:
                    state = 0
                start_time = time.time()
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
