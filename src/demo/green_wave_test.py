import random

import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # 设置同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    #获取所有交通灯
    traffic_lights = world.get_actors().filter('traffic.traffic_light')

    # 设置交通灯
    light2 = traffic_lights[13]
    light3 = traffic_lights[9]

    print(light2.get_green_time())
    print(light2.get_yellow_time())
    print(light2.get_red_time())

    # 设置红绿灯时间
    light2.set_green_time(50)
    light2.set_yellow_time(30)
    light2.set_red_time(20)

    print(light2.get_green_time())
    print(light2.get_yellow_time())
    print(light2.get_red_time())

    while True:
        world.tick()
        time.sleep(0.05)

if __name__ == '__main__':
    main()