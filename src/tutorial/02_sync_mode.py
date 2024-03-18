import time

import carla
import random

# 连接到 Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# 设置仿真器到同步模式
settings = world.get_settings()
settings.synchronous_mode = True  # 启用同步模式
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 从库中获得一辆车
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# 获得生成点
spawn_points = world.get_map().get_spawn_points()

# 生成一辆车
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# 自动驾驶仪
vehicle.set_autopilot(True) 

# 获得世界观察者
spectator = world.get_spectator()

# 没有这个循环，观察者不会跟随车辆
while True:
    try:
        # 在车辆后移动车辆
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
        spectator.set_transform(transform) 
        time.sleep(0.005)

        world.tick()

    except KeyboardInterrupt as e:
        vehicle.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False  # 禁用异步模式
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('Vehicles Destroyed. Bye!')
        break
