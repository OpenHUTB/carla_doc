"""
 add vehicles， but it doesn't move
"""


import glob
import os
import sys
import time
import carla

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


def clean_vehicles(world):
    all_vehicles = world.get_actors().filter('vehicle.*')

    for vehicle in all_vehicles:
        vehicle.destroy()


def main():
    client = carla.Client('127.0.0.1', 2000)  # 多客户端模式和多交通管理器模式
    client.set_timeout(10)  # 设置超时
    world = client.get_world()  # 获取世界对象

    spectator = world.get_spectator()
    location = carla.Location(x=13.299536,y=2.904287,z=30.622684)
    rotation = carla.Rotation(pitch=-25,yaw=180,roll=0)
    new_transform = carla.Transform(location,rotation)
    spectator.set_transform(new_transform)

    clean_vehicles(world)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01
        world.apply_settings(settings)

        vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')

        # 左边静止的车辆
        static_location = carla.Location(x=-37.12, y=13.92, z=0.12)
        static_rotation = carla.Rotation(yaw=0)
        static_transform = carla.Transform(static_location, static_rotation)
        sta_vehicle_instance = world.try_spawn_actor(vehicle_bp, static_transform)

        # 左车道运动的车辆
        left_vehicles_info = [
            [1, -70.05, 7.73, -8.50, 0.43],
            [2, -140.05, 7.73, -8.50, 0.43],
            [3, -28.05, 4.73, -8.50, 0.43],
            [4, -250, 13.8, -8.50, 0.43],
            [5, -260, 18, -8.50, 0.43]
        ]
        key_vehicle = None
        for item in left_vehicles_info:
            location = carla.Location(x=float(item[1]), y=float(item[2]), z=0.12)
            print(location)
            rotation = carla.Rotation(yaw=0)
            transform = carla.Transform(location, rotation)
            vehicle_instance = world.try_spawn_actor(vehicle_bp, transform)
            if item[0] == 2:
                key_vehicle = vehicle_instance
            if item[0] == 1 or item[0] == 2 or item[0] == 3:
                vehicle_instance.set_target_velocity(carla.Vector3D(x=12, y=0, z=0))
                controller = carla.VehicleControl(steer=-0.001)
            else:
                vehicle_instance.set_target_velocity(carla.Vector3D(x=12, y=0, z=0))
                controller = carla.VehicleControl(steer=-0.001)
            vehicle_instance.apply_control(controller)
            world.tick()

        flag = True
        # 左车道运动的车辆
        right_vehicles_info = [
            [1, -10, -3.5, -8.50, 0.43],
            [2, 0, -4.8, -8.50, 0.43],
            [3, 120, -7, -8.50, 0.43]
        ]
        while flag:
            if key_vehicle.get_location().x > -80:
                # key_vehicle.set_target_velocity(carla.Vector3D(x=0, y=0, z=0))
                flag = False
            world.tick()

        for item in right_vehicles_info:
            location = carla.Location(x=float(item[1]), y=float(item[2]), z=0.12)
            print(location)
            rotation = carla.Rotation(yaw=180)
            transform = carla.Transform(location, rotation)
            vehicle_instance = world.try_spawn_actor(vehicle_bp, transform)
            if item[0] == 1:
                vehicle_instance.set_target_velocity(carla.Vector3D(x=-12, y=-1, z=0))
                controller = carla.VehicleControl(steer=-0.0005)
            elif item[0] == 2:
                biased_vehicle = vehicle_instance
                vehicle_instance.set_target_velocity(carla.Vector3D(x=-11.5, y=-1, z=0))
                controller = carla.VehicleControl(steer=0.0005)
            elif item[0] == 3:
                righted_vehicle = vehicle_instance
                vehicle_instance.set_target_velocity(carla.Vector3D(x=-20, y=-1, z=0))
                controller = carla.VehicleControl(steer=0.00005)
            vehicle_instance.apply_control(controller)
            world.tick()

        flag1 = True
        while flag1:
            if biased_vehicle.get_location().x < -150:
                controller = carla.VehicleControl(steer=-0.0018)
                biased_vehicle.apply_control(controller)
                righted_vehicle.set_target_velocity(carla.Vector3D(x=-10, y=0, z=0))
                flag1 = False
            world.tick()

        flag2 = True
        while flag2:
            if righted_vehicle.get_location().x < -200:
                controller = carla.VehicleControl(steer=0.09)
                righted_vehicle.apply_control(controller)
                flag2 = False
            world.tick()

    finally:
        # 最后修改为异步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


if __name__ == '__main__':
    main()
    # result = get_data_by_frame_num(50913);
    # print(result)
