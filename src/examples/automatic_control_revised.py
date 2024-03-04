# -*- coding: utf-8 -*-
import carla
import time
import argparse

spectator_obj_list = []
from agents.navigation.behavior_agent import BehaviorAgent

diming = {
    "北门": {"x": 280.2, "y": 1.7, "z": 1},
    "西门": {"x": -105, "y": -155, "z": 10},
    "楚枫桥": {"x": 1.8, "y": -15.9, "z": 30},
    "楚枫轩": {"x": 14.8, "y": 64.7, "z": 20},
    "日新楼": {"x": 58, "y": -75, "z": 30},
    "体育馆": {"x": 160, "y": -265, "z": 40},
    "湘江楼": {"x": -2.4, "y": -100, "z": 30},
    "科技楼": {"x": -97.4, "y": -217, "z": 15},
}

def main( ):
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--s',
        default="北门",
        type=str)
    argparser.add_argument(
        '--e',
        default="西门",
        type=str)
    args = argparser.parse_args()
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10)

        # Retrieve the world that is currently running
        world = client.get_world()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # read all valid spawn points
        all_default_spawn = world.get_map().get_spawn_points()
        # randomly choose one as the start point
        #spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()
        locations = {key: carla.Location(**value) for key, value in diming.items()}
        # spawn_location = carla.Location(x=92.326691, y= -49.225342,z =1)  # 修改为您想要的位置
        spawn_location = carla.Location(locations[args.s])
        spawn_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)  # 修改为您想要的朝向
        spawn_point = carla.Transform(spawn_location, spawn_rotation)

        # create the blueprint library
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.tt')
        ego_vehicle_bp.set_attribute('color', '100, 250, 250')


        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        agent = BehaviorAgent(vehicle, behavior='normal')
        destination = carla.Location(locations[args.e])

        # generate the route
        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)

        # 获得车辆的中心点，并将点定位到右上角，便于运算
        bound_x = 0.5 + vehicle.bounding_box.extent.x
        bound_y = 0.5 + vehicle.bounding_box.extent.y
        bound_z = 0.5 + vehicle.bounding_box.extent.z

        # 查找相机蓝图
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

        # 设置Camera的附加类型Camera跟随车辆(幽灵模式)；
        Atment_SpringArmGhost = carla.libcarla.AttachmentType.SpringArmGhost

        Atment_Rigid = carla.libcarla.AttachmentType.Rigid

        # 设置相对车辆的安装位置，配置上帝视图(Camera无法实现上帝视图，画面会抖动)
        Vehicle_transform_list = [
            (carla.Location(z=35), carla.Rotation(pitch=-90))
        ]

        # 设置camera的安装位置，配置后往前视图以及前后左右视图
        Camera_transform_list = [
            (carla.Transform(carla.Location(x=-8, y=0, z=5),
                             carla.Rotation(pitch=15, yaw=0, roll=0)), Atment_SpringArmGhost)
        ]

        # 拼接两个transform_list
        spectator_transform_list = Vehicle_transform_list + Camera_transform_list

        # 上帝视图坐标系以及所有camera对象填入spectator_obj_list；
        for spectator_transform_index in spectator_transform_list:

            # spectator_transform_list第0个元素为上帝视图坐标系
            if spectator_transform_list.index(spectator_transform_index) == 0:
                spectator_obj_list.append(spectator_transform_index)

            # spectator_transform_list其余元素为Camera安装参数，下面生成Camera对象
            else:
                camera = world.spawn_actor(camera_bp, spectator_transform_index[0],
                                           attach_to=vehicle, attachment_type=spectator_transform_index[1])
                spectator_obj_list.append(camera)

        # 设置Vehicle_transform_list[0]为初始视图(上帝视图)；
        spectator_obj = Vehicle_transform_list[0]
        c=1
        while True:
            agent.update_information(vehicle)

            world.tick()
            if len(agent._local_planner.waypoints_queue)<1:
                time.sleep(2)
                print('======== Success=============')
                break

            #top view
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            if c:
                # 上一个spectator的索引号；
                last_spectator_obj_index = spectator_obj_list.index(spectator_obj)
                # 计算下一个spectator的索引，如果列表索引超限则重新拿第0个spectator；
                spectator_obj_index = last_spectator_obj_index + 1 if len(
                    spectator_obj_list) - last_spectator_obj_index - 1 > 0 else 0
                spectator_obj = spectator_obj_list[spectator_obj_index]
                time.sleep(0.2)

                # 更新视图
            if spectator_obj_list.index(spectator_obj) == 0:
                # 设置上帝视图
                Vehicle_transform = carla.Transform(vehicle.get_transform().location + spectator_obj_list[0][0],
                                                    spectator_obj_list[0][1])
                world.get_spectator().set_transform(Vehicle_transform)
            else:
                # 设置其他Camera视图
                world.get_spectator().set_transform(spectator_obj.get_transform())
            c=0

            #speed_limit = vehicle.get_speed_limit()
            #agent.get_local_planner().set_speed(50)
            control = agent.run_step(debug=True)
            vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
