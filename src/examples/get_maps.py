    # 获取所有地图，并根据地图名进行切换
import carla
import time
import argparse
import logging
from numpy import random

    # 指定端口启动Carla服务：c
    # CarlaUE4.exe -carla-rpc-port=3000

def main():
    print("yes")
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
         '--mapname',
        type=str,
        default='Town04',
        help='name of maps')
    args = argparser.parse_args()
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    print(client.get_available_maps())  # 获取所有可使用地图名
    world = client.get_world()
    print(world.get_map())  # 获取当前使用地图名
    # ['/Game/Carla/Maps/Town10HD_Opt', '/Game/RoadRunner/Maps/ScenarioBasic']
    # 注意：客户端默认加载使用Town10HD_Opt地图，Opt后缀表示可分层(Optional)地图。

    # world = client.load_world('Town10HD')  # 高精度地图城市
    # CarlaUE4.exe /Game/Carla/Maps/Town07
    # world = client.load_world('Town07')
    world = client.load_world(args.mapname)
    print(world.get_map().name)

    #
    if world.get_map().name == 'Carla/Maps/Town10HD_Opt':
        time.sleep(0.06)
        spectator = world.get_spectator()
        location = carla.Location(x=-180, y=220, z=50)
        rotation = carla.Rotation(pitch=-10, yaw=-45, roll=0)
        new_transform = carla.Transform(location, rotation)
        spectator.set_transform(new_transform)
    
    # 获得世界中的建筑
    world = client.get_world()
    env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)
    env_objs_ = world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
    env_objs_ = world.get_environment_objects(carla.CityObjectLabel.TrafficLight)
    # 访问各个建筑 IDs 并保存在集合当中
    building_01 = env_objs[0]
    building_02 = env_objs[1]
    objects_to_toggle = {building_01.id, building_02.id}

    # 切换建筑为不可见
    # world.enable_environment_objects(objects_to_toggle, False)
    # 切换建筑为可见
    # world.enable_environment_objects(objects_to_toggle, True)
    buildinglength = len(env_objs)
    print(buildinglength)

    ## 显示一半的建筑
    # half_building_ids = set(map(lambda build : build.id, env_objs[buildinglength/2:buildinglength]))
    # half_buildings = set(env_objs[0:buildinglength/2])
    # world.enable_environment_objects(half_building_ids, False)
    # world.enable_environment_objects(half_building_ids, True)

    # 隐藏一半的建筑
    half_building_ids_list = list(map(lambda build: build.id, env_objs[26:buildinglength]))
    half_building_ids = set(map(lambda build: build.id, env_objs[26:buildinglength]))
    world.enable_environment_objects(half_building_ids, False)
    #world.unload_map_layer(carla.MapLayer.All) # 隐藏所有层

if __name__ == '__main__':
    main()


