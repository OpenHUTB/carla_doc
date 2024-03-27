# 获取所有地图，并根据地图名进行切换
import carla

import time

# 指定端口启动Carla服务：
# CarlaUE4.exe -carla-rpc-port=3000
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print(client.get_available_maps())  # 获取所有可使用地图名

world = client.get_world()
print(world.get_map())  # 获取当前使用地图名


# 获得世界中的建筑
world = client.get_world()
env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

# 访问各个建筑 IDs 并保存在集合当中
building_01 = env_objs[0]
building_02 = env_objs[1]
objects_to_toggle = {building_01.id, building_02.id}

# 切换建筑为不可见
# world.enable_environment_objects(objects_to_toggle, False)
# 切换建筑为可见
# world.enable_environment_objects(objects_to_toggle, True)



## 显示一半的建筑
half_building_ids = set(map(lambda build : build.id, env_objs[500:1087]))
#half_buildings = set(env_objs[0:500])
world.enable_environment_objects(half_building_ids, False)
world.enable_environment_objects(half_building_ids, True)


## 逐步显示建筑
pass
# 隐藏所有建筑
all_building_ids_list = list(map(lambda build : build.id, env_objs))
all_building_ids = set(map(lambda build : build.id, env_objs))
world.enable_environment_objects(all_building_ids, False)

for i in range(len(all_building_ids_list)):
    world.enable_environment_objects({all_building_ids_list[i]}, True)
    time.sleep(0.01)
