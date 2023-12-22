import random

import carla

# 指定端口启动Carla服务：
client = carla.Client('localhost', 2000)
world = client.get_world()

blueprint_library = world.get_blueprint_library()

# 找一个指定的蓝图
collision_sensor_bp = blueprint_library.find('sensor.other.collision')
# 随机选择一个车辆蓝图
vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))


# is_bike = [vehicle_bp.get_attribute('number_of_wheels') == 2]
# if(is_bike):
#     vehicle_bp.set_attribute('color', '255,0,0')

# for attr in blueprint:
#     if attr.is_modifiable:
#         blueprint.set_attribute(attr.id, random.choice(attr.recommended_values))


# Carla 的位置和旋转点
spawn_point = carla.Transform()
# 从导航文件中获得行人的随机生成点
spawn_point.location = world.get_random_location_from_navigation()

pass

