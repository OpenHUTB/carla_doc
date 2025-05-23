import carla
from PIL import Image
import random
import time

# 客户端连接
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world = client.get_world()

# 加载已修改的纹理
# image = Image.open('cloth.tga')  # 任意一个.tga文件即可，不需从虚幻编辑器中导出。

# 设置图片尺寸，颜色
width, height=300,400
image_r=Image.new('RGB',(width,height), 'red')
height = image_r.size[1]
width = image_r.size[0]

# 实例化 carla.TextureColor 对象并使用修改后的图像中的数据填充像素
texture_1 = carla.TextureColor(width, height)
for x in range(0, width):
    for y in range(0, height):
        color = image_r.getpixel((x,y))
        r = int(color[0])
        g = int(color[1])
        b = int(color[2])
        a = 255
        texture_1.set(x, y, carla.Color(r,g,b,a))


width, height=300,400
image_b=Image.new('RGB',(width,height), 'blue')
height = image_b.size[1]
width = image_b.size[0]

# 实例化 carla.TextureColor 对象并使用修改后的图像中的数据填充像素
texture_2 = carla.TextureColor(width, height)
for x in range(0, width):
    for y in range(0, height):
        color = image_b.getpixel((x,y))
        r = int(color[0])
        g = int(color[1])
        b = int(color[2])
        a = 255
        texture_2.set(x, y, carla.Color(r,g,b,a))

# 现在将纹理应用到建筑资产
world.apply_color_texture_to_object('BP_Apartment04_v05_Opt_2', carla.MaterialParameter.Diffuse, texture_1)



# 过滤名称中带有“公寓Apartment”的世界对象
apartment_lists = list(filter(lambda k: 'Apartment' in k, world.get_names_of_all_objects()))

# world.get_names_of_all_objects() 返回场景中可以使用应用纹理函数绘制的所有对象的名称列表
# 车辆的名字不是Vehicle，而是BP_*
# audi_vehicle = list(filter(lambda k: 'BP_AudiA2_C_0' in k, world.get_names_of_all_objects()))
# world.apply_color_texture_to_object(audi_vehicle[0], carla.MaterialParameter.Diffuse, texture)


# Get a vehicle from the library
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()

# Spawn a vehicle
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Get world spectator
spectator = world.get_spectator()

# Move the spectator behind the vehicle
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=2, z=2.5)), vehicle.get_transform().rotation)
spectator.set_transform(transform)

while True:
    try:
        lincoln_vehicle = list(filter(lambda k: 'BP_Lincoln' in k, world.get_names_of_all_objects()))
        world.apply_color_texture_to_object(lincoln_vehicle[0], carla.MaterialParameter.Diffuse, texture_1)

        time.sleep(2)

        world.apply_color_texture_to_object(lincoln_vehicle[0], carla.MaterialParameter.Diffuse, texture_2)

        time.sleep(2)

    except KeyboardInterrupt as e:

        vehicle.destroy()

        print('Vehicles Destroyed. Bye!')
        break
