import carla
import random
import numpy as np
import math
import queue
import cv2 #OpenCV to manipulate and save the images

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# We will aslo set up the spectator so we can see what we do
spectator = world.get_spectator()


def center_camera(ped, rot_offset=0):
    # 旋转相机以面向行人并应用偏移
    trans = ped.get_transform()
    offset_radians = 2 * math.pi * rot_offset/360
    x = math.cos(offset_radians) * -2
    y = math.sin(offset_radians) * 2
    trans.location.x += x
    trans.location.y += y
    trans.location.z = 2
    trans.rotation.pitch = -16
    trans.rotation.roll = 0
    trans.rotation.yaw = -rot_offset
    spectator.set_transform(trans)
    return trans


# 获取行人蓝图并生成它
pedestrian_bp = random.choice(world.get_blueprint_library().filter('*walker.pedestrian*'))
transform = carla.Transform(carla.Location(x=-134, y=78.1, z=1.18))
pedestrian = world.try_spawn_actor(pedestrian_bp, transform)

# 生成一个 RGB 相机
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, transform)

# 创建一个队列来存储并获取传感器数据
image_queue = queue.Queue()
camera.listen(image_queue.put)

world.tick()
image_queue.get()
# 每次调用 world.tick() 时，我们必须调用 image_queue.get() 以确保时间步长和传感器数据保持同步

# 现在，我们将旋转相机以面向行人
camera.set_transform(center_camera(pedestrian))
# 移动观察者以查看结果
spectator.set_transform(camera.get_transform())

# 为行人设置AI控制器
controller_bp = world.get_blueprint_library().find('controller.ai.walker')
controller = world.spawn_actor(controller_bp, pedestrian.get_transform(), pedestrian)

# 启动控制器并给它一个随机位置
controller.start()
controller.go_to_location(world.get_random_location_from_navigation())

# 将世界移动几帧，让行人生成
for frame in range(0, 5):
    world.tick()
    trash = image_queue.get()


controller_bp = world.get_blueprint_library().find('controller.ai.walker')
controller = world.spawn_actor(controller_bp, pedestrian.get_transform(), pedestrian)
controller.start()
controller.go_to_location(world.get_random_location_from_navigation())


