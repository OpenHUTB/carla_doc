import carla

import queue
import random

import cv2
import numpy as np

from utils.projection import *

# 第一部分

client = carla.Client('localhost', 2000)
world = client.get_world()

# 在同步模式下配置仿真器
settings = world.get_settings()
settings.synchronous_mode = True  # 启用同步模式
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 获得世界的观察者
spectator = world.get_spectator() 

# 获得地图的生成点
spawn_points = world.get_map().get_spawn_points()

# 生成车辆
bp_lib = world.get_blueprint_library()
vehicle_bp =bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# 创建用于存储和获取传感器数据的队列
image_queue = queue.Queue()
camera.listen(image_queue.put)


# 第二部分

# 记住边对
edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, ]]

# 获得世界坐标系转相机坐标系的矩阵
world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

# 从相机中获得属性
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()

# 计算三维到二维的相机投影矩阵
K = build_projection_matrix(image_w, image_h, fov)
K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

for i in range(20):
    vehicle_bp = bp_lib.filter('vehicle')

    # 排除自行车
    car_bp = [bp for bp in vehicle_bp if int(bp.get_attribute('number_of_wheels')) == 4]
    npc = world.try_spawn_actor(random.choice(car_bp), random.choice(spawn_points))

    if npc:
        npc.set_autopilot(True)

# 获取关卡的所有目标
car_objects = world.get_environment_objects(carla.CityObjectLabel.Car)  # 没有根据类型进行过滤
truck_objects = world.get_environment_objects(carla.CityObjectLabel.Truck)  # 没有根据类型进行过滤
bus_objects = world.get_environment_objects(carla.CityObjectLabel.Bus)  # 没有根据类型进行过滤

env_object_ids = []

for obj in (car_objects + truck_objects + bus_objects):
    env_object_ids.append(obj.id)

# 禁用所有静态车辆
world.enable_environment_objects(env_object_ids, False) 


def clear():
    settings = world.get_settings()
    settings.synchronous_mode = False  # 禁用同步模式
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

    camera.stop()

    for npc in world.get_actors().filter('*vehicle*'):
        if npc:
            npc.destroy()

    print("Vehicles Destroyed.")


# 主循环
vehicle.set_autopilot(True)

while True:
    try:
        world.tick()

        # 将观察者移动到车辆上方
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # 获取并重塑图像
        image = image_queue.get()

        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

        # 获取图像矩阵
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

        for npc in world.get_actors().filter('*vehicle*'):

            # 过滤出自主车辆
            if npc.is_alive and npc.id != vehicle.id:

                npc_transform = npc.transform if isinstance(npc, carla.EnvironmentObject) else npc.get_transform()

                bb = npc.bounding_box
                dist = npc_transform.location.distance(vehicle.get_transform().location)

                # 过滤出 50 米以内的车辆
                if dist < 50:
                    # Calculate the dot product between the forward vector
                    # of the vehicle and the vector between the vehicle
                    # and the other vehicle. We threshold this dot product
                    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                    vehicle_forward_vec = vehicle.get_transform().get_forward_vector()
                    ray = npc_transform.location - vehicle.get_transform().location

                    if vehicle_forward_vec.dot(ray) > 0:
                        verts = [v for v in bb.get_world_vertices(npc_transform)]

                        points_image = []

                        for vert in verts:
                            ray0 = vert - camera.get_transform().location
                            cam_forward_vec = camera.get_transform().get_forward_vector()

                            if (cam_forward_vec.dot(ray0) > 0):
                                p = get_image_point(vert, K, world_2_camera)
                            else:
                                p = get_image_point(vert, K_b, world_2_camera)

                            points_image.append(p)

                        for edge in edges:
                            p1 = points_image[edge[0]]
                            p2 = points_image[edge[1]]

                            p1_in_canvas = point_in_canvas(p1, image_h, image_w)
                            p2_in_canvas = point_in_canvas(p2, image_h, image_w)

                            if not p1_in_canvas and not p2_in_canvas:
                                continue

                            cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        

        cv2.imshow('3D Bounding Boxes',img)

        if cv2.waitKey(1) == ord('q'):
            clear()
            break

    except KeyboardInterrupt as e:
        clear()
        break

cv2.destroyAllWindows()
