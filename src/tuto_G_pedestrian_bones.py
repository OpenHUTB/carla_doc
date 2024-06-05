import carla
import random
import numpy as np
import math
import queue
import cv2  # 使用 OpenCV 操作并保存图像

import os

# 连接客户端并获取世界对象
client = carla.Client('localhost', 2000)
world = client.get_world()

# 以同步模式设立仿真器
settings = world.get_settings()
settings.synchronous_mode = True  # 启用同步模式
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 我们也会设置观察者以便我们能看到我们所做的
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
transform = carla.Transform(carla.Location(x=-134,y=78.1,z=1.18))
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


# 获取4x4矩阵以将点从世界坐标转换为相机坐标
world_2_camera = np.array(camera.get_transform().get_inverse_matrix())


def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(bone_trans):
        # 计算骨骼坐标的二维投影
        
        # 获取骨根的世界位置
        loc = bone_trans.world.location
        bone = np.array([loc.x, loc.y, loc.z, 1])
        # 转换为相机坐标
        point_camera = np.dot(world_2_camera, bone)

        #  我们必须从UE4的坐标系更改为“标准”坐标系
        # (x, y ,z) -> (y, -z, x)
        # 我们还删除了第四个组件
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # 现在使用摄影机矩阵投影3D->2D
        point_img = np.dot(K, point_camera)
        # 正则化
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]
    

def build_skeleton(ped, sk_links, K):
    ######## 获取行人骨架 #########
    bones = ped.get_bones()

    # 列出将投影到相机输出上行的存储位置
    lines = []

    # 在 skeleton.txt 中遍历骨骼对并获取关节位置
    for link in sk_links[1:]:
        # 将两块骨头的根部连接起来
        bone_transform_1 = next(filter(lambda b: b.name == link[0], bones.bone_transforms), None)
        bone_transform_2 = next(filter(lambda b: b.name == link[1], bones.bone_transforms), None)

        # 某些骨骼名称不匹配
        if bone_transform_1 is not None and bone_transform_2 is not None:
            # 计算三维骨骼坐标的二维投影
            point_image = get_image_point(bone_transform_1)
    
            # 将行开始附加到行列表
            lines.append([point_image[0], point_image[1], 0, 0])
            
            # 计算三维骨骼坐标的二维投影
            point_image = get_image_point(bone_transform_2)
            
            # 将行尾附加到行列表
            lines[-1][2] = point_image[0]
            lines[-1][3] = point_image[1]
            
    return lines


skeleton_links = []
with open('skeleton.txt') as f:
    while True:
        line = f.readline()
        if not line:
            break
        stripped = list(map(str.strip, line.strip().split(',')))
        skeleton_links.append(stripped)


world.tick()
trash = image_queue.get()


if not os.path.exists('out'):
    os.mkdir('out')

for frame in range(0, 360):
    # 在行人周围移动摄像头
    camera.set_transform(center_camera(pedestrian, frame + 200))

    # 推进帧并获取图像
    world.tick()
    # 从队列中获取帧
    image = image_queue.get()

    # 获取4x4矩阵以将点从世界坐标转换为相机坐标
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # 从相机获取一些属性
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    # 计算要从3D->2D投影的相机矩阵
    K = build_projection_matrix(image_w, image_h, fov)

    # 构建将显示骨架的线列表
    lines = build_skeleton(pedestrian, skeleton_links, K)

    # 将数据重塑为2D RBGA 矩阵
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    # 使用 OpenCV 将线条绘制到图像中
    for line in lines:
        l = [int(x) for x in line]
        cv2.line(img, (l[0],l[1]), (l[2],l[3]), (255,0,0, 255), 2)

    cv2.namedWindow('Pedestrian bones', cv2.WINDOW_NORMAL)
    cv2.imshow('Pedestrian bones', img)
    key = cv2.waitKey(1)
    # 保存图像
    cv2.imwrite('out/skeleton%04d.png' % frame, img)


# 重复运行在同一个位置还是会产生不同的人，销毁生成的人
pedestrian.destroy()

# todo 中途终止还是会冻结服务端窗口
# 防止结束时，仿真窗口冻结，不能操作，为手动操作界面做准备
settings = world.get_settings()
settings.synchronous_mode = False
settings.no_rendering_mode = False
settings.fixed_delta_seconds = None
world.apply_settings(settings)

