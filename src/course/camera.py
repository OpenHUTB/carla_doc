"""
相机显示车辆左右视角
"""
import random
import weakref
import cv2
import pygame
import numpy as np
import carla
import multiprocessing
import os

VIEW_WIDTH = 1920 // 2
VIEW_HEIGHT = 1080 // 2
VIEW_FOV = 90


class CarlaClient:

    def __init__(self):
        self.client = None
        self.world = None
        self.camera_left = None
        self.camera_right = None
        self.car = None
        self.image_left = None
        self.image_right = None
        self.capture = True

    def camera_bp(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        car_bp = self.world.get_blueprint_library().filter('model3')[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        # 左相机
        left_camera_transform = carla.Transform(carla.Location(x=-1.6, y=-1.5, z=1.7))
        self.camera_left = self.world.spawn_actor(self.camera_bp(), left_camera_transform, attach_to=self.car)
        # 右相机
        right_camera_transform = carla.Transform(carla.Location(x=-1.6, y=1.5, z=1.7))
        self.camera_right = self.world.spawn_actor(self.camera_bp(), right_camera_transform, attach_to=self.car)

        weak_self = weakref.ref(self)
        self.camera_left.listen(lambda image: weak_self().set_image(weak_self, 'left', image))
        self.camera_right.listen(lambda image: weak_self().set_image(weak_self, 'right', image))

    @staticmethod
    def set_image(weak_self, side, img):
        self = weak_self()
        if side == 'left':
            self.image_left = img
        elif side == 'right':
            self.image_right = img

    def get_image_array(self, side):
        if side == 'left' and self.image_left is not None:
            array = np.frombuffer(self.image_left.raw_data, dtype=np.dtype("uint8"))
            array = array.reshape((VIEW_HEIGHT, VIEW_WIDTH, 4))
            array = array[:, :, :3]  # Drop alpha channel
            array = array[:, :, ::-1]  # Convert from BGRA to RGB
            return array
        elif side == 'right' and self.image_right is not None:
            array = np.frombuffer(self.image_right.raw_data, dtype=np.dtype("uint8"))
            array = array.reshape((VIEW_HEIGHT, VIEW_WIDTH, 4))
            array = array[:, :, :3]  # Drop alpha channel
            array = array[:, :, ::-1]  # Convert from BGRA to RGB
            return array
        else:
            return None


def render(queue, side, x, y):
    pygame.init()
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{x},{y}"
    display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption(f'{side.capitalize()} Camera')
    clock = pygame.time.Clock()

    try:
        while True:
            array = queue.get()
            if array is None:
                break
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))
            pygame.display.flip()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
            clock.tick_busy_loop(30)
    finally:
        pygame.quit()


def client_loop(client, left_queue, right_queue):
    try:
        while True:
            client.world.tick()
            if client.image_left is not None:
                left_array = client.get_image_array('left')
                if left_array is not None:
                    left_queue.put(left_array)
            if client.image_right is not None:
                right_array = client.get_image_array('right')
                if right_array is not None:
                    right_queue.put(right_array)
    finally:
        client.set_synchronous_mode(False)
        if client.camera_left is not None:
            client.camera_left.destroy()
        if client.camera_right is not None:
            client.camera_right.destroy()
        if client.car is not None:
            client.car.destroy()


if __name__ == '__main__':
    try:
        cc = CarlaClient()
        cc.client = carla.Client('127.0.0.1', 2000)
        cc.client.set_timeout(5.0)
        cc.world = cc.client.get_world()

        # 生成车辆
        cc.setup_car()
        # 生成相机，且要满足车辆左右
        cc.setup_camera()

        # 设置同步模式
        cc.set_synchronous_mode(True)
        # 设置自动驾驶
        cc.car.set_autopilot(True)

        # 创建队列用于传递图像数据
        left_queue = multiprocessing.Queue()
        right_queue = multiprocessing.Queue()

        # 创建渲染进程，分别显示左右摄像头图像
        left_process = multiprocessing.Process(target=render, args=(left_queue, 'left', 0, 100))
        right_process = multiprocessing.Process(target=render, args=(right_queue, 'right', VIEW_WIDTH + 10, 100))

        left_process.start()
        right_process.start()

        # 主进程用于运行客户端
        client_loop(cc, left_queue, right_queue)

        left_process.join()
        right_process.join()

    except Exception as e:
        print(e)
    finally:
        cv2.destroyAllWindows()