import carla
import pygame
import numpy as np
import random

# 设置Pygame窗口大小
VIEW_WIDTH = 1920 // 2
VIEW_HEIGHT = 1080 // 2
VIEW_FOV = 90


class CarlaSimulator:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.*')[0]
        self.camera = None
        self.vehicles = []
        self.fov = VIEW_FOV

    def setup_camera(self):
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(self.width))
        camera_bp.set_attribute('image_size_y', str(self.height))
        camera_bp.set_attribute('fov', str(self.fov))
        camera_transform = carla.Transform(carla.Location(x=0, y=0, z=50), carla.Rotation(yaw=270, pitch=-90))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform)

    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((self.height, self.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # Convert BGR to RGB
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        return surface

    def convert_pygame_to_world(self, x, y):
        # 获取相机位置和旋转
        cam_transform = self.camera.get_transform()
        cam_loc = cam_transform.location

        # 计算相机的水平和垂直视角
        aspect_ratio = self.width / self.height
        camera_fov_rad = np.radians(self.fov)
        camera_fov_y = 2 * np.arctan(np.tan(camera_fov_rad / 2) / aspect_ratio)

        # 计算每个像素在实际世界中的对应距离
        world_units_per_pixel_x = (2 * cam_loc.z * np.tan(camera_fov_rad / 2)) / self.width
        world_units_per_pixel_y = (2 * cam_loc.z * np.tan(camera_fov_y / 2)) / self.height

        # 将Pygame坐标转换为相对于图像中心的坐标
        relative_x = (x - self.width / 2) * world_units_per_pixel_x
        relative_y = (y - self.height / 2) * world_units_per_pixel_y

        # 转换为世界坐标
        world_x = cam_loc.x + relative_x
        world_y = cam_loc.y - relative_y  # 注意这里是减去，因为y轴方向是相反的

        return world_x, world_y

    def run(self):
        pygame.init()
        display = pygame.display.set_mode((self.width, self.height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        clock = pygame.time.Clock()

        self.setup_camera()
        self.camera.listen(lambda image: display.blit(self.process_image(image), (0, 0)))

        spectator = self.world.get_spectator()

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    x, y = event.pos
                    # 将Pygame坐标转换为CARLA世界坐标
                    world_x, world_y = self.convert_pygame_to_world(x, y)
                    # print(f'World coordinates: ({world_x}, {world_y})')

                    # self.world.debug.draw_string(carla.Location(x=world_x, y=-world_y, z=1), 'ss', life_time=1000,
                    #                              color=carla.Color(255, 0, 0))
                    spawn_point = carla.Transform(carla.Location(x=world_x, y=-world_y, z=1), carla.Rotation())
                    vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point)
                    vehicle.set_autopilot()
                    # if self.vehicles is not None:
                    #     last_vehicle_transform = self.vehicles[-1].get_transform()
                    #     spectator.set_transform(last_vehicle_transform)
                    self.vehicles.append(vehicle)

            pygame.display.flip()
            clock.tick(30)

        for vehicle in self.vehicles:
            vehicle.destroy()
        self.camera.destroy()
        pygame.quit()


if __name__ == '__main__':
    simulator = CarlaSimulator(VIEW_WIDTH, VIEW_HEIGHT)
    simulator.run()
