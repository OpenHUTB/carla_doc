#!/usr/bin/env python

import glob
import os
import sys
import matplotlib.pyplot as plt
import networkx as nx
from matplotlib import pyplot as plt
import carla
import random
import time
import numpy as np
import pygame
import math

VIEW_WIDTH = 1920 // 2
VIEW_HEIGHT = 1080 // 2
VIEW_FOV = 90

# Appending paths - DO NOT CHANGE
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    import carla
except IndexError:
    pass

from motion_primitve import motion_primitive
from basic_PID import VehiclePIDController

from RRT_VO import RRT_VO


# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0, 255, (height, width, 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))


# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:, :, :3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))


class CARLA_world:

    def __init__(self):
        # Connect to the client and retrieve the world object
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(120.0)  # increase time-out buffer if system is slow in responding
        self.carla_world = self.client.get_world()
        self.map = self.carla_world.get_map()

        print("WORLD READY")

        # Set up the simulator in synchronous mode
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = 0.02
        self.carla_world.apply_settings(settings)

        # We will also set up the spectator so we can see what we do
        self.spectator = self.carla_world.get_spectator()

        self.vehicles = []
        self.ego_vehicle = None


if __name__ == '__main__':

    CARLA_world = CARLA_world()
    # Retrieve the map's spawn points
    spawn_points = CARLA_world.map.get_spawn_points()
    # print(spawn_points)

    # 调整目标速度和PID控制器参数
    target_speed = 3.0  # 自车目标速度 (单位: m/s)
    v_obs = 3.0  # 障碍物车辆目标速度 (单位: m/s)
    try:
        # VEHICLE 1 SPAWN
        spawn_point1 = carla.Transform(carla.Location(x=428, y=-51.9, z=0.3), carla.Rotation(yaw=89))
        blueprint_library = CARLA_world.carla_world.get_blueprint_library()
        bp1 = blueprint_library.filter("model3")[0]
        vehicle1 = CARLA_world.carla_world.spawn_actor(bp1, spawn_point1)
        CARLA_world.vehicles.append(vehicle1)
        # EGO VEHICLE SPAWN
        spawn_point3 = carla.Transform(carla.Location(x=404, y=-14, z=0.3), carla.Rotation(yaw=0))
        bp3 = blueprint_library.filter("model3")[0]
        CARLA_world.ego_vehicle = CARLA_world.carla_world.spawn_actor(bp3, spawn_point3)

        # Initialise the camera floating behind the vehicle
        camera_init_trans = carla.Transform(carla.Location(x=-10, y=-4, z=3), carla.Rotation(pitch=-20))
        camera_bp = CARLA_world.carla_world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        camera = CARLA_world.carla_world.spawn_actor(camera_bp, camera_init_trans, attach_to=CARLA_world.ego_vehicle)

        # Instantiate objects for rendering and vehicle control
        renderObject = RenderObject(VIEW_WIDTH, VIEW_HEIGHT)

        # Start camera with PyGame callback
        camera.listen(lambda image: pygame_callback(image, renderObject))

        # Initialise the display
        pygame.init()
        gameDisplay = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Camera View")

        # map = world.get_map()
        goal = CARLA_world.map.get_waypoint(carla.Location(x=444, y=-14), project_to_road=True)
        # print(goal)
        # start = CARLA_world.map.get_waypoint(carla.Location(x=396, y=105), project_to_road=True)
        # print(    goal.transform.location.x,     goal.transform.location.y)
        CARLA_world.carla_world.tick()
        trans = CARLA_world.vehicles[0].get_transform()
        v_obs = 7.8
        # trans2 = CARLA_world.vehicles[1].get_transform()
        # trans3 = CARLA_world.vehicles[2].get_transform()
        obstacles = [trans, v_obs]
        # print(trans.location, trans.rotation.yaw)

        RRT_planner = RRT_VO(CARLA_world, goal, obstacles)
        RRT_planner.RRT_star(n_pts=1000)
        # print(RRT_planner.path[0].waypoint)
        path = RRT_planner.path

        # trajectory generation

        path_x = []
        path_y = []
        trans = CARLA_world.ego_vehicle.get_transform()
        thetai = trans.rotation.yaw * math.pi / 180
        final_theta = thetai
        print(thetai)
        for i in range(len(path) - 1):
            print(i)
            if i == len(path) - 2:
                thetaf = final_theta
            else:
                x1, y1 = path[i + 1].x, path[i + 1].y
                x2, y2 = path[i + 2].x, path[i + 2].y

                thetaf = math.atan2((y2 - y1), (x2 - x1))
                print(x1, y1, x2, y2)

            print(thetaf)

            primitive = motion_primitive(thetai, thetaf, path[i].x,
                                         path[i + 1].x, path[i].y,
                                         path[i + 1].y)
            primitive.cubic_T_Matrix()
            primitive.trajectory()

            pos_x, pos_y = primitive.get_path(0.05)

            path_x += pos_x
            path_y += pos_y
            thetai = thetaf

        print(path_x)
        print(path_y)

        # Game loop
        # controller = VehiclePIDController(CARLA_world.ego_vehicle, [5, 5, 0], [1, 1, 0])
        # controller_obs = VehiclePIDController(CARLA_world.vehicles[0], [5, 5, 0], [1, 1, 0])
        controller = VehiclePIDController(CARLA_world.ego_vehicle, [1.0, 0.05, 0.1], [0.5, 0.05, 0.1])
        controller_obs = VehiclePIDController(CARLA_world.vehicles[0], [1.0, 0.05, 0.1], [0.5, 0.05, 0.1])
        # CARLA_world.vehicles[0].enable_constant_velocty(v_obs)
        actual_x = []
        actual_y = []
        ti = time.time()
        for i in range(len(path_x) - 1):
            trans = CARLA_world.ego_vehicle.get_transform()
            loc_x, loc_y = trans.location.x, trans.location.y

            w_x, w_y = path_x[i], path_y[i]
            w_x2, w_y2 = path_x[i + 1], path_y[i + 1]
            phi = math.atan2((w_y2 - w_y), (w_x2 - w_x))
            physics = CARLA_world.ego_vehicle.get_physics_control()
            wheels = physics.wheels
            wheel_F_x = (wheels[0].position.x + wheels[1].position.x) / 200
            wheel_F_y = (wheels[0].position.y + wheels[1].position.y) / 200
            print("_____________________///////////////BREAK//////////////___________________")
            while ((wheel_F_x - w_x2) ** 2 + (wheel_F_y - w_y2) ** 2) ** 0.5 >= 0.5:

                control = controller.run_step(RRT_planner.v * 3.6 / 3)
                # control_obs = controller_obs.run_step(v_obs * 3.6)
                # control = controller.run_step(target_speed)
                control_obs = controller_obs.run_step(v_obs * 3.6 / 3)
                # print("loc", w_x2, w_y2)
                print("wheel", wheel_F_x, wheel_F_y)
                p1 = np.array([w_x, w_y])
                p2 = np.array([w_x2, w_y2])
                p3 = np.array([wheel_F_x, wheel_F_y])

                trans = CARLA_world.ego_vehicle.get_transform()
                yaw = trans.rotation.yaw

                phi = math.atan2((w_y2 - w_y), (w_x2 - w_x)) - yaw * (math.pi / 180)

                d = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)
                # print("line", d)
                # print("phi", new_phi)
                kp = 3
                ks = 0.2
                Vel = CARLA_world.ego_vehicle.get_velocity()
                v = math.sqrt(Vel.x ** 2 + Vel.y ** 2)
                # print("vel", v)
                # print(math.atan2(kp*d,ks+v))
                # print(phi)
                control.steer = (-math.atan2(kp * d, ks + v) + phi)
                control_obs.steer = 0
                # print("steer", control.steer)

                CARLA_world.ego_vehicle.apply_control(control)
                CARLA_world.vehicles[0].apply_control(control_obs)

                # Advance the simulation time
                CARLA_world.carla_world.tick()

                physics = CARLA_world.ego_vehicle.get_physics_control()
                wheels = physics.wheels
                wheel_F_x = (wheels[0].position.x + wheels[1].position.x) / 200
                wheel_F_y = (wheels[0].position.y + wheels[1].position.y) / 200

                # Update the display
                gameDisplay.fill((0, 0, 0))
                gameDisplay.blit(renderObject.surface, (0, 0))
                pygame.display.flip()
                actual_x.append(wheel_F_x)
                actual_y.append(wheel_F_y)

        plt.scatter(actual_x, actual_y)
        plt.show()
        camera.stop()
        pygame.quit()
    finally:
        settings = CARLA_world.carla_world.get_settings()
        settings.synchronous_mode = False
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = None
        CARLA_world.carla_world.apply_settings(settings)
