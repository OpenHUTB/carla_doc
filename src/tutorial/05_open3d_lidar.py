import time

import carla

import queue
import random

import cv2
import numpy as np

import math
import open3d as o3d
from matplotlib import cm

## Part 1

# Connect to Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# Get a vehicle from the library
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()

# Spawn a vehicle
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Autopilot
vehicle.set_autopilot(True) 

# Get the world spectator
spectator = world.get_spectator() 

## Part 2

# Auxilliary code for colormaps and axes
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
COOL = np.array(cm.get_cmap('winter')(COOL_RANGE))
COOL = COOL[:,:3]

def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))

    vis.add_geometry(axis)

# LIDAR and RADAR callbacks

def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    points = data[:, :-1]

    points[:, :1] = -points[:, :1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

def radar_callback(data, point_list):
    radar_data = np.zeros((len(data), 4))
    
    for i, detection in enumerate(data):
        x = detection.depth * math.cos(detection.altitude) * math.cos(detection.azimuth)
        y = detection.depth * math.cos(detection.altitude) * math.sin(detection.azimuth)
        z = detection.depth * math.sin(detection.altitude)
        
        radar_data[i, :] = [x, y, z, detection.velocity]
        
    intensity = np.abs(radar_data[:, -1])
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, COOL_RANGE, COOL[:, 0]),
        np.interp(intensity_col, COOL_RANGE, COOL[:, 1]),
        np.interp(intensity_col, COOL_RANGE, COOL[:, 2])]
    
    points = radar_data[:, :-1]
    points[:, :1] = -points[:, :1]
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    
# Camera callback
def camera_callback(image, rgb_camera_queue):
    rgb_camera_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

# Set up LIDAR and RADAR, parameters are to assisst visualisation

lidar_bp = bp_lib.find('sensor.lidar.ray_cast') 
lidar_bp.set_attribute('range', '100.0')
lidar_bp.set_attribute('noise_stddev', '0.1')
lidar_bp.set_attribute('upper_fov', '15.0')
lidar_bp.set_attribute('lower_fov', '-25.0')
lidar_bp.set_attribute('channels', '64.0')
lidar_bp.set_attribute('rotation_frequency', '20.0')
lidar_bp.set_attribute('points_per_second', '500000')
    
lidar_init_trans = carla.Transform(carla.Location(z=2))
lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)

radar_bp = bp_lib.find('sensor.other.radar') 
radar_bp.set_attribute('horizontal_fov', '30.0')
radar_bp.set_attribute('vertical_fov', '30.0')
radar_bp.set_attribute('points_per_second', '10000')
radar_init_trans = carla.Transform(carla.Location(z=2))
radar = world.spawn_actor(radar_bp, radar_init_trans, attach_to=vehicle)

# Spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb') 
camera_init_trans = carla.Transform(carla.Location(z=2.5, x=-3), carla.Rotation())
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Add auxilliary data structures
point_list = o3d.geometry.PointCloud()
radar_list = o3d.geometry.PointCloud()

# Set up Queue for camera data
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

rgb_camera_queue = queue.Queue()

# Start sensors
lidar.listen(lambda data: lidar_callback(data, point_list))
radar.listen(lambda data: radar_callback(data, radar_list))
camera.listen(lambda image: camera_callback(image, rgb_camera_queue))

# OpenCV window for camera
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

# Open3D visualiser for LIDAR and RADAR
vis = o3d.visualization.Visualizer()
vis.create_window(
    window_name='Carla Lidar',
    width=960,
    height=540,
    left=480,
    top=270)

vis.get_render_option().background_color = [0.05, 0.05, 0.05]
vis.get_render_option().point_size = 1
vis.get_render_option().show_coordinate_frame = True

add_open3d_axis(vis)

def clear():
    # Close displayws and stop sensors
    radar.stop()
    radar.destroy()

    lidar.stop()
    lidar.destroy()

    camera.stop()
    camera.destroy()

    vis.destroy_window()

    cv2.destroyAllWindows()


# Update geometry and camera in game loop
frame = 0
while True:
    try:
        if frame == 2:
            vis.add_geometry(point_list)
            vis.add_geometry(radar_list)

        vis.update_geometry(point_list)
        vis.update_geometry(radar_list)
        
        vis.poll_events()
        vis.update_renderer()

        # This can fix Open3D jittering issues:
        time.sleep(0.005)
        frame += 1

        cv2.imshow('RGB Camera', rgb_camera_queue.get())

        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Break if user presses 'q'
        if cv2.waitKey(1) == ord('q'):
            clear()
            break

    except KeyboardInterrupt as e:
        clear()
        break
