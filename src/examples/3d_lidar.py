from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import time

import numpy as np
import open3d as o3d
from matplotlib import cm

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

try:
    sys.path.append(
        glob.glob(
            "%s/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
            % (
                os.environ.get("CARLA_ROOT", "."),
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import carla

from carla_world import CarlaWorld

VIRIDIS = np.array(cm.get_cmap("plasma").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
COOL = np.array(cm.get_cmap("winter")(COOL_RANGE))
COOL = COOL[:, :3]


def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    )
    axis.lines = o3d.utility.Vector2iVector(np.array([[0, 1], [0, 2], [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(
        np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    )
    vis.add_geometry(axis)


def initialize_world(args):
    """Create world instance and create/retrieve hero actor"""
    client = carla.Client(args.host, args.port)
    carla_world = CarlaWorld(client)
    if args.map:
        carla_world.load_world(args.map)

    hero = carla_world.find_actor_by_name(args.rolename)
    if hero is None:
        hero = carla_world.spawn_actor(args.rolename, "vehicle.tesla.model3")

    return carla_world, hero


def lidar_callback(point_cloud: carla.LidarMeasurement, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype("f4")))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2]),
    ]

    points = data[:, :-1]

    points[:, :1] = -points[:, :1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def radar_callback(data: carla.RadarMeasurement, point_list):
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
        np.interp(intensity_col, COOL_RANGE, COOL[:, 2]),
    ]

    points = radar_data[:, :-1]
    points[:, :1] = -points[:, :1]
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)


def setup_sensors(carla_world: CarlaWorld, actor):
    # Set up LIDAR and RADAR, parameters are to assisst visualisation
    lidar = carla_world.spawn_sensor(
        "sensor.lidar.ray_cast",
        actor,
        carla.Transform(carla.Location(z=2)),
        range=100.0,
        noise_stddev=0.1,
        upper_fov=15.0,
        lower_fov=-25.0,
        channels=64,
        rotation_frequency=20,
        points_per_second=500000,
    )

    radar = carla_world.spawn_sensor(
        "sensor.other.radar",
        actor,
        carla.Transform(carla.Location(z=2)),
        horizontal_fov=30.0,
        vertical_fov=30.0,
        points_per_second=10000,
    )

    # Add auxilliary data structures
    point_list = o3d.geometry.PointCloud()
    radar_list = o3d.geometry.PointCloud()

    # Start sensors
    lidar.listen(lambda data: lidar_callback(data, point_list))
    radar.listen(lambda data: radar_callback(data, radar_list))
    return point_list, radar_list


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", type=str, default=None)
    parser.add_argument("--rolename", type=str, default="hero")
    parser.add_argument("--keep_ego_vehicle", action="store_true")
    args = parser.parse_args()

    carla_world, hero = initialize_world(args)
    point_list, radar_list = setup_sensors(carla_world, hero)

    # Open3D visualiser for LIDAR and RADAR
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Carla Lidar", width=960, height=540, left=480, top=270)
    vis.get_render_option().background_color = [0.05, 0.05, 0.05]
    vis.get_render_option().point_size = 1
    vis.get_render_option().show_coordinate_frame = True
    add_open3d_axis(vis)

    # Update geometry and camera in game loop
    frame = 0
    try:
        while True:
            if frame == 2:
                vis.add_geometry(point_list)
                vis.add_geometry(radar_list)
            vis.update_geometry(point_list)
            vis.update_geometry(radar_list)

            vis.poll_events()
            vis.update_renderer()
            # # This can fix Open3D jittering issues:
            time.sleep(0.005)
            frame += 1
    finally:
        vis.destroy_window()
        if args.keep_ego_vehicle:
            carla_world.unregister_actor(hero)
        carla_world.cleanup()
