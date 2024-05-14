from __future__ import annotations

import argparse
import glob
import os
import sys

import cv2
import numpy as np

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


def spawn_rgb_cameras(
    carla_world: CarlaWorld, hero: carla.Vehicle, image_size_x: int, image_size_y: int
):
    camera_init_trans = carla.Transform(carla.Location(z=2))

    camera = carla_world.spawn_sensor(
        "sensor.camera.rgb",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )
    sem_camera = carla_world.spawn_sensor(
        "sensor.camera.semantic_segmentation",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )
    inst_camera = carla_world.spawn_sensor(
        "sensor.camera.instance_segmentation",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )
    depth_camera = carla_world.spawn_sensor(
        "sensor.camera.depth",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )
    dvs_camera = carla_world.spawn_sensor(
        "sensor.camera.dvs",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )
    opt_camera = carla_world.spawn_sensor(
        "sensor.camera.optical_flow",
        hero,
        camera_init_trans,
        image_size_x=image_size_x,
        image_size_y=image_size_y,
    )

    camera_dict = {
        "rgb": camera,
        "semantic": sem_camera,
        "instance": inst_camera,
        "depth": depth_camera,
        "dvs": dvs_camera,
        "optical": opt_camera,
    }
    return camera_dict


def setup_callbacks(camera_dict: dict[str, carla.Sensor]):
    sensor = next(iter(camera_dict.values()))
    image_w = int(sensor.attributes["image_size_x"])
    image_h = int(sensor.attributes["image_size_y"])
    sensor_data = {
        "rgb_image": np.zeros((image_h, image_w, 4)),
        "sem_image": np.zeros((image_h, image_w, 4)),
        "depth_image": np.zeros((image_h, image_w, 4)),
        "dvs_image": np.zeros((image_h, image_w, 4)),
        "opt_image": np.zeros((image_h, image_w, 4)),
        "inst_image": np.zeros((image_h, image_w, 4)),
    }

    def rgb_callback(image: carla.Image, data_dict: dict):
        data_dict["rgb_image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )

    def sem_callback(image: carla.Image, data_dict: dict):
        image.convert(carla.ColorConverter.CityScapesPalette)
        data_dict["sem_image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )

    def inst_callback(image: carla.Image, data_dict: dict):
        data_dict["inst_image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )

    def depth_callback(image: carla.Image, data_dict: dict):
        image.convert(carla.ColorConverter.LogarithmicDepth)
        data_dict["depth_image"] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4)
        )

    def opt_callback(data: carla.Image, data_dict: dict):
        image = data.get_color_coded_flow()
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        img[:, :, 3] = 255
        data_dict["opt_image"] = img

    def dvs_callback(data: carla.Image, data_dict: dict):
        dvs_events = np.frombuffer(
            data.raw_data,
            dtype=np.dtype(
                [("x", np.uint16), ("y", np.uint16), ("t", np.int64), ("pol", bool)]
            ),
        )
        data_dict["dvs_image"] = np.zeros((data.height, data.width, 4), dtype=np.uint8)
        dvs_img = np.zeros((data.height, data.width, 3), dtype=np.uint8)
        dvs_img[dvs_events[:]["y"], dvs_events[:]["x"], dvs_events[:]["pol"] * 2] = 255
        data_dict["dvs_image"][:, :, 0:3] = dvs_img

    camera_dict["rgb"].listen(lambda image: rgb_callback(image, sensor_data))
    camera_dict["semantic"].listen(lambda image: sem_callback(image, sensor_data))
    camera_dict["instance"].listen(lambda image: inst_callback(image, sensor_data))
    camera_dict["depth"].listen(lambda image: depth_callback(image, sensor_data))
    camera_dict["dvs"].listen(lambda image: dvs_callback(image, sensor_data))
    camera_dict["optical"].listen(lambda image: opt_callback(image, sensor_data))
    return sensor_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--map", type=str, default=None)
    parser.add_argument("--rolename", type=str, default="hero")
    parser.add_argument("--image_size_x", type=int, default=256)
    parser.add_argument("--image_size_y", type=int, default=256)
    parser.add_argument("--keep_ego_vehicle", action="store_true")
    args = parser.parse_args()

    carla_world, hero = initialize_world(args)
    camera_dict = spawn_rgb_cameras(
        carla_world, hero, args.image_size_x, args.image_size_y
    )
    sensor_data = setup_callbacks(camera_dict)

    # OpenCV named window for display
    cv2.namedWindow("All cameras", cv2.WINDOW_AUTOSIZE)

    # Tile all data in one array
    top_row = np.concatenate(
        (sensor_data["rgb_image"], sensor_data["sem_image"], sensor_data["inst_image"]),
        axis=1,
    )
    lower_row = np.concatenate(
        (
            sensor_data["depth_image"],
            sensor_data["dvs_image"],
            sensor_data["opt_image"],
        ),
        axis=1,
    )
    tiled = np.concatenate((top_row, lower_row), axis=0)

    # Display with imshow
    cv2.imshow("All cameras", tiled)
    cv2.waitKey(1)

    try:
        while True:
            # Tile camera images into one array
            top_row = np.concatenate(
                (
                    sensor_data["rgb_image"],
                    sensor_data["sem_image"],
                    sensor_data["inst_image"],
                ),
                axis=1,
            )
            lower_row = np.concatenate(
                (
                    sensor_data["depth_image"],
                    sensor_data["dvs_image"],
                    sensor_data["opt_image"],
                ),
                axis=1,
            )
            tiled = np.concatenate((top_row, lower_row), axis=0)
            cv2.imshow("All cameras", tiled)

            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()

        if args.keep_ego_vehicle:
            carla_world.unregister_actor(hero)
        carla_world.cleanup()
