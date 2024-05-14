"""
Author: Morphlng
Date: 2023-12-02 21:26:33
LastEditTime: 2023-12-16 22:41:43
LastEditors: Morphlng
Description: Helper function that is useful in Carla
FilePath: /carla-interactive-script/src/carla_world.py
"""
from __future__ import annotations

import glob
import os
import random
import sys
import threading

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

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
import numpy as np

from local_carla_api import Transform


class CarlaWorld:
    def __init__(
        self, client: carla.Client, world: carla.World = None, map: carla.Map = None
    ):
        self.client = client
        """Carla client handler"""

        self._carla_actor_pool: dict[int, carla.Actor] = {}
        """Save all the actors that are spawned by this class."""

        self._carla_actor_map: dict[str, int] = {}
        """A map from role name to actor id (int)."""

        self._carla_sensor_pool: dict[int, carla.Sensor] = {}
        """Save all the sensors that are spawned by this class."""

        self._view_thread: threading.Thread = None
        """The thread that is used to follow an actor."""

        self._init_world(world, map)

    def _init_world(self, world: carla.World = None, map: carla.Map = None):
        """Initialize the world and map.

        Args:
            world (carla.World, optional): If None, use the current world. Defaults to None.
            map (carla.Map, optional): If None, use the current map. Defaults to None.
        """
        self.world = world or self.client.get_world()
        self.map = map or self.world.get_map()
        self.map_spawn_points = self.map.get_spawn_points()
        self.blueprint_library = self.world.get_blueprint_library()
        self.spectator = self.world.get_spectator()
        self.sync_mode = self.world.get_settings().synchronous_mode
        self.traffic_manager = self.client.get_trafficmanager()

        self._carla_actor_map.clear()
        self._carla_sensor_pool.clear()
        self._carla_actor_pool.clear()

    def set_view(self, reference_transform: carla.Transform, view_from: str = "above"):
        """Set the spectator view to a given transform.

        Args:
            reference_transform (carla.Transform): The reference transform.
            view_from (str, optional): The view position. Defaults to "above".
        """
        view_transform = carla.Transform()
        ref_forward = reference_transform.get_forward_vector()
        ref_loc = reference_transform.location
        ref_rot = reference_transform.rotation

        if view_from == "above":
            view_transform.location = ref_loc + carla.Location(z=100)
            view_transform.rotation = carla.Rotation(pitch=-90)
        elif view_from == "behind":
            view_transform.location = ref_loc - ref_forward * 10 + carla.Location(z=3)
            view_transform.rotation = ref_rot
        elif view_from == "front":
            view_transform.location = ref_loc + ref_forward * 3 + carla.Location(z=2)
            view_transform.rotation = ref_rot
        else:
            view_transform = reference_transform

        self.spectator.set_transform(view_transform)

    def follow_actor_view(self, actor: carla.Actor, view_from: str = "above"):
        """Follow an actor with the spectator view.

        Args:
            actor (carla.Actor): The actor to follow.
            view_from (str, optional): The view position. Defaults to "above".
        """
        if self._view_thread is not None:
            self.stop_follow_actor_view()

        self._following_actor = True

        def _follow_actor_view_thread(actor: carla.Actor, view_from: str):
            while self._following_actor:
                self.set_view(actor.get_transform(), view_from)
                self.tick()

        self._view_thread = threading.Thread(
            target=_follow_actor_view_thread, args=(actor, view_from)
        )
        self._view_thread.start()

    def stop_follow_actor_view(self):
        """Stop following the actor."""
        self._following_actor = False
        if self._view_thread is not None:
            self._view_thread.join()
            self._view_thread = None

    def toggle_sync_mode(self, sync: bool = False, fixed_delta_seconds: float = None):
        """Toggle the sync mode of the world.

        Args:
            sync (bool, optional): Whether to enable sync mode. Defaults to False.
            fixed_delta_seconds (float, optional): Fixed delta seconds. Defaults to None (realtime).
        """
        settings = self.world.get_settings()
        self.sync_mode = settings.synchronous_mode = sync
        settings.fixed_delta_seconds = fixed_delta_seconds
        self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(sync)

    def load_world(self, map_name: str):
        """Load a new world.

        Args:
            map_name (str): The name of the map to load.
        """
        world = self.client.load_world(map_name)
        world.wait_for_tick()
        self._init_world(world)

    def reload_world(self, reset_settings: bool = True):
        """Reload the current world.

        Args:
            reset_settings (bool, optional): Whether to reset the world settings. Defaults to True.
        """
        world = self.client.reload_world(reset_settings)
        world.wait_for_tick()
        self._init_world(world)

    def tick(self, cnt: int = 1):
        """Tick the simulator.

        Args:
            cnt (int, optional): How many ticks to tick. Defaults to 1.

        Returns:
            frame (int): The frame number after ticking.
        """
        for _ in range(cnt):
            if self.sync_mode:
                frame = self.world.tick()
            else:
                snapshot = self.world.wait_for_tick()
                frame = snapshot.frame

        return frame

    def spawn_actor(
        self, role_name: str, bp_name: str, transform: carla.Transform = None
    ):
        """Spawn an actor with a given role name, blueprint name and transform.

        Args:
            role_name (str): The role name of the actor.
            bp_name (str): Either the specific blueprint name or a wildcard.
            transform (carla.Transform, optional): The spawn transform. If None, a random transform is chosen. Defaults to None.

        Returns:
            actor (carla.Actor): The spawned actor.
        """
        bp = random.choice(self.blueprint_library.filter(bp_name))
        bp.set_attribute("role_name", role_name)

        if transform is None:
            transform = random.choice(self.map_spawn_points)
            transform.location += carla.Location(z=2.0)

        actor = self.world.spawn_actor(bp, transform)
        self.register_actor(actor)
        return actor

    def spawn_sensor(
        self,
        bp_name: str,
        parent: carla.Actor,
        transform: carla.Transform = None,
        attchment_type: carla.AttachmentType = carla.AttachmentType.Rigid,
        **attributes,
    ) -> carla.Sensor:
        """Spawn a sensor with a given blueprint name and parent actor.

        Args:
            bp_name (str): The specific blueprint name.
            parent (carla.Actor): The parent actor.
            transform (carla.Transform, optional): The spawn transform. Defaults to carla.Transform().
            attchment_type (carla.AttachmentType, optional): The attachment type. Defaults to carla.AttachmentType.Rigid.
            **attributes: The attributes of the sensor.

        Returns:
            sensor (carla.Sensor): The spawned sensor.
        """
        bp = self.blueprint_library.find(bp_name)
        for key, value in attributes.items():
            if bp.has_attribute(key):
                bp.set_attribute(key, str(value))

        transform = transform or carla.Transform()
        sensor = self.world.spawn_actor(bp, transform, parent, attchment_type)
        self.register_sensor(sensor)
        return sensor

    def get_actors(
        self,
        filter_attachment: bool = True,
        filter_misc: bool = True,
        filter_vehicle: bool = False,
        filter_walker: bool = False,
        keep_all: bool = False,
        wildcard_pattern: str = None,
    ):
        """Get all the actors in the world.

        Args:
            filter_attachment (bool, optional): Whether to filter out the attached actors. Defaults to True.
            filter_misc (bool, optional): Whether to filter out the misc actors. Defaults to True.
            filter_vehicle (bool, optional): Whether to filter out the vehicles. Defaults to False.
            filter_walker (bool, optional): Whether to filter out the walkers. Defaults to False.
            keep_all (bool, optional): Whether to keep all the actors. Defaults to False.
            wildcard_pattern (str, optional): The wildcard pattern to filter the actors. Defaults to None.

        Returns:
            list[carla.Actor]: A list of actors.
        """
        actors = self.world.get_actors()

        if wildcard_pattern is not None:
            actors = actors.filter(wildcard_pattern)

        if keep_all:
            return actors

        filtered_actors = []
        for actor in actors:
            if not actor.is_active or not actor.is_alive:
                continue

            if filter_attachment and actor.parent is not None:
                continue

            if filter_misc and isinstance(
                actor, (carla.TrafficSign, carla.TrafficLight)
            ):
                continue

            if filter_walker and isinstance(actor, carla.Walker):
                continue
            if filter_vehicle and isinstance(actor, carla.Vehicle):
                continue

            filtered_actors.append(actor)

        return filtered_actors

    def find_actor_by_blueprint(self, bp_name: str):
        """Find the first actor using the given blueprint

        Args:
            bp_name (str): The blueprint we're looking for

        Return:
            carla.Actor: The first actor using the given blueprint
        """
        for actor in self.world.get_actors():
            if actor.type_id == bp_name:
                return actor

        return None

    def find_actor_by_name(self, target_name: str, ignore_case: bool = True):
        """Find an actor with a given role name.

        Args:
            target_name (str): The target role name of the actor.
            ignore_case (bool, optional): Whether to ignore case when comparing role names. Defaults to True.

        Returns:
            carla.Actor: The actor with the given role name.
        """
        if ignore_case:
            target_name = target_name.lower()

        for name in self._carla_actor_map:
            if ignore_case:
                name = name.lower()

            if name == target_name:
                return self._carla_actor_pool[self._carla_actor_map[name]]

        for actor in self.world.get_actors():
            actor_name = actor.attributes.get("role_name", "")
            if ignore_case:
                actor_name = actor_name.lower()

            if actor_name == target_name:
                return actor

        return None

    def destroy_actor(self, actor: carla.Actor):
        """Destroy an actor.

        Args:
            actor (carla.Actor): The actor to destroy.
        """
        # Check if actor is registered
        id = actor.id
        if id in self._carla_actor_pool:
            actor = self._carla_actor_pool.pop(id)
        elif id in self._carla_sensor_pool:
            actor = self._carla_sensor_pool.pop(id)

        role_name = actor.attributes.get("role_name", None)
        if role_name in self._carla_actor_map:
            self._carla_actor_map.pop(role_name)

        if getattr(actor, "is_listening", False):
            actor.stop()
        if actor.is_alive and actor.is_active:
            actor.destroy()

    def register_actor(self, actor: carla.Actor):
        """Register an actor to the pool.

        Args:
            actor (carla.Actor): The actor to register.

        Raises:
            ValueError: If the actor already exists in the pool.
        """
        if actor.id in self._carla_actor_pool:
            raise ValueError(f"Actor {actor.id} already exists in the pool.")

        rolename = actor.attributes.get(
            "role_name", f"misc_{len(self._carla_actor_pool)}"
        )
        self._carla_actor_pool[actor.id] = actor
        self._carla_actor_map[rolename] = actor.id

    def register_sensor(self, sensor: carla.Sensor):
        """Register a sensor to the pool.

        Args:
            sensor (carla.Sensor): The sensor to register.

        Raises:
            ValueError: If the sensor already exists in the pool.
        """
        if sensor.id in self._carla_sensor_pool:
            raise ValueError(f"Sensor {sensor.id} already exists in the pool.")

        self._carla_sensor_pool[sensor.id] = sensor

    def unregister_actor(self, actor: carla.Actor):
        """Unregister an actor from the pool.

        Args:
            actor (carla.Actor): The actor to unregister.
        """
        if actor.id not in self._carla_actor_pool:
            return

        role_name = actor.attributes.get("role_name", "")
        if role_name in self._carla_actor_map:
            self._carla_actor_map.pop(role_name)
        self._carla_actor_pool.pop(actor.id)

    def unregister_sensor(self, sensor: carla.Sensor):
        """Unregister a sensor from the pool.

        Args:
            sensor (carla.Sensor): The sensor to unregister.
        """
        if sensor.id not in self._carla_sensor_pool:
            return

        self._carla_sensor_pool.pop(sensor.id)

    def cleanup(self):
        """Cleanup the world."""
        for sensor in self._carla_sensor_pool.values():
            if sensor.is_listening:
                sensor.stop()
            if sensor.is_alive and sensor.is_active:
                sensor.destroy()

        for actor in self._carla_actor_pool.values():
            if actor.is_alive and actor.is_active:
                actor.destroy()

        self._carla_actor_pool.clear()
        self._carla_actor_map.clear()
        self._carla_sensor_pool.clear()

    def draw_bounding_box(
        self,
        actor: carla.Actor,
        thickness: float = 0.2,
        color: tuple[int, ...] = (255, 0, 0),
        life_time: float = 10,
    ):
        """Draw the bounding box of an actor.

        Args:
            actor (carla.Actor): The actor to draw the bounding box of.
            thickness (float, optional): The thickness of the lines. Defaults to 0.1.
            color (tuple, optional): The color of the lines. Defaults to (255, 0, 0).
            life_time (float, optional): The life time of the drawn bounding box. Defaults to infinite.
        """
        bb = carla.BoundingBox(actor.bounding_box.location, actor.bounding_box.extent)
        transform = actor.get_transform()
        bb.location += transform.location

        self.world.debug.draw_box(
            bb,
            transform.rotation,
            thickness=thickness,
            color=carla.Color(*color),
            life_time=life_time,
        )

    def draw_polygon(
        self,
        points: list[carla.Location],
        thickness: float = 0.1,
        color: tuple[int, ...] = (255, 0, 0),
        life_time: int = 3,
    ):
        """Draw the given points as a polygon in Carla world

        Args:
            points (list[carla.Location]): The points list
            thickness (float, optional): The thickness of the lines. Defaults to 0.1.
            color (tuple, optional): The color of the lines. Defaults to (255, 0, 0).
            life_time (float, optional): The life time of the drawn bounding box. Defaults to infinite.
        """
        for i in range(len(points)):
            self.world.debug.draw_line(
                points[i],
                points[(i + 1) % len(points)],
                thickness,
                carla.Color(*color),
                life_time,
            )

    def get_waypoint_from_spectator(self, lane_type=carla.LaneType.Any):
        """Get the waypoint from the spectator.

        Args:
            lane_type (carla.LaneType, optional): What type of lane to get. Defaults to carla.LaneType.Any.

        Returns:
            carla.Waypoint: The waypoint.
        """
        transform = self.spectator.get_transform()
        waypoint = self.map.get_waypoint(transform.location, lane_type=lane_type)
        return waypoint

    def proj_to_camera(self, points: np.ndarray, camera: carla.Sensor):
        """Project a points/Location to the sensor.

        Args:
            points (np.ndarray): The points in world coordinate.
            camera (carla.Sensor): The camera to project to.

        Returns:
            np.ndarray: The projected points.
        """
        if not hasattr(camera, "calibration"):
            x_res = camera.attributes["image_size_x"]
            y_res = camera.attributes["image_size_y"]
            fov = camera.attributes["fov"]
            calibration = np.identity(3)
            calibration[0, 2] = int(x_res) / 2.0
            calibration[1, 2] = int(y_res) / 2.0
            calibration[0, 0] = calibration[1, 1] = int(x_res) / (
                2.0 * np.tan(int(fov) * np.pi / 360.0)
            )
            camera.calibration = calibration

        # Project from world frame to sensor frame
        cam_trans = Transform.from_simulator_transform(camera.get_transform())
        sensor_points = np.matrix(cam_trans.inverse_transform_points(points).T)

        # Calibrate the points
        cords_y_minus_z_x = np.concatenate(
            [sensor_points[1, :], -sensor_points[2, :], sensor_points[0, :]]
        )
        pp = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_points = np.concatenate(
            [pp[:, 0] / pp[:, 2], pp[:, 1] / pp[:, 2], pp[:, 2]], axis=1
        )
        return camera_points

    @staticmethod
    def relative_position(
        reference_transform: carla.Transform,
        transform: carla.Transform,
        side_range: float = 1.0,
    ) -> str:
        """Given two transforms, tell the relative position of the second transform with respect to the first transform.

        Args:
            reference_transform (Transform): The reference transform.
            transform (Transform): The transform to be compared.
            side_range (float): The range to consider the second transform on the side of the reference transform.

        Returns:
            str: The relative position of the second transform with respect to the first transform.
        """

        def to_numpy(vector: carla.Vector3D):
            return np.array([vector.x, vector.y, vector.z])

        reference_forward = to_numpy(reference_transform.get_forward_vector())
        reference_loc = to_numpy(reference_transform.location)
        current_loc = to_numpy(transform.location)
        ref_to_cur = current_loc - reference_loc

        forward_projection = np.dot(ref_to_cur, reference_forward)
        reference_side = np.array([-reference_forward[1], reference_forward[0], 0])
        side_projection = np.dot(ref_to_cur, reference_side)

        if abs(forward_projection) <= side_range and abs(side_projection) <= side_range:
            return "left_side" if side_projection < 0 else "right_side"
        elif forward_projection > 0:
            return "front"
        elif forward_projection < 0:
            return "rear"
