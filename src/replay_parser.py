"""
Author: Morphlng
Date: 2023-12-16 15:36:03
LastEditTime: 2023-12-17 21:36:14
LastEditors: Morphlng
Description: Replay parser for CARLA
FilePath: /carla-interactive-script/src/replay_parser.py
"""
from __future__ import annotations

import functools
import glob
import math
import os
import re
import sys
import weakref
from collections import namedtuple
from dataclasses import dataclass

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


def weak_lru(maxsize: int = 128, typed: bool = False):
    """LRU Cache decorator that keeps a weak reference to 'self'"""

    def wrapper(func):
        @functools.lru_cache(maxsize, typed)
        def _func(_self, *args, **kwargs):
            return func(_self(), *args, **kwargs)

        @functools.wraps(func)
        def inner(self, *args, **kwargs):
            return _func(weakref.ref(self), *args, **kwargs)

        return inner

    return wrapper


class Vector3D(namedtuple("Vector3D", ["x", "y", "z"])):
    """Vector3D base class"""

    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def __sub__(self, other):
        return type(self)(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return type(self)(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, other: float):
        return type(self)(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other: float):
        return type(self)(self.x / other, self.y / other, self.z / other)

    def as_simulator_vector(self):
        return carla.Vector3D(self.x, self.y, self.z)

    def __hash__(self) -> int:
        return hash((self.x, self.y, self.z))


class Location(Vector3D):
    """Location of an actor"""

    def as_simulator_location(self):
        return carla.Location(self.x, self.y, self.z)


class Rotation(namedtuple("Rotation", ["pitch", "yaw", "roll"])):
    """Rotation of an actor"""

    def as_simulator_rotation(self):
        return carla.Rotation(self.pitch, self.yaw, self.roll)

    def __hash__(self) -> int:
        return hash((self.pitch, self.yaw, self.roll))


@dataclass
class Actor:
    id: int = None
    """Carla actor id"""

    blueprint: str = None
    """Carla blueprint name"""

    spawn_point: Location = None
    """Spawn location (x,y,z) of the actor"""

    attributes: dict = None
    """Extra attributes of the actor"""

    sensors: list[Actor] = None
    """Sensors attached to the actor"""

    def __post_init__(self):
        if self.attributes is None:
            self.attributes = {}
        if self.sensors is None:
            self.sensors = []
        self.spawn_point = Location(*self.spawn_point)


@dataclass
class Transform:
    location: Location = None
    """Location of the actor"""

    rotation: Rotation = None
    """Rotation of the actor"""

    def __post_init__(self):
        self.location = Location(*self.location)
        self.rotation = Rotation(*self.rotation)
        self._carla_trans = None

    def as_simulator_transform(self):
        if self._carla_trans is None:
            self._carla_trans = carla.Transform(
                carla.Location(*self.location),
                carla.Rotation(*self.rotation),
            )
        return self._carla_trans

    def __hash__(self) -> int:
        return hash((self.location, self.rotation))


@dataclass
class TrafficLightState:
    state: int = None
    """State of the traffic light"""

    frozen: int = None
    """Whether the traffic light is frozen"""

    elapsed_time: float = None
    """Time elapsed since the traffic light is frozen"""


@dataclass
class CollisionInfo:
    actor: int = None
    """Id of the actor that attached the collision sensor"""

    other_actor: int = None
    """Id of the other actor that collided with the collision sensor"""

    def __contains__(self, id: int) -> bool:
        return id in (self.actor, self.other_actor)


@dataclass
class Action:
    steering: float = None
    """Steering angle of the vehicle"""

    throttle: float = None
    """Throttle of the vehicle"""

    brake: float = None
    """Brake of the vehicle"""

    handbrake: bool = None
    """Handbrake of the vehicle"""

    gear: int = None
    """Gear of the vehicle"""

    speed: float = None
    """Speed of the walker"""

    type: str = None
    """Type of the actor"""

    def __post_init__(self):
        if self.speed is not None:
            self.type = "walker"
        else:
            self.type = "vehicle"
            self.handbrake = bool(self.handbrake)
            self.gear = int(self.gear)


@dataclass
class ActorFrameInfo:
    id: int = None
    """Id of the actor"""

    transform: Transform = None
    """Transform of the actor"""

    velocity: Vector3D = None
    """Velocity of the actor"""

    acceleration: Vector3D = None
    """Acceleration of the actor"""

    action: Action = None
    """Action of the actor"""


@dataclass
class Frame:
    frame_id: int = None
    """Current frame id"""

    frame_time: float = None
    """Time elapsed since the beginning of the replay file in seconds"""

    collision: list[CollisionInfo] = None
    """Collision record of the frame"""

    transform: dict[int, Transform] = None
    """Actors transform info of the frame"""

    velocity: dict[int, Vector3D] = None
    """Actors velocity info of the frame"""

    acceleration: dict[int, Vector3D] = None
    """Actors acceleration info of the frame"""

    traffic_light: dict[int, TrafficLightState] = None
    """Traffic light states of the frame"""

    action: dict[int, Action] = None
    """Actor action of the frame"""

    def __post_init__(self):
        if self.velocity is None:
            self.velocity = {}
        if self.acceleration is None:
            self.acceleration = {}

    @classmethod
    def from_dict(cls, frame_dict: dict) -> Frame:
        """Create a frame from a dictionary

        Args:
            frame_dict (dict): Dictionary of the frame, including data directly extracted from the replay file

        Note:
            velocity and acceleration are calculated from transform data, this function will not initialize them
        """

        return cls(
            frame_id=frame_dict["frame_id"],
            frame_time=frame_dict["frame_time"],
            collision=list(map(lambda colli: CollisionInfo(*colli), frame_dict["collision"])),
            transform={id: Transform(**info) for id, info in frame_dict["transform"].items()},
            traffic_light={id: TrafficLightState(**state) for id, state in frame_dict["traffic_light"].items()}, # fmt: skip
            action={id: Action(**action) for id, action in frame_dict["action"].items()},  # fmt: skip
        )

    @weak_lru(maxsize=128)
    def get_info_by_id(self, id: int, use_carla_api: bool = False) -> ActorFrameInfo:
        """Get all information of an actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            ActorFrameInfo: All information of the actor
        """
        return ActorFrameInfo(
            id=id,
            transform=self.get_transform_by_id(id, use_carla_api),
            velocity=self.get_velocity_by_id(id, use_carla_api),
            acceleration=self.get_acceleration_by_id(id, use_carla_api),
            action=self.get_action_by_id(id, use_carla_api),
        )

    @weak_lru(maxsize=128)
    def get_transform_by_id(
        self, id: int, use_carla_api: bool = False
    ) -> "Transform | carla.Transform":
        """Get transform of an actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            Transform: Transform of the actor
        """
        transform = self.transform.get(id, None)
        if transform is not None:
            return transform.as_simulator_transform() if use_carla_api else transform
        return None

    @weak_lru(maxsize=128)
    def get_velocity_by_id(
        self, id: int, use_carla_api: bool = False
    ) -> Vector3D | carla.Vector3D:
        """Get velocity of an actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            Vector3D | carla.Vector3D: Velocity of the actor
        """
        velocity = self.velocity.get(id, None)
        if velocity is not None:
            return velocity.as_simulator_vector() if use_carla_api else velocity
        return None

    @weak_lru(maxsize=128)
    def get_acceleration_by_id(
        self, id: int, use_carla_api: bool = False
    ) -> Vector3D | carla.Vector3D:
        """Get velocity of an actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            Vector3D | carla.Vector3D: Velocity of the actor
        """
        acceleration = self.acceleration.get(id, None)
        if acceleration is not None:
            return acceleration.as_simulator_vector() if use_carla_api else acceleration
        return None

    @weak_lru(maxsize=128)
    def get_action_by_id(
        self, id: int, use_carla_api: bool = False
    ) -> "Action | carla.VehicleControl | carla.WalkerControl":
        """Get action of an actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            Action | carla.VehicleControl | carla.WalkerControl: Action of the actor
        """
        act = self.action.get(id, None)
        if act is None:
            return None

        if use_carla_api:
            if act.type == "vehicle":
                return carla.VehicleControl(
                    throttle=act.throttle,
                    steer=act.steering,
                    brake=act.brake,
                    hand_brake=act.handbrake,
                    gear=act.gear,
                )
            elif act.type == "walker":
                transform = self.get_transform_by_id(id, use_carla_api=True)
                return carla.WalkerControl(
                    direction=transform.get_forward_vector(),
                    speed=act.speed,
                )
        return act

    def __hash__(self) -> int:
        return hash((self.frame_id, self.frame_time))


class Replay:
    def __init__(self, client: carla.Client, filepath: str, lazy_init: bool = True):
        """Parse one replay file

        Args:
            client (carla.Client): The client to connect to the server
            filepath (str): The path to the replay file
            lazy_init (bool, optional): Whether to delay parsing the replay file. Defaults to True.
        """
        self.client = client
        self.filepath = filepath
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")

        self._info = None
        self._duration = None
        self._map_name = None
        self._vehicles = None
        self._walkers = None
        self._sensors = None
        self._obstacles = None
        self._actors = None
        self._frame_info = None

        if not lazy_init:
            self._parse_replay_file()

    @property
    def info(self) -> str:
        """Information of the replay file"""
        if self._info is None:
            self._info = self.client.show_recorder_file_info(self.filepath, True)
        return self._info

    @property
    def duration(self) -> float:
        """Total duration of the replay file in seconds"""
        if self._duration is None:
            self._parse_duration()
        return self._duration

    @property
    def map_name(self) -> str:
        """Map name of the replay file"""
        if self._map_name is None:
            self._parse_map()
        return self._map_name

    @property
    def vehicles(self) -> list[Actor]:
        """List of vehicles in the replay file"""
        if self._vehicles is None:
            self._vehicles = self._parse_actors("vehicle")
        return self._vehicles

    @property
    def walkers(self) -> list[Actor]:
        """List of walkers in the replay file"""
        if self._walkers is None:
            self._walkers = self._parse_actors("walker")
        return self._walkers

    @property
    def sensors(self) -> list[Actor]:
        """List of sensors in the replay file"""
        if self._sensors is None:
            self._sensors = self._parse_actors("sensor")
        return self._sensors

    @property
    def obstacles(self) -> list[Actor]:
        """List of obstacles in the replay file"""
        if self._obstacles is None:
            self._obstacles = self._parse_actors("dynamic")
        return self._obstacles

    @property
    def actors(self) -> list[Actor]:
        """List of all actors in the replay file"""
        # fmt: off
        if self._actors is None:
            self._actors = sorted(self.vehicles + self.walkers + self.obstacles, key=lambda x: x.id)
            for i in range(len(self._actors)):
                current_actor = self._actors[i]
                next_actor = self._actors[i + 1] if i < len(self._actors) - 1 else None
                current_actor.sensors = [
                    sensor
                    for sensor in self.sensors
                    if (next_actor is None and sensor.id > current_actor.id)
                    or (next_actor is not None and current_actor.id < sensor.id < next_actor.id
                    )
                ]
        # fmt: on
        return self._actors

    @property
    def frame_info(self) -> list[Frame]:
        """List of all frames in the replay file"""
        if self._frame_info is None:
            self._frame_info = self._parse_frame_info()
        return self._frame_info

    def get_frame(self, frame_id: int) -> Frame:
        """Get frame by its id

        Args:
            frame_id (int): Id of the frame

        Returns:
            Frame: Frame with the given id
        """
        try:
            return self.frame_info[frame_id - 1]
        except IndexError:
            return None

    @weak_lru(maxsize=128)
    def get_actor_by_rolename(self, name: str) -> Actor:
        """Get actor by its rolename

        Args:
            name (str): Rolename of the actor

        Returns:
            Actor: Actor with the given rolename
        """
        for actor in self.actors:
            if actor.attributes.get("role_name", None) == name:
                return actor
        return None

    @weak_lru(maxsize=128)
    def get_actor_by_id(self, id: int) -> Actor:
        """Get actor by its id

        Args:
            id (int): Id of the actor

        Returns:
            Actor: Actor with the given id
        """
        for actor in self.actors:
            if actor.id == id:
                return actor
        return None

    @weak_lru(maxsize=128)
    def filter_actor_by_blueprint(self, blueprint: str) -> list[Actor]:
        """Filter actors by blueprint

        Args:
            blueprint (str): Blueprint name of the actor

        Returns:
            list[Actor]: List of actors with the given blueprint
        """
        pattern = re.compile(blueprint)
        return [actor for actor in self.actors if pattern.match(actor.blueprint)]

    @weak_lru(maxsize=128)
    def get_collision_frames(self) -> list[Frame]:
        """Get all frames with collision

        Returns:
            list[Frame]: List of frames with collision
        """
        return [frame for frame in self.frame_info if frame.collision]

    @weak_lru(maxsize=128)
    def get_actor_trajectory(self, id: int) -> list[Transform]:
        """Get the trajectory of an actor

        Args:
            id (int): Id of the actor

        Returns:
            list[Transform]: List of locations of the actor
        """
        trajectory = [frame.get_transform_by_id(id) for frame in self.frame_info]

        # Find the last unique transform before the final stopping
        last_unique_index = len(trajectory) - 1
        for i in range(len(trajectory) - 2, -1, -1):
            if trajectory[i] != trajectory[-1]:
                last_unique_index = i
                break

        # Slice the trajectory up to the last unique transform
        return trajectory[: last_unique_index + 1]

    def _parse_replay_file(self):
        """Parse all information from a replay file"""
        # -------------Duration information-------------
        self._duration = self._parse_duration()

        # -------------Map information-------------
        self._map_name = self._parse_map()

        # -------------Actor information-------------
        self._vehicles = self._parse_actors("vehicle")
        self._walkers = self._parse_actors("walker")
        self._sensors = self._parse_actors("sensor")
        self._obstacles = self._parse_actors("dynamic")

        # Attach sensor to actors
        # fmt: off
        self._actors = sorted(self._vehicles + self._walkers + self._obstacles, key=lambda x: x.id)
        for i in range(len(self._actors)):
            current_actor = self._actors[i]
            next_actor = self._actors[i + 1] if i < len(self._actors) - 1 else None
            current_actor.sensors = [
                sensor
                for sensor in self._sensors
                if (next_actor is None and sensor.id > current_actor.id)
                or (next_actor is not None and current_actor.id < sensor.id < next_actor.id
                )
            ]

        # -------------Frame information-------------
        # fmt: on
        self._frame_info = self._parse_frame_info()

    def _parse_duration(self) -> float:
        """Parse the duration of the replay file in seconds

        Returns:
            float: Duration of the replay file in seconds
        """
        split_info = [line.strip() for line in self.info.splitlines()]
        return float(split_info[-1].split(" ")[1].strip())

    def _parse_map(self) -> str:
        """Parse the map name from the replay file

        Returns:
            str: Map name
        """
        map_pattern = r"Map: ([\w.]+)"
        map_match = re.search(map_pattern, self.info)
        return map_match.group(1)

    def _parse_actors(self, actor_type: str) -> list[Actor]:
        """Parse certain type of actor information from a replay file

        Args:
            actor_type (str): The type of actor that needs to be parsed (blueprint prefix)

        Returns:
            list[Actor]: List of actors
        """
        actor_template = r"Create (\d+): ({}(?:\.[\w.]+)?) \(\d+\) at \(([^)]+)\)(?:\n((?:  \w+ = .*\n?)+))?"
        pattern = re.compile(actor_template.format(actor_type))

        actors = []
        for match in pattern.finditer(self.info):
            actor_id, blueprint, location, attributes_data = match.groups()
            attributes = {}
            if attributes_data:
                for line in attributes_data.split("\n"):
                    if not line.strip():
                        continue
                    key, value = re.match(r"  (\w+) = (.*)", line).groups()
                    attributes[key] = value

            actors.append(
                Actor(
                    id=int(actor_id),
                    blueprint=blueprint,
                    spawn_point=tuple(float(x) / 100 for x in location.split(", ")),
                    attributes=attributes,
                )
            )
        return actors

    def _parse_frame_info(self) -> list[Frame]:
        """Parse each frame's information

        Returns:
            frame_info_list (list): List of frame information
        """

        def parse_match(pattern: re.Pattern, text: str, processor=lambda x: x):
            """Apply a regular expression pattern to the text and process matches with the given processor function."""
            return [processor(match) for match in re.findall(pattern, text)]

        def process_collision(match):
            """Process collision matches."""
            return frozenset(map(int, match))

        def process_transform(match):
            """Process transform matches."""
            id_number, location, rotation = match
            if int(id_number) in focus_actor:
                location = tuple(float(x) / 100 for x in location.split(", "))
                rotation = tuple(float(x) for x in rotation.split(", "))
                return int(id_number), {"location": location, "rotation": rotation}
            return None

        def process_traffic_light(match):
            """Process traffic light matches."""
            id_number, state, frozen, elapsed_time = match
            return int(id_number), {
                "state": int(state),
                "frozen": int(frozen),
                "elapsed_time": float(elapsed_time),
            }

        def process_action(match, action_type):
            """Process action matches."""
            id_number = int(match[0])
            if id_number in focus_actor:
                if action_type == "vehicle":
                    keys = ["steering", "throttle", "brake", "handbrake", "gear"]
                    values = [float(v) for v in match[1:6]]
                    return id_number, dict(zip(keys, values))
                elif action_type == "walker":
                    return id_number, {"speed": float(match[1]) / 100}
            return None

        # fmt: off
        frame_info_list: list[Frame] = []
        focus_actor = set(actor.id for actor in self.actors)
        frames = re.split(r"(Frame \d+ at [\d\.]+ seconds)", self.info)[1:]

        # Define the patterns
        frame_info_pattern = r"Frame (\d+) at ([\d\.]+) seconds"
        collision_pattern = (r"Collision id \d+ between (\d+)(?: \(hero\))? +with +(\d+)(?: \(hero\))?")
        transform_pattern = r"Id: (\d+) Location: \((.*?)\) Rotation \((.*?)\)"
        traffic_light_pattern = r"Id: (\d+) state: (\d+) frozen: (\d+) elapsedTime: ([\d\.]+)"
        vehicle_pattern = r"Id: (\d+) Steering: ([\d\.]+) Throttle: ([\d\.]+) Brake ([\d\.]+) Handbrake: ([\d\.]+) Gear: (\d+)(?: \w+)*"
        walker_pattern = r"Id: (\d+) speed: ([\d\.]+)"

        for frame_info, frame in zip(frames[::2], frames[1::2]):
            frame_id, frame_time = re.match(frame_info_pattern, frame_info).groups()
            frame_dict = {
                "frame_id": int(frame_id),
                "frame_time": float(frame_time),
                "collision": list(map(tuple, set(parse_match(collision_pattern, frame, process_collision)))),
                "transform": dict(filter(None, parse_match(transform_pattern, frame, process_transform))),
                "traffic_light": dict(parse_match(traffic_light_pattern, frame, process_traffic_light)),
                "action": {}
            }

            frame_dict["action"].update(filter(None, parse_match(vehicle_pattern, frame, lambda m: process_action(m, "vehicle"))))
            frame_dict["action"].update(filter(None, parse_match(walker_pattern, frame, lambda m: process_action(m, "walker"))))
            current_frame = Frame.from_dict(frame_dict)

            # Calculate velocity and acceleration
            for id in current_frame.transform:
                if current_frame.frame_id == 1:
                    current_frame.velocity[id] = current_frame.acceleration[id] = Vector3D(0, 0, 0)
                else:
                    previous_frame = frame_info_list[-1]
                    time_diff = max(1e-6, current_frame.frame_time - previous_frame.frame_time)
                    d_loc = current_frame.transform[id].location - previous_frame.transform[id].location
                    current_frame.velocity[id] = d_loc / time_diff
                    d_velocity = current_frame.velocity[id] - previous_frame.velocity[id]
                    current_frame.acceleration[id] = d_velocity / time_diff
            frame_info_list.append(current_frame)

        # fmt: on
        return frame_info_list

    def __hash__(self) -> int:
        return hash(self.filepath)

    def __eq__(self, other) -> bool:
        return isinstance(other, Replay) and self.filepath == other.filepath


__all__ = ["Replay"]
