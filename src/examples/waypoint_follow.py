from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

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

from carla_agent.navigation.local_planner import LocalPlanner, RoadOption
from carla_agent.tools.misc import draw_waypoints
from carla_world import CarlaWorld
from local_carla_api import Waypoint


class WaypointFollower:
    def __init__(
        self,
        actor: carla.Actor,
        target_speed: float = None,
        plan: "list[tuple[carla.Waypoint, RoadOption]]" = None,
    ):
        """This is an atomic behavior to follow waypoints while maintaining a given speed.

        If no plan is provided, the actor will follow its foward waypoints indefinetely.
        Otherwise, the behavior will end with SUCCESS upon reaching the end of the plan.
        If no target velocity is provided, the actor will try to follow the speed limit.

        Args:
            actor (carla.Actor): Actor to apply the behavior.
            target_speed (float, optional): Target speed in m/s. Defaults to None, meaning the speed limit of the road.
            plan (List[carla.Waypoint] | List[carla.Location] | List[Tuple[carla.Location, RoadOption]]): Plan to follow. Defaults to None.
            duration (int, optional): Duration of the behavior in ticks. Defaults to math.inf
        """
        self._actor = actor
        self._target_speed = target_speed
        self._plan = plan

        self._local_planner = None
        self._args_lateral_dict = {"K_P": 1.0, "K_D": 0.01, "K_I": 0.0, "dt": 0.05}

        self.succeed = False
        self._apply_local_planner()

    def _apply_local_planner(self):
        if self._target_speed is None:
            self._target_speed = self._actor.get_speed_limit()

        if isinstance(self._actor, carla.Walker):
            if self._plan is not None:
                if isinstance(self._plan[0], carla.Location):
                    self._plan = self._plan
                elif isinstance(self._plan[0], carla.Waypoint):
                    self._plan = list(
                        map(lambda waypoint: waypoint.transform.location, self._plan)
                    )
                elif isinstance(self._plan[0], list):
                    self._plan = list(
                        map(lambda point: carla.Location(*point), self._plan)
                    )
            self._local_planner = None
        else:
            self._local_planner = LocalPlanner(
                self._actor,
                opt_dict={
                    "target_speed": self._target_speed * 3.6,
                    "lateral_control_dict": self._args_lateral_dict,
                    "max_throttle": 1.0,
                },
            )

            if self._plan is not None:
                if isinstance(self._plan[0], carla.Waypoint):
                    self._plan = [
                        (waypoint, RoadOption.LANEFOLLOW) for waypoint in self._plan
                    ]
                elif isinstance(self._plan[0], carla.Location):
                    pseudo_rotation = carla.Rotation()
                    self._plan = list(
                        map(
                            lambda location: (
                                Waypoint(
                                    transform=carla.Transform(location, pseudo_rotation)
                                ),
                                RoadOption.LANEFOLLOW,
                            ),
                            self._plan,
                        )
                    )
                elif isinstance(self._plan[0], list):
                    pseudo_rotation = carla.Rotation()
                    self._plan = list(
                        map(
                            lambda point: (
                                Waypoint(
                                    transform=carla.Transform(
                                        carla.Location(*point), pseudo_rotation
                                    )
                                ),
                                RoadOption.LANEFOLLOW,
                            ),
                            self._plan,
                        )
                    )
                self._local_planner.set_global_plan(self._plan)

    def get_local_plan(self, only_waypoints: bool = True):
        if self._local_planner:
            plan = self._local_planner.get_plan()
            if only_waypoints:
                return list(map(lambda waypoint: waypoint[0], plan))
            else:
                return plan

        return None

    def set_target_speed(self, target_speed: float):
        """set a new target speed for the behavior

        Args:
            target_speed (float): speed in m/s
        """
        self._target_speed = target_speed
        if not isinstance(self._actor, carla.Walker):
            self._local_planner.set_speed(target_speed * 3.6)

    def run_step(self) -> "carla.VehicleControl | carla.WalkerControl":
        """Apply one step of the behavior.

        Returns:
            carla.VehicleControl | carla.WalkerControl: The control to apply to the actor
        """
        control = None
        if not self.succeed and self._actor.is_active:
            if not isinstance(self._actor, carla.Walker):
                control = self._local_planner.run_step()
                if self._local_planner.done():
                    self.succeed = True
            else:
                actor_location = self._actor.get_location()
                if self._plan:
                    location = self._plan[0]
                    direction = location - actor_location
                    direction_norm = math.sqrt(direction.x**2 + direction.y**2)
                    control = carla.WalkerControl(
                        direction=direction / direction_norm,
                        speed=self._target_speed,
                    )

                    if direction_norm < 1.0:
                        self._plan = self._plan[1:]
                        if not self._plan:
                            self.succeed = True

        if control is None:
            control = type(self._actor.get_control())()

        return control


def parse_waypoint(carla_world: CarlaWorld, point: "dict | list"):
    """Convert the configuration point to a carla.Location

    Args:
        carla_world (CarlaWorld): The world to get the map
        point (dict | list): The point to convert

    Returns:
        carla.Location: A carla location
    """

    if isinstance(point, dict):
        if "x" in point and "y" in point:
            location = carla.Location(point["x"], point["y"], point.get("z", 1.0))
            return location
        elif "road_id" in point and "lane_id" in point:
            waypoint = carla_world.map.get_waypoint_xodr(
                point["road_id"], point["lane_id"], point.get("s", 0.0)
            )
            return waypoint.transform.location
    else:
        return carla.Location(*point)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="Waypoint follower")
    argparser.add_argument("--host", type=str, default="localhost")
    argparser.add_argument("--port", type=int, default=2000)
    argparser.add_argument("--map", type=str, default=None)
    argparser.add_argument("--rolename", type=str, default="hero")
    argparser.add_argument("--speed", type=int, default=10)
    argparser.add_argument("--json_file", type=str, default=None)
    argparser.add_argument("--waypoints", type=str, default=None)
    argparser.add_argument("--sync", action="store_true")
    argparser.add_argument("--fixed_delta_seconds", type=float, default=None)
    argparser.add_argument("--camera_follow", action="store_true")
    argparser.add_argument("--spawn", action="store_true")
    argparser.add_argument("--visualize", action="store_true")
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    carla_world = CarlaWorld(client)
    if args.map is not None:
        if args.map not in carla_world.map.name or args.spawn:
            carla_world.load_world(args.map)

    try:
        waypoints = None
        if args.json_file:
            with open(args.json_file, "r") as f:
                waypoints = json.load(f)

            waypoints = list(
                map(lambda point: parse_waypoint(carla_world, point), waypoints)
            )
        elif args.waypoints:
            waypoints = json.loads(args.waypoints)
            waypoints = list(
                map(lambda point: parse_waypoint(carla_world, point), waypoints)
            )
        else:
            raise ValueError("No waypoints provided")

        if args.sync:
            carla_world.toggle_sync_mode(args.sync, args.fixed_delta_seconds)

        if args.spawn:
            spawn_point = carla_world.map.get_waypoint(waypoints[0])
            if spawn_point is None:
                spawn_point = carla.Transform(waypoints[0])
            else:
                spawn_point = spawn_point.transform
            spawn_point.location.z += 1.0
            hero = carla_world.spawn_actor(
                args.rolename, "vehicle.tesla.model3", spawn_point
            )
            carla_world.tick()
        else:
            while True:
                hero = carla_world.find_actor_by_name(args.rolename)
                if hero is not None:
                    break

                print("Waiting for hero to spawn...")
                carla_world.tick()

        if args.camera_follow:
            carla_world.follow_actor_view(hero, "behind")

        follower = WaypointFollower(hero, args.speed, waypoints)
        while not follower.succeed:
            control = follower.run_step()
            hero.apply_control(control)
            if args.visualize:
                draw_waypoints(carla_world.world, follower.get_local_plan())
            carla_world.tick()
    finally:
        if args.camera_follow:
            carla_world.stop_follow_actor_view()
        carla_world.toggle_sync_mode(False)
        carla_world.cleanup()
