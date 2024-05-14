import argparse
import glob
import os
import queue
import sys

import numpy as np
import pygame

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../"))

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

from local_carla_api import BoundingBox, Transform
from carla_world import CarlaWorld

BB_COLOR = (248, 64, 24)


def get_camera_pos(actor: carla.Actor, view_from: str):
    """Get camera attach point for given actor.

    Args:
        actor (carla.Actor): Actor to attach camera to.
        view_from (str): Camera position relative to actor.

    Returns:
        carla.Transform: Camera attach point.
        carla.AttachmentType: Camera attachment type.
    """

    attachment_type = carla.AttachmentType.Rigid
    if view_from == "behind":
        camera_pos = carla.Transform(
            carla.Location(
                x=-2.8 * actor.bounding_box.extent.x,
                y=0.0,
                z=4.6 * actor.bounding_box.extent.z,
            ),
            carla.Rotation(pitch=6.0),
        )
        attachment_type = carla.AttachmentType.SpringArm
    elif view_from == "above":
        camera_pos = carla.Transform(
            carla.Location(
                x=0.0,
                y=0.0,
                z=50.0,
            ),
            carla.Rotation(pitch=-90.0),
        )
    else:
        camera_pos = carla.Transform()

    return camera_pos, attachment_type


def get_bb_camera_cords(
    carla_world: CarlaWorld, actor: carla.Actor, camera: carla.Sensor
):
    bb = BoundingBox.from_simulator_bounding_box(actor.bounding_box)
    actor_trans = Transform.from_simulator_transform(actor.get_transform())
    # The bounding box is relative to the center of the vehicle
    # So we either calculate the transform or adjust the location center
    actor_trans.location += bb.location

    # Projection to camera
    bb_world_cords = actor_trans.transform_points(bb.as_numpy_array(False))
    bb_camera_cords = carla_world.proj_to_camera(bb_world_cords, camera)
    return bb_camera_cords


def render(display: pygame.Surface, image: carla.Image):
    """
    Transforms image from camera sensor and blits it to main pygame display.
    """
    if image is not None:
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        display.blit(surface, (0, 0))


def draw_bounding_boxes(display: pygame.Surface, bounding_boxes: "list[np.ndarray]"):
    """
    Draws bounding boxes on pygame display.
    """
    bb_surface = pygame.Surface(display.get_size())
    bb_surface.set_colorkey((0, 0, 0))
    for bbox in bounding_boxes:
        try:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
        except:
            continue

        # draw lines
        # base
        pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
        pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
        pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
        pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
        pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
        # top
        pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
        pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
        pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
        pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
        # base-top
        pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
        pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
        pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
        pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
    display.blit(bb_surface, (0, 0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--rolename", type=str, default="hero")
    parser.add_argument("--view", type=str, default="behind")
    parser.add_argument("--distance", type=float, default=100)
    parser.add_argument("--res", type=str, default="800x600")
    parser.add_argument("--fov", type=int, default=90)
    args = parser.parse_args()

    x_res, y_res = [int(x) for x in args.res.split("x")]

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    carla_world = CarlaWorld(client)

    hero_actor = carla_world.find_actor_by_name(args.rolename)
    if hero_actor is None:
        print(f"Actor {args.rolename} not found")
        sys.exit(-1)

    camera = carla_world.spawn_sensor(
        "sensor.camera.rgb",
        hero_actor,
        *get_camera_pos(hero_actor, args.view),
        image_size_x=x_res,
        image_size_y=y_res,
        fov=args.fov,
    )
    image_queue = queue.Queue()
    camera.listen(lambda image: image_queue.put(image))

    pygame.display.set_caption("Sensor Bounding Box")
    display = pygame.display.set_mode(
        (x_res, y_res), pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    clock = pygame.time.Clock()

    try:
        while True:
            clock.tick(60)

            camera_bboxes = []
            hero_trans = hero_actor.get_transform()
            for actor in carla_world.get_actors():
                actor_trans = actor.get_transform()
                if hero_trans.location.distance(actor_trans.location) > args.distance:
                    continue
                if (
                    args.view != "above"
                    and CarlaWorld.relative_position(hero_trans, actor_trans) == "rear"
                ):
                    continue
                camera_bboxes.append(get_bb_camera_cords(carla_world, actor, camera))

            render(display, image_queue.get())
            draw_bounding_boxes(display, camera_bboxes)
            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
    finally:
        carla_world.cleanup()
