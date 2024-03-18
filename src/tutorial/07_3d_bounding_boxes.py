import carla

import queue
import random

import cv2
import numpy as np

from utils.projection import *

# Part 1

client = carla.Client('localhost', 2000)
world  = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Get the world spectator
spectator = world.get_spectator() 

# Get the map spawn points
spawn_points = world.get_map().get_spawn_points()

# spawn vehicle
bp_lib = world.get_blueprint_library()
vehicle_bp =bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Create a queue to store and retrieve the sensor data
image_queue = queue.Queue()
camera.listen(image_queue.put)

# Part 2

# Remember the edge pairs
edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

# Get the world to camera matrix
world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

# Get the attributes from the camera
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()

# Calculate the camera projection matrix to project from 3D -> 2D
K   = build_projection_matrix(image_w, image_h, fov)
K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

for i in range(20):
    vehicle_bp = bp_lib.filter('vehicle')

    # Exclude bicycle
    car_bp = [bp for bp in vehicle_bp if int(bp.get_attribute('number_of_wheels')) == 4]
    npc = world.try_spawn_actor(random.choice(car_bp), random.choice(spawn_points))

    if npc:
        npc.set_autopilot(True)

# Retrieve all the objects of the level
car_objects = world.get_environment_objects(carla.CityObjectLabel.Car) # doesn't have filter by type yet
truck_objects = world.get_environment_objects(carla.CityObjectLabel.Truck) # doesn't have filter by type yet
bus_objects = world.get_environment_objects(carla.CityObjectLabel.Bus) # doesn't have filter by type yet

env_object_ids = []

for obj in (car_objects + truck_objects + bus_objects):
    env_object_ids.append(obj.id)

# Disable all static vehicles
world.enable_environment_objects(env_object_ids, False) 

def clear():
    settings = world.get_settings()
    settings.synchronous_mode = False # Disables synchronous mode
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

    camera.stop()

    for npc in world.get_actors().filter('*vehicle*'):
        if npc:
            npc.destroy()

    print("Vehicles Destroyed.")

# Main Loop
vehicle.set_autopilot(True)

while True:
    try:
        world.tick()

        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Retrieve and reshape the image
        image = image_queue.get()

        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

        # Get the camera matrix 
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

        for npc in world.get_actors().filter('*vehicle*'):

            # Filter out the ego vehicle
            if npc.is_alive and npc.id != vehicle.id:

                npc_transform = npc.transform if isinstance(npc, carla.EnvironmentObject) else npc.get_transform()

                bb = npc.bounding_box
                dist = npc_transform.location.distance(vehicle.get_transform().location)

                # Filter for the vehicles within 50m
                if dist < 50:
                    # Calculate the dot product between the forward vector
                    # of the vehicle and the vector between the vehicle
                    # and the other vehicle. We threshold this dot product
                    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                    vehicle_forward_vec = vehicle.get_transform().get_forward_vector()
                    ray = npc_transform.location - vehicle.get_transform().location

                    if vehicle_forward_vec.dot(ray) > 0:
                        verts = [v for v in bb.get_world_vertices(npc_transform)]

                        points_image = []

                        for vert in verts:
                            ray0 = vert - camera.get_transform().location
                            cam_forward_vec = camera.get_transform().get_forward_vector()

                            if (cam_forward_vec.dot(ray0) > 0):
                                p = get_image_point(vert, K, world_2_camera)
                            else:
                                p = get_image_point(vert, K_b, world_2_camera)

                            points_image.append(p)

                        for edge in edges:
                            p1 = points_image[edge[0]]
                            p2 = points_image[edge[1]]

                            p1_in_canvas = point_in_canvas(p1, image_h, image_w)
                            p2_in_canvas = point_in_canvas(p2, image_h, image_w)

                            if not p1_in_canvas and not p2_in_canvas:
                                continue

                            cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        

        cv2.imshow('3D Bounding Boxes',img)

        if cv2.waitKey(1) == ord('q'):
            clear()
            break

    except KeyboardInterrupt as e:
        clear()
        break

cv2.destroyAllWindows()
