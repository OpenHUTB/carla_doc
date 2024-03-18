import carla

import math
import queue
import random

import cv2
import numpy as np

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

# Create a camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))

# Create a RGB camera
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(rgb_camera_bp, camera_init_trans, attach_to=vehicle)

# Callback stores sensor data in a dictionary for use outside callback                         
def camera_callback(image, rgb_image_queue):
    rgb_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

# Start camera recording
rgb_image_queue = queue.Queue()
camera.listen(lambda image: camera_callback(image, rgb_image_queue))

# OpenCV named window for rendering
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

# Get the map
m = world.get_map()

def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    # for wpt in waypoints:
    wpt = waypoints
    wpt_t = wpt.transform
    begin = wpt_t.location + carla.Location(z=z)
    angle = math.radians(wpt_t.rotation.yaw)
    end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
    world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=2.0)

# Clear the spawned vehicle and camera
def clear():

    vehicle.destroy()
    print('Vehicle Destroyed.')

    camera.stop()
    camera.destroy()
    print('Camera Destroyed. Bye!')

    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()

    cv2.destroyAllWindows()

# Main loop
count = 0
while True:
    try:
        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=20)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Display RGB camera image
        cv2.imshow('RGB Camera', rgb_image_queue.get())

        # Quit if user presses 'q'
        if cv2.waitKey(1) == ord('q'):
            clear()
            break

        # Draw the closest waypoints
        m.generate_waypoints(10)
        w = m.get_waypoint(vehicle.get_location())

        if count % 20 == 0:
            draw_waypoints(world, w)

        count = count + 1

    except KeyboardInterrupt as e:
        clear()
        break
