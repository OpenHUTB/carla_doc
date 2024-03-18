import carla

import random
import queue

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

# Get gamera dimensions and initialise dictionary                       
image_w = rgb_camera_bp.get_attribute("image_size_x").as_int()
image_h = rgb_camera_bp.get_attribute("image_size_y").as_int()

# Start camera recording
rgb_image_queue = queue.Queue()
camera.listen(lambda image: camera_callback(image, rgb_image_queue))

# OpenCV named window for rendering
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

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
while True:
    try:
        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Display RGB camera image
        cv2.imshow('RGB Camera', rgb_image_queue.get())

        # Quit if user presses 'q'
        if cv2.waitKey(1) == ord('q'):
            clear()
            break

    except KeyboardInterrupt as e:
        clear()
        break
