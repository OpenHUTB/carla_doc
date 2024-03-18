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

# Print all camera types
for bp in bp_lib.filter("camera"):
    print(bp.id)

# Create a camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))

# Create a RGB camera
rgb_camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
rgb_camera = world.spawn_actor(rgb_camera_bp, camera_init_trans, attach_to=vehicle)

# Create a semantic segmentation camera
seg_camera_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
seg_camera = world.spawn_actor(seg_camera_bp, camera_init_trans, attach_to=vehicle)

# Create a instance segmentation camera
ins_camera_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
ins_camera = world.spawn_actor(ins_camera_bp, camera_init_trans, attach_to=vehicle)

# Create a depth camera
depth_camera_bp = world.get_blueprint_library().find('sensor.camera.depth')
depth_camera = world.spawn_actor(depth_camera_bp, camera_init_trans, attach_to=vehicle)

# Create a DVS camera
dvs_camera_bp = world.get_blueprint_library().find('sensor.camera.dvs')
dvs_camera = world.spawn_actor(dvs_camera_bp, camera_init_trans, attach_to=vehicle)

# Create an optical flow camera
opt_camera_bp = world.get_blueprint_library().find('sensor.camera.optical_flow')
opt_camera = world.spawn_actor(opt_camera_bp, camera_init_trans, attach_to=vehicle)

# Define camera callbacks                       
def rgb_camera_callback(image, rgb_image_queue):
    rgb_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

def seg_camera_callback(image, seg_image_queue):
    image.convert(carla.ColorConverter.CityScapesPalette)
    seg_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

def ins_camera_callback(image, ins_image_queue):
    ins_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

def depth_camera_callback(image, depth_image_queue):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    depth_image_queue.put(np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)))

def dvs_camera_callback(data, dvs_image_queue):
    dvs_events = np.frombuffer(data.raw_data, dtype=np.dtype([
        ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool_)
    ]))

    dvs_img = np.zeros((data.height, data.width, 4), dtype=np.uint8)

    # Blue is positive, red is negative
    dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255

    dvs_image_queue.put(dvs_img)

def opt_camera_callback(data, opt_image_queue):
    image = data.get_color_coded_flow()
    height, width = image.height, image.width
    image = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    image = np.reshape(image, (height, width, 4))

    opt_image_queue.put(image)

# Get gamera dimensions and initialise dictionary                       
image_w = rgb_camera_bp.get_attribute("image_size_x").as_int()
image_h = rgb_camera_bp.get_attribute("image_size_y").as_int()

rgb_image_queue = queue.Queue()
seg_image_queue = queue.Queue()
ins_image_queue = queue.Queue()
depth_image_queue = queue.Queue()
dvs_image_queue = queue.Queue()
opt_image_queue = queue.Queue()

# Start camera recording
rgb_camera.listen(lambda image: rgb_camera_callback(image, rgb_image_queue))
seg_camera.listen(lambda image: seg_camera_callback(image, seg_image_queue))
ins_camera.listen(lambda image: ins_camera_callback(image, ins_image_queue))
depth_camera.listen(lambda image: depth_camera_callback(image, depth_image_queue))
dvs_camera.listen(lambda image: dvs_camera_callback(image, dvs_image_queue))
opt_camera.listen(lambda image: opt_camera_callback(image, opt_image_queue))

cv2.namedWindow('All Cameras', cv2.WINDOW_NORMAL)

# Clear the spawned vehicle and camera
def clear():

    rgb_camera.stop()
    seg_camera.stop()
    ins_camera.stop()
    depth_camera.stop()
    dvs_camera.stop()
    opt_camera.stop()
    print('\nCameras Stopped.')
    
    vehicle.destroy()
    print('Vehicle Destroyed. Bye!')

    cv2.destroyAllWindows()

# Main loop
while True:
    try:

        # Imshow renders sensor data to display
        top_row = np.hstack((rgb_image_queue.get(), seg_image_queue.get(), ins_image_queue.get()))
        lower_row = np.hstack((depth_image_queue.get(), dvs_image_queue.get(), opt_image_queue.get()))
        tiled = np.vstack((top_row, lower_row))

        cv2.imshow('All Cameras', tiled)

        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Quit if user presses 'q'
        if cv2.waitKey(1) == ord('q'):
            clear()
            break
        
    except KeyboardInterrupt as e:
        clear()
        break
