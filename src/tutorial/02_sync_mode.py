import time

import carla
import random

## Part 1

# Connect to Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

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

# Without the loop, the spectator won't follow the vehicle
while True:
    try:
        # Move the spectator behind the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
        spectator.set_transform(transform) 
        time.sleep(0.005)

        world.tick()

    except KeyboardInterrupt as e:

        vehicle.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False # Disables synchronous mode
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('Vehicles Destroyed. Bye!')
        break
