import carla
from agents.navigation.basic_agent import BasicAgent

import math
import random

import numpy as np

from collections import deque
import matplotlib.pyplot as plt

## Part 1

# Connect to Carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# Get a vehicle from the library
bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()

# Navigation
start_point = spawn_points[0]
stop_point = spawn_points[-1]

# Draw the start and stop point
world.debug.draw_string(start_point.location, 'Start', life_time=60)
world.debug.draw_string(stop_point.location, 'Stop', life_time=60)

# Spawn a vehicle
vehicle = world.try_spawn_actor(vehicle_bp, start_point)

# Start a basic agent
agent = BasicAgent(vehicle)

# Set destination
destination = stop_point.location
agent.set_destination(destination)

# Get the world spectator 
spectator = world.get_spectator() 

# Get the map
m = world.get_map()

## Part 2

velocities = deque(maxlen=40)

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
    world.debug.draw_arrow(begin, end, arrow_size=0.5, life_time=60.0)

# Main loop
count = 0
while True:
    try:
        # Move the spectator to the top of the vehicle 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=30)), carla.Rotation(yaw=-180, pitch=-90)) 
        spectator.set_transform(transform) 

        # Get the vehicle's current velocity
        v = vehicle.get_velocity()
        lon_vel = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        # Plot velocities
        velocities.append(lon_vel)

        plt.clf()

        plt.plot(velocities)
        plt.scatter(range(len(velocities)), velocities)
        plt.ylim(0, 60)
        plt.ylabel('velocity (km/h)')
        plt.draw()

        plt.pause(0.01)

        # Apply navigation control
        vehicle.apply_control(agent.run_step())

        if agent.done():
            print("The target has been reached, stopping the simulation")
            vehicle.destroy()
            print('Vehicle Destroyed.')
            break

        # Draw the closest waypoints
        m.generate_waypoints(10)
        w = m.get_waypoint(vehicle.get_location())

        if count % 10 == 0:
            draw_waypoints(world, w)

        count = count + 1

    except KeyboardInterrupt as e:
        vehicle.destroy()
        print('Vehicle Destroyed.')
        break
