#all imports
# 代码位置：https://github.com/vadim7s/SelfDrive/blob/master/Tutorials/tutorial_2_positioning.ipynb
# 视频教程：https://www.youtube.com/watch?v=f9NGX2T6bmY
import carla

client = carla.Client('localhost', 2000)

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

# 生成消防车
vehicle_bp = world.get_blueprint_library().filter('*firetruck*')
start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)


# get the car's position on the map
vehicle_pos = vehicle.get_transform()
print(vehicle_pos)

# initial spawn point is the same - just 0.6m higher off the ground
print(start_point)

#send vehicle off
vehicle.set_autopilot(True)

# get actual position from the car moving
vehicle_pos = vehicle.get_transform()
print(vehicle_pos)

# now look at the map
town_map = world.get_map()

type(town_map)

print(town_map)

roads = town_map.get_topology()

# topology is pairs of waypoints defining all roads - tuples of those
print(roads)

#have a look at a waypoint - it is transform wrapped as a waypoint
print(roads[0][0])

len(roads)


# import some code coming with the sim
import sys
sys.path.append('D:/work/workspace/carla_doc/src/')
from carla_agent.navigation.global_route_planner import GlobalRoutePlanner


# utility script of destruction

for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()

# now we define 2 cars
truck_bp = world.get_blueprint_library().filter('*firetruck*')
mini_bp = world.get_blueprint_library().filter('*cooper_s*')

#start first car in alredy defined start point
truck = world.try_spawn_actor(truck_bp[0], start_point)

# tweak spectator position to watch the show

spectator = world.get_spectator()
spawn_points = world.get_map().get_spawn_points()
start_point = spawn_points[0]

spectator_pos = carla.Transform(start_point.location + carla.Location(x=20,y=10,z=4),
                                carla.Rotation(yaw = start_point.rotation.yaw -155))

spectator.set_transform(spectator_pos)

# drop the Mini the sky - watch what happens after

#spawn it first somewhere else
mini = world.try_spawn_actor(mini_bp[0], spawn_points[10])

mini_pos = carla.Transform(start_point.location + carla.Location(x=-4,z=10),
                            carla.Rotation(yaw = start_point.rotation.yaw - 0))
mini.set_transform(mini_pos)


import time
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()
# now we define 2 cars
truck_bp = world.get_blueprint_library().filter('*firetruck*')
mini_bp = world.get_blueprint_library().filter('*cooper_s*')

#start first car in alredy defined start point
truck = world.try_spawn_actor(truck_bp[0], start_point)
time.sleep(3)

vw_bp = world.get_blueprint_library().filter('*volkswagen*')

vw = world.try_spawn_actor(vw_bp[0], spawn_points[20])
vw_pos = carla.Transform(start_point.location + carla.Location(x=1,z=6),
                            carla.Rotation(yaw = start_point.rotation.yaw - 0))
vw.set_transform(vw_pos)
mini = world.try_spawn_actor(mini_bp[0], spawn_points[10])

mini_pos = carla.Transform(start_point.location + carla.Location(x=1,z=10),
                            carla.Rotation(yaw = start_point.rotation.yaw - 0))
mini.set_transform(mini_pos)


vw_bp = world.get_blueprint_library().filter('*volkswagen*')


vehicle_all = world.get_blueprint_library()
print(vehicle_all)