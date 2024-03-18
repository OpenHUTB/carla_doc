import carla
import random

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# Set a seed so behaviour can be repeated if necessary
traffic_manager.set_random_device_seed(0)
random.seed(0)

# We will aslo set up the spectator so we can see what we do
spectator = world.get_spectator()

spawn_points = world.get_map().get_spawn_points()

# Move the spectator to the top view
transform = carla.Transform(spawn_points[0].transform(carla.Location(x=-4,z=50)), carla.Rotation(yaw=-180, pitch=-90)) 
spectator.set_transform(transform) 

# Draw the spawn point locations as numbers in the map
for i, spawn_point in enumerate(spawn_points):
    world.debug.draw_string(spawn_point.location, str(i), life_time=60)

# Select some models from the blueprint library
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = []
for vehicle in world.get_blueprint_library().filter('*vehicle*'):
    if any(model in vehicle.id for model in models):
        blueprints.append(vehicle)

# Set a max number of vehicles and prepare a list for those we spawn
max_vehicles = 20
max_vehicles = min([max_vehicles, len(spawn_points)])
vehicles = []

# Take a random sample of the spawn points and spawn some vehicles
for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
    temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
    if temp is not None:
        vehicles.append(temp)

# Parse the list of spawned vehicles and give control to the TM through set_autopilot()
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    # Randomly set the probability that a vehicle will ignore traffic lights
    traffic_manager.ignore_lights_percentage(vehicle, random.randint(0,50))

spawn_points = world.get_map().get_spawn_points()

# Route 1
spawn_point_1 =  spawn_points[32]

# Create route 1 from the chosen spawn points
route_1_indices = [129, 28, 124, 33, 97, 119, 58, 154, 147]
route_1 = []
for ind in route_1_indices:
    route_1.append(spawn_points[ind].location)

# Route 2
spawn_point_2 =  spawn_points[149]

# Create route 2 from the chosen spawn points
route_2_indices = [21, 76, 38, 34, 90, 3]
route_2 = []
for ind in route_2_indices:
    route_2.append(spawn_points[ind].location)

# Now let's print them in the map so we can see our routes
world.debug.draw_string(spawn_point_1.location, 'Spawn point 1', life_time=30, color=carla.Color(255,0,0))
world.debug.draw_string(spawn_point_2.location, 'Spawn point 2', life_time=30, color=carla.Color(0,0,255))

for ind in route_1_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(255,0,0))

for ind in route_2_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(0,0,255))

# Set delay to create gap between spawn times
spawn_delay = 20
counter = spawn_delay

# Set max vehicles (set smaller for low hardward spec)
max_vehicles = 200

# Alternate between spawn points
alt = False

while True:
    try:
        world.tick()

        n_vehicles = len(world.get_actors().filter('*vehicle*'))
        vehicle_bp = random.choice(blueprints)

        # Spawn vehicle only after delay
        if counter == spawn_delay and n_vehicles < max_vehicles:
            # Alternate spawn points
            if alt:
                vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_1)
            else:
                vehicle = world.try_spawn_actor(vehicle_bp, spawn_point_2)

            if vehicle: # IF vehicle is succesfully spawned
                vehicle.set_autopilot(True) # Give TM control over vehicle

                # Set parameters of TM vehicle control, we don't want lane changes
                traffic_manager.update_vehicle_lights(vehicle, True)
                traffic_manager.random_left_lanechange_percentage(vehicle, 0)
                traffic_manager.random_right_lanechange_percentage(vehicle, 0)
                traffic_manager.auto_lane_change(vehicle, False)

                # Alternate between routes
                if alt:
                    traffic_manager.set_path(vehicle, route_1)
                    alt = False
                else:
                    traffic_manager.set_path(vehicle, route_2)
                    alt = True

                vehicle = None

            counter -= 1
        elif counter > 0:
            counter -= 1
        elif counter == 0:
            counter = spawn_delay

    except KeyboardInterrupt as e:

        settings = world.get_settings()
        settings.synchronous_mode = False # Disables synchronous mode
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        for vehicle in vehicles:
            if vehicle:
                vehicle.destroy()

        print("Vehicles Destroyed.")
        break
