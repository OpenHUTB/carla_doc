import carla
import random

# 连接到客户端并获取世界对象
client = carla.Client('localhost', 2000)
world = client.get_world()

# 以同步模式设置仿真器
settings = world.get_settings()
settings.synchronous_mode = True  # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 以同步模式设置交通管理器
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# 设置种子，以便必要时行为可以重复
traffic_manager.set_random_device_seed(0)
random.seed(0)

# 我们还将设置观察者，这样我们就可以看到我们在做什么
spectator = world.get_spectator()

spawn_points = world.get_map().get_spawn_points()

# 将观察者移动到俯视图
transform = carla.Transform(spawn_points[0].transform(carla.Location(x=-4, z=50)), carla.Rotation(yaw=-180, pitch=-90))
spectator.set_transform(transform) 

# 在地图上用数字标出生成点的位置
for i, spawn_point in enumerate(spawn_points):
    world.debug.draw_string(spawn_point.location, str(i), life_time=60)

# 从蓝图库中选择一些模型
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = []
for vehicle in world.get_blueprint_library().filter('*vehicle*'):
    if any(model in vehicle.id for model in models):
        blueprints.append(vehicle)

# 设置车辆的最大数量，并为我们生成的车辆准备一个列表
max_vehicles = 20
max_vehicles = min([max_vehicles, len(spawn_points)])
vehicles = []

# 随机抽取一些生成点并生成一些车辆
for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
    temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
    if temp is not None:
        vehicles.append(temp)

# 解析生成的车辆列表，并通过 set_autopilot() 将控制权交给交通管理器
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    # 随机设置车辆无视交通灯的概率
    traffic_manager.ignore_lights_percentage(vehicle, random.randint(0, 50))

spawn_points = world.get_map().get_spawn_points()

# 路线 1
spawn_point_1 = spawn_points[32]

# 从选择的生成点创建路线1
route_1_indices = [129, 28, 124, 33, 97, 119, 58, 154, 147]
route_1 = []
for ind in route_1_indices:
    route_1.append(spawn_points[ind].location)

# 路线 2
spawn_point_2 = spawn_points[149]

# 从选择的生成点中创建路线 2
route_2_indices = [21, 76, 38, 34, 90, 3]
route_2 = []
for ind in route_2_indices:
    route_2.append(spawn_points[ind].location)

# 现在我们把它们打印到地图上，这样我们就可以看到我们的路线了
world.debug.draw_string(spawn_point_1.location, 'Spawn point 1', life_time=30, color=carla.Color(255,0,0))
world.debug.draw_string(spawn_point_2.location, 'Spawn point 2', life_time=30, color=carla.Color(0,0,255))

for ind in route_1_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(255,0,0))

for ind in route_2_indices:
    spawn_points[ind].location
    world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(0,0,255))

# 设置延迟来创建刷出时间之间的间隔
spawn_delay = 20
counter = spawn_delay

# 设置最大车辆数(低硬件配置设置较小值)
max_vehicles = 200

# 在生成点之间交替
alt = False

while True:
    try:
        world.tick()

        n_vehicles = len(world.get_actors().filter('*vehicle*'))
        vehicle_bp = random.choice(blueprints)

        # 只能在延迟后生成车辆
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
