# Pygame 用于车辆控制

[__PyGame__](https://www.pygame.org/news) 是一组跨平台的 Python 模块，可用于编写视频游戏。它提供了一种渲染 Carla 实时视觉输出的有用方法，以便监视传感器输出，例如摄像机。PyGame 还可以捕获键盘事件，因此它是控制车辆等参与者的好方法。

在本教程中，我们将学习如何设置一个简单的 PyGame 界面，该界面使我们能够监控由交通管理器控制的地图周围的自动驾驶交通，然后使用键盘手动控制任何车辆。

## 设置模拟器并初始化交通管理器

首先，我们将初始化交通管理器并创建一些随机分布在城市周围的流量。

```py
import carla
import random
import pygame
import numpy as np

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

```

## 生成车辆

我们想要创建一个在整个城市中产生的车辆集合，并让交通管理器控制它们。

```py
# Retrieve the map's spawn points
spawn_points = world.get_map().get_spawn_points()

# Select some models from the blueprint library
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = []
for vehicle in world.get_blueprint_library().filter('*vehicle*'):
    if any(model in vehicle.id for model in models):
        blueprints.append(vehicle)

# Set a max number of vehicles and prepare a list for those we spawn
max_vehicles = 50
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


```

## 使用 PyGame 渲染相机输出并控制车辆

现在我们有一个交通拥挤的城市，我们可以设置一个摄像头来跟踪其中一辆车，并设置一个控制界面来通过键盘输入接管它。

首先，我们需要为 `camera.listen(...)` 定义一个回调函数，用于将像素数据渲染到 PyGame 接口。PyGame 将数据渲染到表面，然后渲染到屏幕，因此在回调中，我们填充存储在传递给回调函数的对象中的表面。

```py
# 渲染对象以保持并传递 PyGame 表面
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        # 创建与阵列上的数据和格式最相似的新曲面
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# 摄像头传感器回调，将摄像头的原始数据重塑为二维 RGB，并应用于 PyGame曲面
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))
```

现在我们将创建一个对象来处理控制逻辑。这通常需要进行一些调整才能满足特定需求，但这概述了一个基本界面。当控制车辆时，该控制界面允许通过任何标准键盘上的箭头键进行控制。向前箭头加速，向后箭头制动，左右箭头转动车辆。如果在车辆静止或停止时按住向下箭头，它将接合倒档并开始向后移动。

```py

# Control object to manage vehicle controls
class ControlObject(object):
    def __init__(self, veh):
        
        # Conrol parameters to store the control state
        self._vehicle = veh
        self._steer = 0
        self._throttle = False
        self._brake = False
        self._steer = None
        self._steer_cache = 0
        # A carla.VehicleControl object is needed to alter the 
        # vehicle's control state
        self._control = carla.VehicleControl()
    
    # Check for key press events in the PyGame window
    # and define the control state
    def parse_control(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self._vehicle.set_autopilot(False)
            if event.key == pygame.K_UP:
                self._throttle = True
            if event.key == pygame.K_DOWN:
                self._brake = True
            if event.key == pygame.K_RIGHT:
                self._steer = 1
            if event.key == pygame.K_LEFT:
                self._steer = -1
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                self._throttle = False
            if event.key == pygame.K_DOWN:
                self._brake = False
                self._control.reverse = False
            if event.key == pygame.K_RIGHT:
                self._steer = None
            if event.key == pygame.K_LEFT:
                self._steer = None
    
    # 处理当前控制状态，更改控制参数
    # if the key remains pressed
    def process_control(self):
        
        if self._throttle: 
            self._control.throttle = min(self._control.throttle + 0.01, 1)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0
        
        if self._brake:
            # If the down arrow is held down when the car is stationary, switch to reverse
            if self._vehicle.get_velocity().length() < 0.01 and not self._control.reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            else:
                self._control.throttle = 0.0
                self._control.brake = min(self._control.brake + 0.3, 1)
        else:
            self._control.brake = 0.0
            
        if self._steer is not None:
            if self._steer == 1:
                self._steer_cache += 0.03
            if self._steer == -1:
                self._steer_cache -= 0.03
            min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache,1)
        else:
            if self._steer_cache > 0.0:
                self._steer_cache *= 0.2
            if self._steer_cache < 0.0:
                self._steer_cache *= 0.2
            if 0.01 > self._steer_cache > -0.01:
                self._steer_cache = 0.0
            self._control.steer = round(self._steer_cache,1)
            
        # Ápply the control parameters to the ego vehicle
        self._vehicle.apply_control(self._control)
  
```

现在我们将初始化车辆和相机。

```py

# Randomly select a vehicle to follow with the camera
ego_vehicle = random.choice(vehicles)

# Initialise the camera floating behind the vehicle
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

# Start camera with PyGame callback
camera.listen(lambda image: pygame_callback(image, renderObject))

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for rendering and vehicle control
renderObject = RenderObject(image_w, image_h)
controlObject = ControlObject(ego_vehicle)

```

初始化 PyGame 接口。这将调用 PyGame 的新窗口。

```py

# Initialise the display
pygame.init()
gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
# Draw black to the display
gameDisplay.fill((0,0,0))
gameDisplay.blit(renderObject.surface, (0,0))
pygame.display.flip()

```
现在我们可以开始游戏循环了。该视图可以在地图中的不同车辆之间随机循环，并在交通管理器控制下可视化它们在交通中的旅程。按 TAB 键切换到随机选择的新车辆，按回车键可以通过键盘上的箭头键手动控制车辆。例如，如果需要挑战驾驶行为不稳定的代理，这种设置可能会很有用。可以调整选择逻辑以选择与代理驾驶的车辆接近的车辆。


```py

# Game loop
crashed = False

while not crashed:
    # Advance the simulation time
    world.tick()
    # Update the display
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
    # Process the current control state
    controlObject.process_control()
    # Collect key press events
    for event in pygame.event.get():
        # If the window is closed, break the while loop
        if event.type == pygame.QUIT:
            crashed = True
        
        # Parse effect of key press event on control state
        controlObject.parse_control(event)
        if event.type == pygame.KEYUP:
            # TAB key switches vehicle
            if event.key == pygame.K_TAB:
                ego_vehicle.set_autopilot(True)
                ego_vehicle = random.choice(vehicles)
                # Ensure vehicle is still alive (might have been destroyed)
                if ego_vehicle.is_alive:
                    # Stop and remove the camera
                    camera.stop()
                    camera.destroy()

                    # Spawn new camera and attach to new vehicle
                    controlObject = ControlObject(ego_vehicle)
                    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
                    camera.listen(lambda image: pygame_callback(image, renderObject))

                    # Update PyGame window
                    gameDisplay.fill((0,0,0))               
                    gameDisplay.blit(renderObject.surface, (0,0))
                    pygame.display.flip()

# Stop camera and quit PyGame after exiting game loop
camera.stop()
pygame.quit()

```

![manual_control](./img/tuto_G_pygame/manual_control.gif)
