# 实例分割传感器

**实例分割(Instance segmentation)**是一种新型相机传感器，可为场景中的每个对象生成唯一的像素值。这与语义分割传感器形成对比，语义分割传感器对于相同对象类的实例（例如车辆）具有相同的 ID。

要生成语义分割相机，我们需要`sensor.camera.instance_segmentation`蓝图：

```py
instance_camera_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
```

# 示例

我们将首先设置一个带有实例分割相机的世界，并在场景中生成大量车辆。

连接到服务器并设置为同步模式。

```py
import carla
import random
import time
import queue

# 连接到客户端并将Carla服务器设置为同步模式
client = carla.Client('localhost', 2000)
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings) 
```

设置实例分割传感器并将其生成在所需的地图位置。

```py
# 获得地图的生成点和观察者
spawn_points = world.get_map().get_spawn_points()
spectator = world.get_spectator()

# 在地图上某些位置设置相机
cam_location = carla.Location(x=-46., y=152, z=18)
cam_rotation = carla.Rotation(pitch=-21, yaw=-93.4, roll=0)
camera_transform = carla.Transform(location=cam_location, rotation=cam_rotation)
spectator.set_transform(camera_transform)

# 获取语义相机蓝图和生成相机
instance_camera_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
instance_camera = world.try_spawn_actor(instance_camera_bp, camera_transform)
```

在摄像机周围生成车辆，用大量对象实例填充场景。

```py
# 在相机周围80米内生成车辆
vehicle_bp_library = world.get_blueprint_library().filter('*vehicle*')
radius = 80
for spawn_point in spawn_points:
    vec = [spawn_point.location.x - cam_location.x, spawn_point.location.y - cam_location.y]
    if vec[0]*vec[0] + vec[1]*vec[1] < radius*radius:
        world.try_spawn_actor(random.choice(vehicle_bp_library), spawn_point)
world.tick()
```

现在生成图像。

```py
# 将图片保存到磁盘
instance_image_queue = queue.Queue()
instance_camera.listen(instance_image_queue.put)
world.tick()
instance_image=instance_image_queue.get()
instance_image.save_to_disk('instance_segmentation.png')
```

## 图像输出

保存到磁盘的实例分割图像的实例 ID 编码在 RGB 图像文件的 G 和 B 通道中。R通道包含标准语义ID。

![instance_segmentation](img/instance_segmentation.png)




