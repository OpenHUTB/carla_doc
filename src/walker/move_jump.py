# 行人在生成时跳跃，视角并切换到行人的位置
import random
import time

import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print(client.get_available_maps())  # 获取所有可使用地图名

world = client.get_world()

bp = random.choice(world.get_blueprint_library().filter('walker'))
transform = world.get_map().get_spawn_points()[0]
pedestrain = world.spawn_actor(bp, transform)

# 我们也会设置观察者以便我们能看到我们所做的
spectator = world.get_spectator()
trans = pedestrain.get_transform()
spectator.set_transform(trans)

control = carla.WalkerControl()

i = 0
while True:
    control = carla.WalkerControl()
    control.speed = 0.9
    control.direction.y = 1
    control.direction.x = 0
    control.direction.z = 0
    pedestrain.apply_control(control)
    time.sleep(1)

    # boolean operation 1 for jump and 0 for not to jump
    control.jump = 1
    pedestrain.apply_control(control)
    time.sleep(1)

    pedestrain.apply_control(control)
    time.sleep(1)

    i += 1