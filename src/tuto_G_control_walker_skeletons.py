import carla
import random

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world = client.get_world()
blueprint = random.choice(world.get_blueprint_library().filter('walker.*'))
spawn_points = world.get_map().get_spawn_points()
spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
test_walker = world.try_spawn_actor(blueprint, spawn_point)


# 将相机转向行人的生成点
trans = test_walker.get_transform()
spectator = world.get_spectator()
spectator.set_transform(trans)


control = carla.WalkerBoneControlIn()
first_tuple = ('crl_hand__R', carla.Transform(rotation=carla.Rotation(roll=90)))
second_tuple = ('crl_hand__L', carla.Transform(rotation=carla.Rotation(roll=90)))
control.bone_transforms = [first_tuple, second_tuple]
# test_walker.apply_control(control)
# carla.command.ApplyWalkerControl(control)  # 骨骼始终与其父骨骼相关，因此没有世界坐标，它们是相对于父骨骼的本地坐标。


# 参考：https://github.com/carla-simulator/carla/issues/7621
all_bones = test_walker.get_bones()
new_bones = []

for bone in all_bones.bone_transforms:
    if bone.name == 'crl_root':
        rotation = carla.Rotation(0, 0, 90)
    else:
        rotation = carla.Rotation(0, 0, 0)

    location = [bone.relative.location.x, bone.relative.location.y, bone.relative.location.z]

    new_bones.append((bone.name, carla.Transform(rotation=rotation,
                                                 location=carla.Location(x=location[0], y=location[1], z=location[2]))))

test_walker.set_bones(carla.WalkerBoneControlIn(new_bones))
