import carla
import random

# 设置ego生成位置的选择模式，"spectator" OR "random"
SPAWN_POINT = "random"

# 设置ego的控制模式，"set_transform" OR "ackermann_control" OR "control" OR "autopilot"
CONTROL_MODE = "autopilot"


def main():
    # 创建client，并获取world
    client = carla.Client("localhost", 2000)
    world = client.get_world()

    # 设置ego的车型
    ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    ego_bp.set_attribute('role_name', 'ego')

    if SPAWN_POINT == "spectator":
        # 选择当前spectator位置为ego生成位置
        spectator = world.get_spectator()
        spectator_tf = spectator.get_transform()
        spawn_point = spectator_tf
    elif SPAWN_POINT == "random":
        # 随机选择预定义的生成点为ego生成位置
        spawn_points = world.get_map().get_spawn_points()

        # #生成点的可视化
        # for i, spawn_point in enumerate(spawn_points):
        #     world.debug.draw_string(spawn_point.location, str(i), life_time=100)
        #     world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

        spawn_point = random.choice(spawn_points)

    # 生成ego车
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    snap = world.wait_for_tick()
    init_frame = snap.frame
    run_frame = 0
    print(f"spawn ego vehicle at: {spawn_point}")

    # 获取和设置车辆属性
    bbx = ego_vehicle.bounding_box
    physics = ego_vehicle.get_physics_control()
    print(f"bounding_box = {bbx}")
    print(f"physics = {physics}")

    physics.mass = 2000
    ego_vehicle.apply_physics_control(physics)
    ego_vehicle.set_light_state(carla.VehicleLightState.All)

    # 采用默认的异步变步长模式
    while run_frame < 10000:
        snap = world.wait_for_tick()
        run_frame = snap.frame - init_frame
        print(f"-- run_frame = {run_frame}")

        # 设置spectator跟随ego车
        spectator = world.get_spectator()
        ego_tf = ego_vehicle.get_transform()
        spectator_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
        spectator_tf.location += ego_tf.get_forward_vector() * (-10)
        spectator_tf.location += ego_tf.get_up_vector() * 3
        spectator.set_transform(spectator_tf)

        # 控制车辆
        if CONTROL_MODE == "set_transform":
            new_ego_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
            new_ego_tf.location += ego_tf.get_forward_vector() * 0.1
            ego_vehicle.set_transform(new_ego_tf)
        elif CONTROL_MODE == "ackermann_control":
            # ackermann_control
            ackermann_control = carla.VehicleAckermannControl()
            ackermann_control.speed = 5.0
            ego_vehicle.apply_ackermann_control(ackermann_control)
        elif CONTROL_MODE == "control":
            # control
            control = carla.VehicleControl()
            control.throttle = 0.3
            ego_vehicle.apply_control(control)
        else:
            # 为ego车设置自动驾驶
            ego_vehicle.set_autopilot(True)

        # 获取车辆状态
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()
        acceleration = ego_vehicle.get_acceleration()
        print(f"-current transform = {transform}")
        print(f"-current velocity = {velocity}")
        print(f"-current acceleration = {acceleration}")

    # 销毁车辆
    is_destroyed = ego_vehicle.destroy()
    if is_destroyed:
        print(f"ego_vehicle has been destroyed sucessfully")


if __name__ == "__main__":
    main()