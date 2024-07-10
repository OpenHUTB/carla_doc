import glob
import os
import sys
import carla
import argparse
import time
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

SATURATION = 32     # 路口最大容量
DWELL_TIME = 50      # 50秒后认为车辆已离开路口


# 计算车流量
def get_traffic_flow(world, junction, traffic_flows, i, counted_vehicles):
    # 获取所有正在行驶的车辆列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    # counted_vehicles = {} 是一个字典
    current_time = world.get_snapshot().timestamp.elapsed_seconds
    # 遍历所有正在行驶的车辆
    for vehicle in vehicle_list:
        vehicle_id = vehicle.id
        # 表示车辆还没经过路口
        if vehicle_id not in counted_vehicles:
            # 获取车辆的位置信息
            location = vehicle.get_location()
            x = location.x
            y = location.y
            # 车辆经过路口
            if x <= junction[0] and x >= junction[1] and y <= junction[2] and y >= junction[3]:
                # 车流量加1
                traffic_flows[i] = traffic_flows[i] + 1
                counted_vehicles[vehicle_id] = current_time

    # 移除离开路口时间超过dwell_time的车辆
    vehicles_to_remove = [vid for vid, enter_time in counted_vehicles.items() if current_time - enter_time > DWELL_TIME]
    for vehicle_id in vehicles_to_remove:
        del counted_vehicles[vehicle_id]

#得到交通灯的配时信息
def get_traffic_light(world, light_id):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    total_time = 0
    green_time = 0
    for traffic_light in traffic_lights:
        if traffic_light.id == light_id:
            red_time = traffic_light.get_red_time()
            green_time = traffic_light.get_green_time()
            yellow_time = traffic_light.get_yellow_time()
            total_time = red_time + green_time + yellow_time

    return total_time, green_time

#计算路口饱和度
def saturation(world,junctions,counted_vehicles):
    saturation_degrees = [0, 0]
    ave_saturation = [0, 0]
    # 流量
    traffic_flows = [0, 0]

    # 用于跟踪已经计算的车辆id以及进入的时间

    queue_length = [0, 0]
    time_tamp = 0
    while True:
        time_tamp += 1
        # i表示第i个路口
        for i in range(len(junctions)):
            get_traffic_flow(world, junctions[i], traffic_flows, i, counted_vehicles[i])
            saturation_degrees[i] = traffic_flows[i] / SATURATION + saturation_degrees[i]

        # print(saturation_degrees)
        # 1000个时间步拟作1天
        # print(time_tamp)
        if time_tamp == 10000:
            for i in range(len(junctions)):
                ave_saturation[i] = saturation_degrees[i] / 10000
                queue_length[i] = (traffic_flows[i] * 3) / 4
            # 车流量
            # print(traffic_flows)
            # 排队长度
           # print(queue_length)
            # 饱和度
            #print(ave_saturation)
            #print(ave_delay(world, 2, ave_saturation, junctions))
            break



    return ave_saturation




#计算平均路口车均延误
def ave_delay(world, light_id, ave_saturation, junctions):
    vehicle_ave_delays = []
    total_time, green_time = get_traffic_light(world, light_id)
    for i in range(len(junctions)):
        vehicle_ave_delay = 0.5 * total_time * (1-green_time / total_time)**2 / (1 - ave_saturation[i]) + min(1, ave_saturation[i]) * 2
        vehicle_ave_delays.append(vehicle_ave_delay)
    return vehicle_ave_delays


#计算排队长度
def queue_lengths(world,junctions,counted_vehicles):
    # 流量
    traffic_flows = [0, 0]

    # 用于跟踪已经计算的车辆id以及进入的时间

    queue_length = [0, 0]
    time_tamp = 0
    while True:
        time_tamp += 1
        # i表示第i个路口
        for i in range(len(junctions)):
            get_traffic_flow(world, junctions[i], traffic_flows, i, counted_vehicles[i])


        # print(saturation_degrees)
        # 1000个时间步拟作1天
        # print(time_tamp)
        if time_tamp == 10000:
            for i in range(len(junctions)):
                queue_length[i] = (traffic_flows[i] * 3) / 4
            break
    return queue_length


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    # settings = world.get_settings()
    # settings.synchronous_mode = True
    # settings.fixed_delta_seconds = 0.05
    # world.apply_settings(settings)

    # 桐梓坡路-西二环、桐梓坡-望岳路
    # global x_max, x_min, y_max, y_min
    # x_max = -194
    # x_min = -372
    # y_max = 79
    # y_min = -34
    # 路口坐标
    junctions = [
        [-194, -372, 79, -34],
        [449, 381, 48, -87]

    ]
    # 用于跟踪已经计算的车辆id以及进入的时间
    counted_vehicles = [
        {},
        {}
    ]
    # traffic_flows=get_traffic_flow(world, junction, traffic_flows, i, counted_vehicles)
    ave_saturation=saturation(world,junctions,counted_vehicles)  #路口饱和度
    queue_length=queue_lengths(world,junctions,counted_vehicles)
    print(queue_length)
    print(ave_saturation)


if __name__ == '__main__':
    main()
