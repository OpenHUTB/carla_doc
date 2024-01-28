import glob
import os
import sys
import carla
import argparse
import numpy as np
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
# 统计路口,设置为全局变量
global xmax,xmin,ymax,ymin
xmax = -11
xmin = -77
ymax = 57
ymin = -13

def get_light_location(world,traffic_lights):
        # 将交通灯的位置信息存储
    traffic_lights_data = {}

    for traffic_light in traffic_lights:
        location = traffic_light.get_location()
        #status = traffic_light.get_state()
        position = (location.x, location.y, location.z)
        # status_str = str(status).split('.')[-1]
        # traffic_lights_data[traffic_light.id] = {'position': position, 'status': status_str}
        traffic_lights_data[traffic_light.id] = {'position': position}

        # 将交通灯在carla中标示出来，以traffic_light.id
        world.debug.draw_string(location, str(traffic_light.id)
                                    , life_time=6000, color=carla.Color(255, 0, 0))

    print(traffic_lights_data)

def get_light_time(traffic_lights):
    #获取交通灯的配时信息
    # 创建一个空的字典，用于存储红绿灯信息
    traffic_lights_info = {}
    # 遍历所有红绿灯
    for traffic_light in traffic_lights:
        # 获取红绿灯的ID
        traffic_light_id = traffic_light.id
        # 获取红绿灯的当前状态
        status = traffic_light.get_state()
        status_str = str(status).split('.')[-1]
        # 获取红绿灯时间
        red_time = traffic_light.get_red_time()
        green_time = traffic_light.get_green_time()
        yellow_time = traffic_light.get_yellow_time()
        # 将红绿灯信息添加到字典中
        traffic_lights_info[traffic_light_id] = {'status': status_str, 'red_time': red_time, 'green_time': green_time,
                                                 'yellow_time': yellow_time}
    print(traffic_lights_info)

def change_light_time(traffic_lights,traffic_light_id,red_time,yellow_time,green_time,world):
    # 遍历红绿灯找到对应路灯
    for traffic_light in traffic_lights:
        if traffic_light.id == traffic_light_id:
                #展示原始红绿灯配时方案
                redtime = traffic_light.get_red_time()
                greentime = traffic_light.get_green_time()
                yellowtime = traffic_light.get_yellow_time()
                print("编号为{}红绿灯的红灯时间为{}，黄灯时间为{}，绿灯时间为{}".format(traffic_light_id, redtime, yellowtime, greentime))

                # 修改红绿灯时长
                traffic_light.set_red_time(float(red_time))
                traffic_light.set_yellow_time(float(yellow_time))
                traffic_light.set_green_time(float(green_time))
                print("已修改编号为{}红绿灯的红灯时间为{}，黄灯时间为{}，绿灯时间为{}".format(traffic_light_id, red_time, yellow_time, green_time))
    world.tick()

def get_traffic_flow(world):
    # 获取所有正在行驶的车辆列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    # 创建一个空的字典,存储车辆的位置信息
    #vehicle_positions = {}
    # 创建两个空列表，用于存储时刻和车流量数据
    #timestamps = []
    traffic_flow = []
    # 循环推进仿真时间并统计车流量
    for t in range(1000):  # 假设模拟1000个时间步
        # 推进仿真时间
        world.tick()
        # 重置车流量计数
        count = 0
        # 遍历所有正在行驶的车辆
        for vehicle in vehicle_list:
            # 获取车辆的位置信息
            location = vehicle.get_location()
            x = location.x
            y = location.y
            z = location.z
            if x <= xmax and x >= xmin and y <= ymax and y >= ymin:
                count += 1
            #vehicle_positions[vehicle.id] = (x, y, z)
        # 将时间和车流量数据添加到相应的列表中
        # timestamps.append(t)
        traffic_flow.append(count)
        #采用1000个时间步的路口车流量的均值作为这段时间的车流量
    return int(np.mean(traffic_flow))



def traffic_signal(traffic_lights,traffic_flow,world):
    #根据路口的车流量去优化路口红绿灯的配时方案简单示例
    #分为高流量：路口车流量>10,中流量：路口车辆在5—10，低流量：路口车辆<5
    # 初始化优化的配时方案
    optimized_timings = {}
    print('路口车流量为{}'.format(traffic_flow))
    # 遍历所有交通灯
    for traffic_light in traffic_lights:

        #获得交通灯的位置信息
        location = traffic_light.get_location()
        #找到位于该路口范围的红绿灯
        if location.x>=xmin and location.x<=xmax and location.y>=ymin and location.y<=ymax:
            # 获取交通灯ID
            traffic_light_id = traffic_light.id
            # 获取交通灯的红绿灯配时方案
            red_time = traffic_light.get_red_time()
            green_time = traffic_light.get_green_time()
            yellow_time = traffic_light.get_yellow_time()
        # 根据车流量调整红绿灯时长
            if traffic_flow <= 4:
            # 低流量，增加红灯灯时间
                red_time = red_time+5
            elif traffic_flow >4 and traffic_flow <7:
            # 中等流量，平衡红绿灯时长
                red_time =red_time+3
                yellow_time = yellow_time+2
                green_time = green_time+2
            else:
            # 高流量，增加绿灯时间
                red_time = red_time+1
                yellow_time=yellow_time+1
                green_time = green_time+5

        # 将优化的配时方案添加到字典中
            optimized_timings[traffic_light_id] = {'red_time': red_time, 'yellow_time': yellow_time,
                                               'green_time': green_time}
        #修改红绿灯的配时方案
            change_light_time(traffic_lights,traffic_light_id,red_time,yellow_time,green_time,world)




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
    argparser.add_argument(
        '--traffic_light_id',
        metavar='I',
        default=9,
        type=int,
        help='traffic light id')

    argparser.add_argument(
        '--red_time',
        metavar='R',
        default=10,
        type=int,
        help='set traffic light red time')
    argparser.add_argument(
        '--yellow_time',
        metavar='Y',
        default=10,
        type=int,
        help='set traffic light yellow time')
    argparser.add_argument(
        '--green_time',
        metavar='G',
        default=10,
        type=int,
        help='set traffic light green time')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 获取Town10HD中所有红绿灯(15个)
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    print(len(traffic_lights))
    #获取交通灯的位置信息
    get_light_location(world,traffic_lights)
    #获取交通灯的配时信息
    get_light_time(traffic_lights)
    #修改红绿灯配时方案
    change_light_time(traffic_lights,args.traffic_light_id,args.red_time,args.yellow_time,args.green_time,world)
    #获取交通灯的配时信息
    get_light_time(traffic_lights)
    # 获取某一时刻的路口车流量
    traffic_flow = get_traffic_flow(world)
    # 根据路口车流量进行简单红绿灯时长优化方案设置
    traffic_signal(traffic_lights, traffic_flow, world)
    # 获取交通灯的配时信息
    get_light_time(traffic_lights)

if __name__ == '__main__':
    main()