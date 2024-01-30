# 交通灯配置和使用（get_traffic_light）

## 交通灯在Carla中的简单配置

#### 1、在Carla中为场景设置红绿灯

##### 1.1、从内容浏览器中选择Carla > Static > TrafficLight > StreetLights_01下的交通灯蓝图

##### 1.2、将交通灯拖到场景中，进行位置调整。

##### 1.3、与交通灯相连接的有一个立方体组件，通过拖到这个立方体的位置和改变长度来决定对应红绿灯的影响范围

##### 1.4、将交通灯分配给路口交通灯组

*从内容浏览器中选择Carla > Static > TrafficLight > StreetLights_01下选择BP_TrafficLightGroup拖动到路口，并在该对象的细节面板下找到Traffic Light，将需要添加到该路口组的交通灯设置*

**参考链接：**[内容创作 - 地图 - 交通仿真器文档 (openhutb.github.io)](https://openhutb.github.io/carla_doc/tuto_content_authoring_maps/#_5)的交通灯部分





## 使用python脚本简单使用交通灯

#### 创建与carla的连接

```python
client = carla.Client(args.host, args.port)
client.set_timeout(10.0)  # 设置超时
world = client.get_world()  # 获取世界对象

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
```

#### 获取场景中的红绿灯对象

```python
traffic_lights = world.get_actors().filter('traffic.traffic_light')
```

#### 1、获取红绿灯的位置信息

```python
def get_light_location(world,traffic_lights):
        # 将交通灯的位置信息存储
    traffic_lights_location = {}

    for traffic_light in traffic_lights:
        location = traffic_light.get_location()
        #status = traffic_light.get_state()
        position = (location.x, location.y, location.z)
        # status_str = str(status).split('.')[-1]
        # traffic_lights_data[traffic_light.id] = {'position': position, 'status': status_str}
        traffic_lights_location[traffic_light.id] = {'position': position}

        # 将交通灯在carla中标示出来，以traffic_light.id
        world.debug.draw_string(location, str(traffic_light.id)
                                    , life_time=6000, color=carla.Color(255, 0, 0))
    return  traffic_lights_location
    print(traffic_lights_location)
```

#### 2、获取红绿灯的当前配时方案

```python
def get_light_time(traffic_lights):
    #获取交通灯的配时信息
    # 创建一个空的字典，用于存储红绿灯信息
    traffic_lights_time = {}
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
        traffic_lights_time[traffic_light_id] = {'status': status_str, 'red_time': red_time, 'green_time': green_time,
                                                 'yellow_time': yellow_time}
    print(traffic_lights_time)
    return traffic_lights_time
```

#### 3、修改红绿灯的配时方案

```python
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
    world.tick()  #更新Carla世界的状态，使得修改的红绿灯方案立即生效。
```

#### 4、根据路口车流量调整交通灯的红绿灯配时方案

##### 4.1、获取指定路口的车流量

```python
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
            #暂时采用xmin,xmax,ymin,ymax来指定区域路口
            if x <= xmax and x >= xmin and y <= ymax and y >= ymin:
                count += 1
            #vehicle_positions[vehicle.id] = (x, y, z)
        # 将时间和车流量数据添加到相应的列表中
        # timestamps.append(t)
        traffic_flow.append(count)
        #采用1000个时间步的路口车流量的均值作为这段时间的车流量
    return int(np.mean(traffic_flow))
```

##### 4.2、调整配时方案

```python
def traffic_signal(traffic_lights,traffic_flow,world):
    #根据路口的车流量去优化路口红绿灯的配时方案简单示例
    #该示例中设置的车辆总数为70辆，分为高流量：路口车流量>7,中流量：路口车辆在4—7，低流量：路口车辆<4
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

        #修改红绿灯的配时方案
            change_light_time(traffic_lights,traffic_light_id,red_time,yellow_time,green_time,world)
```
