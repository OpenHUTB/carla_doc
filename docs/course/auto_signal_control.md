# 自动信号控制

这是一个基于Flask的应用程序，用于控制[Carla仿真环境](https://pan.baidu.com/s/15T1hGoWJ70tVmsTX7-zcSw?pwd=hutb)中的红绿灯。该应用程序通过检测路口的交通流量动态调整红绿灯的时长，以优化交通通行效率。

#### 主要功能

[应用程序](https://github.com/OpenHUTB/carla_doc/blob/master/src/course/auto_signal_control.py) 主要功能包括：
1. 获取指定路口的交通流量。
2. 根据交通流量动态调整红绿灯（绿灯）的时长。
3. 返回指定路口当前全部红绿灯的状态和时长信息。

#### 传入参数

- 无需传入参数，该应用程序自动检测指定路口的交通流量并调整红绿灯的时长。

#### 核心实现步骤

**1.初始化和连接Carla服务器**：获取世界对象并设置仿真参数。

```
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
```
**2.获取路口信息**：通过中心位置获取指定路口的交通信号灯信息。

为了保证每次请求不会往该路口的交通灯集合中新加重复的红绿灯，先用集合存储红绿灯的唯一ID

```
traffics_in_junction = set()
```

```
def get_traffic_lights_in_junction(world, junction):
    """获取路口的所有红绿灯"""
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for traffic_light in traffic_lights:
        if get_distance(traffic_light.get_location(), center_location[0]) <= 50:
            id = traffic_light.get_opendrive_id()
            traffics_in_junction.add(id)
```

```
def get_light(all_traffic_lights):
    """根据红绿灯ID列表获取对应的红绿灯对象"""
    drive_list = list(traffics_in_junction)
    traffic_lights = []
    for traffic_light in all_traffic_lights:
        if traffic_light.get_opendrive_id() in drive_list:
            traffic_lights.append(traffic_light)
    return traffic_lights
```
**3.计算交通流量**：检测指定范围内的车辆数量。

```
def get_vehicles_in_intersection(world, intersection_location, intersection_radius):
    """获取路口范围内的车辆数量"""
    vehicles = world.get_actors().filter('vehicle.*')
    vehicles_in_intersection = []

    for vehicle in vehicles:
        distance = get_distance(vehicle.get_location(), intersection_location)
        if distance <= intersection_radius:
            vehicles_in_intersection.append(vehicle)

    return len(vehicles_in_intersection)
```
**4.调整红绿灯时长**：根据交通流量动态调整红绿灯的时长。

```
 if junction_flow > 15:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 15)
            last_green_time = GREEN_TIME + 15
    elif junction_flow > 10:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 10)
            last_green_time = GREEN_TIME + 10
    elif junction_flow > 5:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME + 5)
            last_green_time = GREEN_TIME + 5
    else:
        for traffic_light in get_light(all_traffic_lights):
            traffic_light.set_green_time(GREEN_TIME)
            last_green_time = GREEN_TIME

```
**5.返回结果**：返回当前红绿灯的状态和调整后的时长信息。

```
 response_data = {
            "traffic_lights": [
                {"id": traffic_ids},
                {"init_green_time": traffic_init_green_time},
                {"last_green_time": last_green_time}
            ]
        }

```
#### 使用步骤

获取路口交通灯信息，可以发送一个GET请求到 `/set_traffic_light`路由,应用程序会自动根据路口的交通流量调整红绿灯的时长并返回结果。

1. 打开Carla仿真环境
2. 先运行[generate_traffic.py](https://github.com/OpenHUTB/carla_doc/blob/master/src/examples/generate_traffic.py) 来生成交通
3. 运行脚本auto_signal_control.py来开启服务端
4. 发送http请求:

```
http://127.0.0.1:5000/set_traffic_light
```

通过这些步骤和说明，用户可以使用该应用程序动态调整Carla仿真环境中的红绿灯时长，以优化交通通行效率。