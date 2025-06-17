# 手动信号控制

这是一个用于控制交通信号灯的Flask应用程序，通过 [Carla模拟环境](https://pan.baidu.com/s/15T1hGoWJ70tVmsTX7-zcSw?pwd=hutb) 实现。用户可以通过指定交通信号灯的OpenDRIVE ID、颜色ID以及设置时长来控制交通信号灯的状态。交通信号灯的颜色设置包括绿灯、黄灯和红灯，分别对应颜色ID [1, 2, 3]。颜色持续时间以秒为单位。

#### 参数说明

[脚本](https://github.com/OpenHUTB/doc/blob/master/src/course/signal_control.py) 接收以下参数：
1. `--traffic_id` (`-I`): 目标交通信号灯的OpenDrive ID。
2. `--color_id` (`-C`): 交通信号灯颜色ID，对应绿灯、黄灯和红灯，分别为1、2和3。
3. `--color_time` (`-T`): 交通信号灯颜色的持续时间，单位为秒。

#### 参数格式

交通信号灯颜色设置参数的格式为【交通信号灯OpenDrive ID，颜色ID，设置时长】。

交通信号灯颜色ID说明：

- 绿灯：`1`
- 黄灯：`2`
- 红灯：`3`

#### 使用步骤

要设置交通信号灯，可以发送一个GET请求到 `/set_traffic_light`：

1. 打开Carla模拟环境
2. 运行脚本signal_control.py来开启服务端
3. 发送http请求:

```
http://127.0.0.1:5000/set_traffic_light?traffic_id=-5&color_id=3&color_time=30
```

以上请求表示设置OpenDrive ID为`-5`的交通信号灯的红灯的持续时间为`30`秒。

#### 脚本工作流程

应用程序的主要功能是根据用户提供的参数设置指定交通信号灯的颜色和持续时间。具体实现步骤如下：

1.接收用户请求，并解析传入的参数。

2.连接到Carla模拟服务器并获取当前世界对象。

```
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # 设置超时
world = client.get_world()  # 获取世界对象
```
3.查找指定的交通信号灯对象,输出修改前的灯光时间。

```
for traffic_light in traffic_lights:
    if float(traffic_light.get_opendrive_id()) == traffic_id:
        if color_id == 1:
            init_time = traffic_light.get_green_time()
        elif color_id == 2:
            init_time = traffic_light.get_yellow_time()
        elif color_id == 3:
            init_time = traffic_light.get_red_time()
```
4.跳转视角到该交通信号灯合适的位置。

```
lights_setting = [
    [-6, carla.Transform(carla.Location(x=-220, y=-9, z=5), carla.Rotation(yaw=180))],
    [-5, carla.Transform(carla.Location(x=-260, y=35, z=5), carla.Rotation(yaw=-90))]
]
```

```
spectator = world.get_spectator()
for setting in lights_setting:
    if setting[0] == traffic_id:
        spectator.set_transform(setting[1])
        break
```
5.根据颜色ID设置交通信号灯的持续时间。

```
for traffic_light in traffic_lights:
    if float(traffic_light.get_opendrive_id()) == traffic_id:
        if color_id == 1:
            traffic_light.set_green_time(float(color_time))
            color_name = "绿色"
        elif color_id == 2:
            traffic_light.set_yellow_time(float(color_time))
            color_name = "黄色"
        elif color_id == 3:
            traffic_light.set_red_time(float(color_time))
            color_name = "红色"
```
6.返回设置结果的JSON响应。

```
response_data = {
    "id": traffic_id,
    "color": color_name,
    "init_time": init_time,
    "last_time": color_time
}
```
#### 注意事项

- 在执行脚本前，请确保CARLA服务器已启动。
- 发送请求时，需要传入对应的参数值。
- settings.no_rendering_mode = False 不能重复设置，否则即使修改交通信号灯时长也不会有效果。

#### 运行结果

![](../img/traffic_course_img/signal_control.gif)

