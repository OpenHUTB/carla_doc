#### 设置您的代理以进行评估

## 重要提示：此信息已过时，挑战文件已被删除。该功能现已移至 [排行榜](https://github.com/carla-simulator/leaderboard) 。但是，它仍然可以与 scene_runner.py 的路由参数一起使用，而不是与challenge_evaluator.py一起使用 

要让挑战评估系统评估您的代理，您必须定义继承 [AutonomousAgent](https://github.com/carla-simulator/scenario_runner/blob/master/srunner/autoagents/autonomous_agent.py) 基类的 Agent 类。此外，您需要按照挑战评估器教程中的说明设置环境。

在您的代理类上，需要定义三个主要函数来覆盖它们，以便设置代理运行。此外，您还应该考虑最初设置为变量的目标路径。


##### "setup" 函数:
您应该在该功能中为代理进行所有必要的设置。

该函数接收要由用户解析的配置文件的路径作为参数。

执行“challenge_evaluator.py”时，您应该将配置文件路径作为参数传递。例如：

```
python srunner/challenge/challenge_evaluator_routes.py  --agent=<path_to_my_agent> --config=myconfigfilename.format
```


##### "sensors" 函数:

您可以在此功能中设置代理所需的所有传感器。例如，在虚拟代理示例类上定义了以下传感器：

```Python
def sensors(self):
    sensors = [{'type': 'sensor.camera.rgb', 'x':0.7, 'y':0.0, 'z':1.60, 'roll':0.0, 'pitch':0.0, 'yaw':0.0, 'width':800, 'height': 600, 'fov':100, 'id': 'Center'},
               {'type': 'sensor.camera.rgb', 'x':0.7, 'y':-0.4, 'z': 1.60,   'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'width': 800, 'height': 600, 'fov': 100, 'id': 'Left'},
               {'type': 'sensor.camera.rgb', 'x':0.7, 'y':0.4, 'z':1.60, 'roll':0.0, 'pitch':0.0, 'yaw':45.0, 'width':800, 'height':600, 'fov':100, 'id': 'Right'},
               {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'id': 'LIDAR'},
               {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
               {'type': 'sensor.speedometer','reading_frequency': 25, 'id': 'speed'}
              ]
    return sensors
```


每个传感器都是一本字典，您应该在其中指定：

* type: 基本上是要添加的传感器，例如：“sensor.camera.rgb”表示 RGB 相机，“sensor.lidar.ray_cast”表示光线投射激光雷达。
* id: 为传感器提供的标签，以便稍后访问。
* 其他参数：这些参数取决于传感器，例如位置“x”和“y”，或相机的视野“fov”




##### "run_step" 函数:

从挑战评估开始，模拟的每个步骤都会调用此函数，并接收一些输入数据作为参数。

该输入数据是一个字典，其中包含“传感器”函数中指定的所有传感器。

此函数应返回要 应用于自我车辆的 [车辆控制](https://carla.readthedocs.io/en/latest/python_api_tutorial/#vehicles) 。




##### 初始路线:

在执行开始时，英雄代理应该行驶的整个路线是在“self.global_plan”变量上设置的： 

```
[({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002271601998707}, <RoadOption.LANEFOLLOW: 4>),
 ({'z': 0.0, 'lat': 48.99822669411668, 'lon': 8.002709765148996}, <RoadOption.LANEFOLLOW: 4>),
 ...
 ({'z': 0.0, 'lat': 48.99822679980298, 'lon': 8.002735250105061}, <RoadOption.LANEFOLLOW: 4>)]`
 ```

它表示为元组列表，包含以纬度和经度表示的路线路径点以及建议的当前道路选项。对于交叉路口，选项可以是直行、左转或右转。对于路线的其余部分，建议选择车道跟随。

