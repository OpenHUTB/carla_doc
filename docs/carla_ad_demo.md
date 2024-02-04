# Carla 自动驾驶演示

[自动驾驶](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_demo) 是一个示例包，提供使用自动驾驶车辆启动 Carla ROS 环境所需的一切。 

- [__在你开始之前__](#before-you-begin)
- [__运行演示__](#run-the-demo)
    - [随机路线](#random-route)
    - [场景执行](#scenario-execution)
---

## 在你开始之前

安装 [Scenario Runner](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/) 并按照 [Scenario Runner 入门教程](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) 验证其是否正常工作。

设置环境变量以查找 Scenario Runner 安装：

```sh
export SCENARIO_RUNNER_PATH=<path_to_scenario_runner>
```

---

## 运行演示


#### 随机路线

要启动自我车辆遵循随机生成的路线的演示，请在启动 Carla 服务器后运行以下命令：

```sh
# ROS 1
roslaunch carla_ad_demo carla_ad_demo.launch

# ROS 2
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

您还可以通过在另一个终端中执行以下命令来生成更多车辆或行人：

```sh
cd <CARLA_PATH>/PythonAPI/examples/

python3 spawn_npc.py
```

#### 场景执行

要使用预定义场景执行演示，请在启动 Carla 服务器后运行以下命令：

```sh
# ROS1
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch

# ROS2
ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py
```

在 RVIZ Carla 插件中选择示例场景“FollowLeadingVehicle”，然后按“执行”。自我车辆重新定位并处理场景。

您可以通过发布到 `/carla/available_scenarios` 来指定自己的场景。[启动文件](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch) 显示了如何执行此操作的示例：

```launch
  <node pkg="rostopic" type="rostopic" name="publish_scenarios"
    args="pub /carla/available_scenarios carla_ros_scenario_runner_types/CarlaScenarioList '{ 'scenarios':  
      [
        {
          'name': 'FollowLeadingVehicle',
          'scenario_file': '$(find carla_ad_demo)/config/FollowLeadingVehicle.xosc'
        }
      ]
    }' -l"/>
```

---



