# Carla ROS Scenario Runner

The [CARLA ROS Scenario Runner 包](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ros_scenario_runner) 是一个包装器，用于通过 ROS 使用 CARLA [Scenario Runner](https://github.com/carla-simulator/scenario_runner) 执行 [OpenScenarios](https://www.asam.net/standards/detail/openscenario/) 。 

- [__在你开始之前__](#before-you-begin)
- [__使用 ROS Scenario Runner__](#using-ros-scenario-runner)
- [__运行 ROS Scenario Runner__](#run-ros-scenario-runner)
- [__ROS 应用程序接口__](#ros-api)
    - [服务](#services)
    - [发布](#publications)

---

## 在你开始之前

- 按照 Scenario Runner [入门”教程](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md) 安装 Scenario Runner。
- 安装 Python 模块 __Pexpect__:

```shell
sudo apt install python-pexpect
```
---

## 使用 ROS Scenario Runner

ROS Scenario Runner 最好在 [`rviz_carla_plugin`](rviz_plugin.md) 中使用。

!!! 笔记
    目前不支持更改地图。每个场景都需要使用当前活动的地图。

[此处](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_demo/config/FollowLeadingVehicle.xosc) 提供了一个示例场景。特别重要的是 [ROS 控制器](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ad_demo/config/FollowLeadingVehicle.xosc#L78) 的设置：

```xml
<Controller name="EgoVehicleAgent">
    <Properties>
        <Property name="module" value="carla_ros_scenario_runner.ros_vehicle_control" />
        <Property name="launch" value="carla_ad_agent.launch"/>
        <Property name="launch-package" value="carla_ad_agent"/>
        <Property name="path_topic_name" value="waypoints"/>
    </Properties>
</Controller>
```

上面的代码示例显示了一个正在启动的 [`carla_ad_agent`](carla_ad_agent.md) 实例。任何附加内容`<Property>`都应作为 ROS 参数附加（名称:=值）。


---

## 运行 ROS Scenario Runner

__1.__ 运行 ROS Scenario Runner 包：

```sh
# ROS 1
roslaunch carla_ros_scenario_runner carla_ros_scenario_runner.launch scenario_runner_path:=<path_to_scenario_runner>

# ROS 2
ros2 launch carla_ros_scenario_runner carla_ros_scenario_runner.launch.py scenario_runner_path:=<path_to_scenario_runner>
```

__2.__ 运行场景：

```sh
# ROS 1
rosservice call /scenario_runner/execute_scenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>' } }"

# ROS 2
ros2 service call /scenario_runner/execute_scenario carla_ros_scenario_runner_types/srv/ExecuteScenario "{ 'scenario': { 'scenario_file': '<full_path_to_openscenario_file>' } }"
```

---

## ROS 应用程序接口

### 服务

| 服务 | 类型 | 描述 |
|---------|------|-------------|
| `/scenario_runner/execute_scenario` | [`carla_ros_scenario_runner_types.ExecuteScenario`](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ros_scenario_runner_types/srv/ExecuteScenario.srv) | 执行一个场景。如果当前正在运行另一个场景，它将停止。 |

<br>

### 发布

| 主题                        | 类型 | 描述                                                                                                                                                           |
|---------------------------|------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `/scenario_runner/status` | [`carla_ros_scenario_runner_types.CarlaScenarioRunnerStatus`](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ros_scenario_runner_types/msg/CarlaScenarioRunnerStatus.msg) | 场景运行器执行的当前状态（由 [rviz_carla_plugin](rviz_plugin.md) 使用）  |


控制器`ros_vehicle_control`提供以下主题：

| 主题       | 类型                         | 描述                                                                                                                                                                |
|-----------------------------------|----------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `/carla/<ROLE NAME>/waypoints`    | [`nav_msgs.Path`](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)       | 场景中定义的路径。注：可以通过修改参数`path_topic_name`来更改主题名称 |
| `/carla/<ROLE NAME>/target_speed` | [`std_msgs.Float64`](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) | 场景中定义的目标速度                                                                                                                   |

<br>
