# 基于ROS的挑战智能体

与 ROS 的 CARLA 接口通常通过 [CARLA ROS Bridge](https://github.com/carla-simulator/ros-bridge) 完成。在挑战模式下，此桥接功能由 RosAgent 提供。它对传感器使用相同的主题和消息类型，但不发布 tf 转换。
 
# 要求

* `roscore` 预计将在 docker 容器中运行。请调整您的入口点。

## 设置

要在挑战模式下启用堆栈，需要执行以下步骤：

1. 定义传感器设置
2. 定义启动

### 定义传感器设置

派生自RosAgent并实现sensors()方法。

    from srunner.autoagents.ros_agent import RosAgent

    class MyRosAgent(RosAgent):

        def sensors(self):
            return [ <sensor-definition> ]

作为传感器定义的示例，请参阅 [HumanAgent.py](../srunner/autoagents/human_agent.py)。 


### 定义启动

堆栈的启动是在 shell 脚本 `$TEAM_CODE_ROOT/start.sh` 内完成的。因此 `TEAM_CODE_ROOT` 必须设置环境变量。

RosAgent 负责执行和监控。只要堆栈处于活动状态，脚本就应保持运行。

start.sh 的示例

    #!/bin/bash -e
    roslaunch $TEAM_CODE_ROOT/challenge.launch


## 测试

一般来说，挑战执行是无头的。为了进行诊断，您仍然可以使用 rviz 或 rqt 等 ros 工具。此外，您可以使用carla_ros_bridge 中的 [carla_manual_control](https://github.com/carla-simulator/ros-bridge/tree/master/carla_manual_control) 进行可视化（并控制车辆）。

