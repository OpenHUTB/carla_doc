## 交通信号灯蓝图

* 自定义触发器组件 [BP_CustomTriggerComponent](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_CustomTriggerComponent.uasset)
* 交通信号灯与OpenDrive [BP_TLOpenDrive](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_TLOpenDrive.uasset)
* 交通信号灯与OpenDrive版本2 [BP_TLOpenDrive_v02](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_TLOpenDrive_v02.uasset)
* [BP_TrafficLightNew_T10_master_largeBIG_rsc](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_TrafficLightNew_T10_master_largeBIG_rsc.uasset)
* [BP_TrafficLightNew_T10_master_mediumBIG_rsc](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_TrafficLightNew_T10_master_mediumBIG_rsc.uasset)
* [BP_TrafficLightNew_T10_master_noArmsBIG_rsc](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/TrafficLight/BP_TrafficLightNew_T10_master_noArmsBIG_rsc.uasset)
# Carla虚幻场景中交通信号灯（TrafficLight）技术说明

## 1. 概述
在Carla自动驾驶仿真平台中，交通信号灯（TrafficLight）是模拟交通环境中控制车辆和行人动态行为的关键元素。它们模仿现实中的交通信号装置，通过红绿灯的变化调节交通流，确保模拟场景的真实性和可靠性。本文介绍Carla中交通信号灯的基本功能、管理方式、操作接口以及其在模拟中的应用场景。

---

## 2. 交通信号灯（TrafficLight）简介
- **定义**：交通信号灯（TrafficLight）是位于道路交叉口或特定路段，用以指示车辆及行人通行状态的场景元素。
- **功能**：
  - 控制交叉口或道路片段的通行与等待状态。
  - 支持多种状态切换（如绿灯、黄灯、红灯）。
  - 可以自动或手动控制，实现模拟中的交通管理。
- **作用场景**：
  - 构建真实的交通环境，测试自动驾驶策略。
  - 仿真交通规则遵守与违法行为。
  - 配合其他场景元素，模拟复杂交通场景。

---

## 3. 交通信号灯的基本构造与属性
### 3.1 实例创建
- 交通信号灯作为场景中的参与者（Actor）存在，可由蓝图（Blueprint）实例化。
- 通过API可以获取现有信号灯，修改或控制其状态。
  
### 3.2 主要属性
- **状态（State）**：显示当前交通信号灯的颜色状态（红色、黄色、绿色）。
- **时间参数（Timing）**：定义状态持续时间以及转换时间。
- **位置（Transform）**：信号灯在场景中的位置与方向。
- **灯光类型**：单灯或多灯组合（如箭头灯、圆形灯等）。

### 3.3 信号灯状态
- **Green（绿灯）**：允许车辆和行人通行。
- **Yellow（黄灯）**：警示即将变红，车辆应准备停止。
- **Red（红灯）**：禁止车辆和行人通行。

---

