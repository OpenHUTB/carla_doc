# Carla 消息参考 

以下参考列出了 ROS 桥中可用的所有 Carla 消息。

对这些消息或 Carla-ROS 桥的任何疑问都可以在论坛中解决。

<div class="build-buttons">
<!-- Latest release button -->
<p>
<a href="https://forum.carla.org/c/carla-ecosystem/ros-bridge" target="_blank" class="btn btn-neutral" title="Go to the CARLA Forum, ROS bridge section">
Carla 论坛</a>
</p>
</div>

---
## CarlaActorInfo.msg

ROS 和 Carla 之间共享有关参与者的信息。

| 字段          | 类型                                                      | 描述                                                      |
|-------------| ---------------------------------------------------------- |---------------------------------------------------------|
| `id`        | uint32                                                    | 参与者的ID。                                                 |
| `parent_id` | uint32                                                    | 父参与者的 ID。如果没有可用的父级，则为\`0\`。                             |
| `type`      | string                                                    | 该参与者所基于的蓝图的标识符。                                         |
| `rolename`  | string                                                    | 生成时分配给参与者的名称。 |




---
## CarlaActorList.msg

包含 Carla 参与者一些基本信息的消息列表。

| 字段       | 类型                                     | 描述              |
|----------|----------------------------------------|-----------------|
| `actors` | [CarlaActorInfo](<#carlaactorinfomsg>) | 包含参与者信息的消息列表。 |



---
## CarlaCollisionEvent.msg

检索由参与者的碰撞传感器检测到的碰撞事件的数据。

| 字段                                                                     | 类型                                                                      | 描述                                                               |
| ------------------------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| `header`                                                                  | [Header](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) | 消息发布时的时间戳和帧ID。                   |
| `other_actor_id`                                                          | uint32                                                                    | 检测到碰撞的参与者的 ID。                  |
| `normal_impulse`                                                          | geometry\_msgs/Vector3                                                    | 表示碰撞产生的脉冲的矢量。                 |




---
## CarlaControl.msg

这些消息在同步、非被动模式下控制仿真。定义的常量被翻译为步进命令。

| 字段        | 类型                                                | 描述                                         |
|-----------| ----------------------------------------------------------- |--------------------------------------------|
| `command` | int8                                                | **播放**=0 <br>**暂停**=1 <br>**STEP\_ONCE**=2 |

---

## CarlaEgoVehicleControl.msg

发送消息以在自动驾驶和手动两种模式下对车辆进行控制。这些内容以堆栈形式发布。

| 字段                                                                                                   | 类型                                                                                                    | 描述                                    |
| ------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |---------------------------------------|
| `header`                                                                                                | [Header](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html)                               | 消息发布时的时间戳和帧ID。                        |
| `throttle`                                                                                              | float32                                                                                                 | 控制车辆油门的标量值： **[0.0, 1.0]**            |
| `steer`                                                                                                 | float32                                                                                                 | 控制车辆转向方向的标量值： **[-1.0, 1.0]** 来控制车辆转向 |
| `brake`                                                                                                 | float32                                                                                                 | 控制车辆刹车的标量值： **[0.0, 1.0]**            |
| `hand_brake`                                                                                            | bool                                                                                                    | 如果为 **True**，则启用手刹。                   |
| `reverse`                                                                                               | bool                                                                                                    | 如果为 **True**，车辆将向后移动。                 |
| `gear`                                                                                                  | int32                                                                                                   | 车辆中可用档位之间的变化。                         |
| `manual_gear_shift`                                                                                     | bool                                                                                                    | 如果为 **True**，则将使用 `gear` 进行换档。        |

---

## CarlaEgoVehicleInfo.msg

有关车辆的静态信息，主要是用于定义车辆物理特性的属性。 

| 字段                                                   | 类型                                                           | 描述                                                           |
|------------------------------------------------------| -------------------------------------------------------------- |--------------------------------------------------------------|
| `id`                                                 | uint32                                                         | 车辆参与者的 ID。                                                   |
| `type`                                               | string                                                         | 该车辆所基于的蓝图的标识符。                                               |
| `type`                                               | string                                                         | 该车辆所基于的蓝图的标识符。                                               |
| `rolename`                                           | string                                                         | 分配给车辆的参与者。                                                    |
| `wheels`                                             | [CarlaEgoVehicleInfoWheel](<#carlaegovehicleinfowheelmsg>)     | 包含有关车轮信息的消息列表。                                               |
| `max_rpm`                                            | float32                                                        | 车辆发动机的最大转速。                                                  |
| `moi`                                                | float32                                                        | 车辆发动机的转动惯量。                                                  |
| `damping_rate_full_throttle`                         | float32                                                        | 油门最大时的阻尼率。                                                   |
| `damping_rate_zero_throttle`<br>`_clutch_engaged`    | float32                                                        | 当离合器接合且油门为零时的阻尼率。                                            |
| `damping_rate_zero_throttle`<br>`_clutch_disengaged` | float32                                                        | 当油门为零且离合器分离时的阻尼率。                                            |
| `use_gear_autobox`                                   | bool                                                           | 如果为 **True**，车辆将配备自动变速箱。                                     |
| `gear_switch_time`                                   | float32                                                        | 档位之间的切换时间。                      |
| `clutch_strength`                                    | float32                                                        | 车辆的离合器强度。以 **Kgm^2/s**为单位测量。 |
| `mass`                                               | float32                                                        | 车辆的质量以千克为单位测量。                      |
| `drag_coefficient`                                   | float32                                                        | 车辆底盘的阻力系数。                   |
| `center_of_mass`                                     | geometry\_msgs/Vector3                                         | 车辆的质心。                           |

---

## CarlaEgoVehicleInfoWheel.msg

有关车轮的静态信息将成为 [CarlaEgoVehicleInfo.msg](#carlaegovehicleinfomsg) 消息的一部分。

| 字段                     | 类型                     | 描述                                              |
|------------------------|------------------------| -------------------------------------------------------- |
| `tire_friction`        | float32                | 表示车轮摩擦力的标量值。 |
| `damping_rate`         | float32                | 车轮的阻尼率。                           |
| `max_steer_angle`      | float32                | 车轮可以转向的最大角度（以度为单位）。   |
| `radius`               | float32                | 轮子的半径，以厘米为单位。                  |
| `max_brake_torque`     | float32                | 最大制动扭矩（Nm）。                          |
| `max_handbrake_torque` | float32                | 最大手刹扭矩（Nm）。                     |
| `position`             | geometry\_msgs/Vector3 | 车轮的世界位置。                             |

---

## CarlaEgoVehicleStatus.msg

车辆作为世界上物体的当前状态。

| 字段             | 类型                                                                      | 描述              |
|----------------| ------------------------------------------------------------------------- |-----------------|
| `header`       | [Header](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) | 消息发布时的时间戳和帧ID。  |
| `velocity`     | float32                                                                   | 车辆的当前速度。        |
| `acceleration` | geometry\_msgs/Accel                                                      | 车辆当前的加速度。       |
| `orientation`  | geometry\_msgs/Quaternion                                                 | 车辆的当前朝向。        |
| `control`      | [CarlaEgoVehicleControl](<#carlaegovehiclecontrolmsg>)                    | Carla 报告的当前控制值。 |

---

## CarlaLaneInvasionEvent.msg

这些消息发布由连接到车辆的压线传感器检测到的压线。最后一步中检测到的入侵将作为具有常量定义的列表传递，以识别穿过的车道。



| 字段                      | 类型                                                                          | 描述                                                                                        |
|-------------------------|-----------------------------------------------------------------------------|-------------------------------------------------------------------------------------------|
| `header`                | [header](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) | 消息发布时的时间戳和帧ID。                                    |
| `crossed_lane_markings` | int32[]                                                                     | **LANE\_MARKING\_OTHER**=0 <br>**LANE\_MARKING\_BROKEN**=1 <br>**LANE\_MARKING\_SOLID**=2 |

---

## CarlaScenario.msg

测试场景的详细信息。


| 字段              | 类型                               | 描述           |
|-----------------| ---------------------------------- |--------------|
| `name`          | string                             | 场景名称。        |
| `scenario_file` | string                             | 该场景的测试文件。    |
| `destination`   | geometry\_msgs/Pose                | 场景的目标位置。     |
| `target_speed`  | float64                            | 场景期间期望的速度。 |

---

## CarlaScenarioList.msg

要在 ScenarioRunner 中运行的测试场景列表。


| 字段          | 类型                                     | 描述                 |
|-------------|----------------------------------------|--------------------|
| `scenarios` | [CarlaScenario[]](<#carlascenariomsg>) | 场景列表。 |

---

## CarlaScenarioRunnerStatus.msg

ScenarioRunner 的当前状态。它是使用常量来管理的。


| 字段       | 类型                                                                                                                                              | 描述                                                                                                                                       |
|----------| ------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `status` | uint8                                                                                                                                             | 场景的当前状态作为枚举： <br>**STOPPED**=0 <br>**STARTING**=1 <br>**RUNNING**=2 <br>**SHUTTINGDOWN**=3 <br>**ERROR**=4 |

---

## CarlaStatus.msg

当前世界的仿真设置。 


| 字段                         | 类型    | 描述                             |
|----------------------------| -------------------------------------------------- |--------------------------------|
| `frame`                    | uint64        | 当前帧号。                          |
| `fixed_delta_seconds`      | float32    | 最后一步和当前步之间的仿真时间。               |
| `synchronous_mode`         | bool  | 如果为 **True**，则启用同步模式。          |
| `synchronous_mode_running` | bool    | 当仿真运行时为**True**。暂停时为**False**。 |

---

## CarlaTrafficLightStatus.msg

关于交通灯状态的恒定定义。

| 字段      | 类型     | 描述                                                                         |
|---------|--------|----------------------------------------------------------------------------|
| `id`    | uint32 | 交通灯参与者的 ID。                                                              |
| `state` | uint8  | **RED**=0 <br>**YELLOW**=1 <br>**GREEN**=2 <br>**OFF**=3 <br>**UNKNOWN**=4 |

---

## CarlaTrafficLightStatusList.msg

交通灯列表及其状态。

| 字段          | 类型                                                       | 描述                                                                |
|-------------| ---------------------------------------------------------- |-------------------------------------------------------------------|
| `scenarios` | [CarlaTrafficLightStatus[]](<#carlatrafficlightstatusmsg>) | 汇总的交通灯状态消息列表。 |

---

## CarlaWalkerControl.msg

将运动控制器应用于行人所需的信息。

| 字段          | 类型                     | 描述                     |
|-------------|------------------------|------------------------|
| `direction` | geometry\_msgs/Vector3 | 控制行人方向的向量。             |
| `speed`     | float32                | 用于控制行人速度的标量值。          |
| `jump`      | bool                   | 如果为 **True**，行人会跳跃。 |

---

## CarlaWaypoint.msg

路径点对象中包含的数据。

| 字段            | 类型                                                                                                                                     | 描述                                         |
|---------------| ---------------------------------------------------------------------------------------------------------------------------------------- |--------------------------------------------|
| `road_id`     | int32                                                                                                                                    | OpenDRIVE 道路的 id。                          |
| `section_id`  | int32                                                                                                                                    | OpenDRIVE 部分的 id，基于它们最初定义的顺序。              |
| `lane_id`     | int32                                                                                                                                    | OpenDRIVE 车道的id，该值可以是正数或负数，表示当前车道相对于道路的方向。 |
| `is_junction` | bool                                                                                                                                     | 如果当前路点位于 OpenDRIVE 定义的交汇点上，则为 **True**。    |
| `is_junction` | [geometry\_msgs/Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html)                                                        | 当仿真运行时为 **True** 。暂停时为  **False**。         |

---

## CarlaWorldInfo.msg

有关当前 Carla 地图的信息。

| 字段          | 类型                                                 | 描述                                                                              |
|-------------| ---------------------------------------------------- |---------------------------------------------------------------------------------|
| `map_name`  | string                                               | 当前世界中加载的 Carla 地图的名称。                                                           |
| `opendrive` | string                                               | 当前地图的 .xodr OpenDRIVE 文件（字符串形式）。 |

---

## EgoVehicleControlCurrent.msg

车辆的当前时间、速度和加速度值。由控制器使用。它是`Carla_Ackermann_Control.EgoVehicleControlInfo.msg`消息的一部分。

| 字段          | 类型                                            | 描述                                              |
|-------------| ----------------------------------------------- |-------------------------------------------------|
| `time_sec`  | float32                                         | 应用控制器的当前时间。                                     |
| `speed`     | float32                                         | 控制器应用的当前速度。                                     |
| `speed_abs` | float32                                         | 为绝对值的速度。                                        |
| `accel`     | float32                                         | 控制器应用的当前加速度。 |

---

## EgoVehicleControlInfo.msg

阿克曼控制器内的当前值。这些消息对于调试很有用。

| 字段             | 类型                                                                          | 描述                                                               |
|----------------|-----------------------------------------------------------------------------| ------------------------------------------------------------------------- |
| `header`       | [header](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html) | 消息发布时的时间戳和帧ID。                    |
| `restrictions` | [EgoVehicleControlMaxima](<#egovehiclecontrolmaximamsg>)                    | 控制器值的限制。                                          |
| `target`       | [EgoVehicleControlTarget](<#egovehiclecontroltargetmsg>)                    | 控制器值的限制。                                          |
| `current`      | [EgoVehicleControlCurrent](<#egovehiclecontrolcurrentmsg>)                  | 控制器值的限制。                                          |
| `status`       | [EgoVehicleControlStatus](<#egovehiclecontrolstatusmsg>)                    | 控制器值的限制。                                          |
| `output`       | [CarlaEgoVehicleControl](<#carlaegovehiclecontrolmsg>)                      | 控制器值的限制。                                         |

---

## EgoVehicleControlMaxima.msg

控制器限制（极限值）。它是`Carla_Ackermann_Control.EgoVehicleControlInfo.msg`消息的一部分。

| 字段                   | 类型                                                                                                                 | 描述                                 |
|----------------------| -------------------------------------------------------------------------------------------------------------------- |------------------------------------|
| `max_steering_angle` | float32                                                                                                              | 车辆的最大转向角。                          |
| `max_speed`          | float32                                                                                                              | 车辆的最大速度。                           |
| `max_accel`          | float32                                                                                                              | 车辆的最大加速度。                          |
| `max_decel`          | float32                                                                                                              | 车辆的最大减速度。默认值： **8m/s^2**           |
| `min_accel`          | float32                                                                                                              | 车辆的最小加速度。当阿克曼目标加速时。超过此值，输入加速度。被控制。 |
| `max_pedal`          | float32                                                                                                              | 最小踏板。                              |

---

## EgoVehicleControlStatus.msg

自我车辆控制器的当前状态。它是`Carla_Ackermann_Control.EgoVehicleControlInfo.msg`消息的一部分。

| 字段                               | 类型      | 描述                      |
|----------------------------------|---------| -------------------------------- |
| `status`                         | string  | 当前控制状态。          |
| `speed_control_activation_count` | uint8   | 速度控制器。                |
| `speed_control_accel_delta`      | float32 | 速度控制器。                |
| `speed_control_accel_target`     | float32 | 速度控制器。                |
| `accel_control_pedal_delta`      | float32 | 加速控制器。        |
| `accel_control_pedal_target`     | float32 | 加速控制器。         |
| `brake_upper_border`             | float32 | 搁置踏板的边框。       |
| `throttle_lower_border`          | float32 | 搁置踏板的边框。       |

---

## EgoVehicleControlTarget.msg

自我车辆控制器的目标值。它是`Carla_Ackermann_Control.EgoVehicleControlInfo.msg`消息的一部分。

| 字段               | 类型                                      | 描述           |
|------------------| ----------------------------------------- |--------------|
| `steering_angle` | float32                                   | 控制器的目标转向角。   |
| `speed`          | float32                                   | 控制器的目标速度。    |
| `speed_abs`      | float32                                   | 为绝对值的速度。     |
| `accel`          | float32                                   | 控制器的目标加速度。   |
| `jerk`           | float32                                   | 控制器的目标抖动。 |

<br>
