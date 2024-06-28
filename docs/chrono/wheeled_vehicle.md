# 轮式车辆
`Chrono::Vehicle` 中的轮式车辆是抽象通用车辆系统 ( ChVehicle ) 的一种特殊化，被定义为子系统的集合（如下所示）。

![](../img/chrono/wheeled_subsystems.png)

轮式车辆包含底盘子系统、传动系统子系统和任意数量的车轴，按照惯例，车轴从车辆前部开始编号。多转向车辆通过允许任意数量的转向机构（连接到不同的车轴）或允许多个可转向车轴连接到同一转向机构来支持。每个车轴每侧可以容纳一个或两个轮胎。

参见 [ChWheeledVehicle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_wheeled_vehicle.html) 。

以下章节描述了各种子系统模板：

- [悬挂模型](https://api.projectchrono.org/wheeled_suspension.html) 
- [转向机构模型](https://api.projectchrono.org/wheeled_steering.html) 
- [传动系统模型](https://api.projectchrono.org/wheeled_driveline.html)
- [轮胎型号](https://api.projectchrono.org/wheeled_tire.html)
