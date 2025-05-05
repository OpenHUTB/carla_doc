# 定制车辆悬架

本教程介绍了 Carla 车辆悬架系统的基础知识，以及如何针对不同的可用车辆实施这些基础知识。使用此信息可以访问虚幻引擎中车辆的悬架参数，并随意对其进行自定义。

*   [__悬架系统的基础知识__](#basics-of-the-suspension-system)  
*   [__悬架组__](#suspension-groups)  
	*   [轿跑车](#coupe)  
	*   [越野](#off-road)  
	*   [卡车](#truck)  
	*   [城市车](#urban)  
	*   [货车](#van)  

---
## 悬架系统的基础知识 <span id="basics-of-the-suspension-system"></span>

车辆的悬架系统由所述车辆的车轮限定。每个车轮都有一个带有一些参数化的独立蓝图，其中包括悬架系统。

这些蓝图可以在 `Content/Carla/Blueprints/Vehicles/<vehicle_name>` 中找到。它们的名称如下：`BP_<vehicle_name>_<F/R><R/L>W`。 

*   `F` 或 `R` 相应地用于前轮或后轮。
*   `R` 或 `L` 应地用于右轮或左轮。 

![tuto_suspension_blueprints](img/tuto_suspension_blueprints.jpg)
<div style="text-align: center"><i>在本例中，奥迪A2左前轮的蓝图命名为 <code>BP_AudiA2_FLW</code>.</i></div>

`shape_radius` 让车轮停在路面上，既不悬停也不在路面内部。

在蓝图中，有一个部分包含有关车轮悬架的一些参数化。以下是虚幻引擎中描述的它们的定义。

*   `Suspension Force Offset` — 距施加悬挂力的位置的垂直偏移（沿 Z 轴）。
*   `Suspension Max Raise` — 车轮可以超出静止位置多远。
*   `Suspension Max Drop` — 车轮可以下降到静止位置以下多远。
*   `Suspension Natural Frequency` — 悬架的振荡频率。标准汽车的值介于`5`和之间`10`。
*   `Suspension Damping Ratio` — 能量从弹簧耗散的速率。标准汽车的值介于`0.8`和`1.2`之间。值`<1`更加迟缓，值`>1`更加动荡。
*   `Sweep Type` — 车轮悬架是否考虑简单、复杂或两者兼而有之。

![tuto_suspension_parameterization](img/tuto_suspension_parameterization.jpg)
<div style="text-align: center"><i>车轮蓝图中的悬架面板。</i></div>

!!! 笔记
    默认情况下，车辆的所有车轮在 Carla 中具有相同的参数化。以下解释将按车辆而不是按车轮进行介绍。 

---
## 悬架组 <span id="suspension-groups"></span>

根据其系统悬架，Carla 中的车辆可分为五组。组中的所有车辆都具有相同的参数化，因为它们预计在道路上具有相似的行为。车辆的悬架可以随意修改，不受这五组的限制。然而，了解这些并观察它们在模拟中的行为对于定义自定义悬架非常有用。

这五个组是：*轿跑车*、*越野车*、*卡车*、*城市车*和*货车*。通过仔细观察，这些组的参数化遵循特定的模式。


| 刚性悬架	 | 轿跑车            | 城市车            | 货车              | 越野         | 卡车            | 软悬架  |
| ---------------- | ---------------- | ---------------- | ---------------- | ---------------- | ---------------- | ---------------- |



<br>

当从软悬架转向硬悬架时，参数化有一些明显的趋势。

*   `Suspension Max Raise` 和 `Suspension Max Drop` 的 __减少__ — 刚性车辆应在平坦的道路上行驶且没有颠簸。出于空气动力学的考虑，底盘不应大幅移动，而应始终保持靠近地面。
*   `Suspension Damping Ratio` 的 __增加__ — 对于刚性车辆来说，减震器吸收的弹跳更大。

### 轿跑车 <span id="coupe"></span>

具有最硬悬架的车辆。


| 参数化   | 汽车       |
| -------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `Suspension Force Offset` — `0.0`<br>`Suspension Max Raise` — `7.5`<br>`Suspension Max Drop` — `7.5`<br>`Suspension Natural Frequency` — `9.5`<br>`Suspension Damping Ratio` — `1.0`<br>`Sweep Type` — `SimpleAndComplex`<br> | `vehicle.audi.tt`<br>`vehicle.lincoln.mkz2017`<br>`vehicle.mercedes-benz.coupe`<br>`vehicle.seat.leon`<br>`vehicle.tesla.model3`<br>                                                                                            |



### 越野 <span id="off-road"></span>

配备软悬架的车辆。

| 参数化    | 汽车     |
| ----------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `Suspension Force Offset` — `0.0`<br>`Suspension Max Raise` — `15.0`<br>`Suspension Max Drop` — `15.0`<br>`Suspension Natural Frequency` — `7.0`<br>`Suspension Damping Ratio` — `0.5`<br>`Sweep Type` — `SimpleAndComplex`<br> | `vehicle.audi.etron`<br>`vehicle.jeep.wrangler_rubicon`<br>`vehicle.nissan.patrol`<br>`vehicle.tesla.cybertruck`<br>     |


### 卡车 <span id="truck"></span>

具有最软悬架的车辆。 

| 参数化                                                                                                                                                                                                                          | 汽车                                                                                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Suspension Force Offset` — `0.0`<br>`Suspension Max Raise` — `17.0`<br>`Suspension Max Drop` — `17.0`<br>`Suspension Natural Frequency` — `6.0`<br>`Suspension Damping Ratio` — `0.4`<br>`Sweep Type` — `SimpleAndComplex`<br> | `vehicle.carlamotors.carlacola`<br>                                                                                                                                                                                                       |
<br>


### 城市车 <span id="urban"></span>

配备软悬架的车辆。

| 参数化                                                                                                                                                                                                                                                       | 汽车                                                                                                                                                                                                                                                               |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Suspension Force Offset` — `0.0`<br>`Suspension Max Raise` — `8.0`<br>`Suspension Max Drop` — `8.0`<br>`Suspension Natural Frequency` — `9.0`<br>`Suspension Damping Ratio` — `0.8`<br>`Sweep Type` — `SimpleAndComplex`<br>                                | `vehicle.audi.a2`<br>`vehicle.bmw.grandtourer`<br>`vehicle.chevrolet.impala`<br>`vehicle.citroen.c3`<br>`vehicle.dodge_charger.police`<br>`vehicle.mini.cooperst`<br>`vehicle.mustang.mustang`<br>`vehicle.nissan.micra`<br>`vehicle.toyota.prius`<br> |


<br>

### 货车 <span id="van"></span>

具有中地悬架的车辆。



| 参数化    | 汽车    |
| -------------------------------------------------------------------- | -------------------------------------------------------------------- |
| `Suspension Force Offset` — `0.0`<br>`Suspension Max Raise` — `9.0`<br>`Suspension Max Drop` — `9.0`<br>`Suspension Natural Frequency` — `8.0`<br>`Suspension Damping Ratio` — `0.8`<br>`Sweep Type` — `SimpleAndComplex`<br> |  `vehicle.volkswagen.t2`<br>    |




---

使用论坛发布有关此主题的任何疑问、问题或建议。 

<div class="build-buttons">
<p>
<a href="https://github.com/OpenHUTB/carla_doc/issues" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
讨论页面</a>
</p>
</div>

以下是本文之后的一些建议读物。

<div class="build-buttons">
<p>
<a href="../tuto_G_control_vehicle_physics" target="_blank" class="btn btn-neutral" title= "Set runtime changes on a vehicle physics.">
控制车辆物理</a>
</p>
<p>
<a href="../tuto_G_add_friction_triggers" target="_blank" class="btn btn-neutral" title= "Define dynamic box triggers for wheels.">
添加摩擦触发器</a>
</p>
<p>
<a href="../tuto_D_generate_colliders" target="_blank" class="btn btn-neutral" title="Create detailed colliders for vehicles">
生成详细的碰撞</a>
</p>
</div>
