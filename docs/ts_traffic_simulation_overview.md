# [Carla 中的交通模拟](https://carla.readthedocs.io/en/latest/ts_traffic_simulation_overview/) 

交通模拟是自动驾驶堆栈准确高效的训练和测试不可或缺的一部分。Carla 提供了许多不同的选项来模拟交通和特定的交通场景。本节概述了可用选项，可帮助您确定最适合您的用例的选项。

- [__交通管理器__](#traffic-manager)
- [__Scenario Runner 和 OpenScenario__](#scenario-runner-and-openscenario)
- [__Scenic__](#scenic)
- [__SUMO__](#sumo)
- [__Inverted AI__](#inverted_ai)

---

## 交通管理器 <span id="traffic-manager"></span>

交通管理器是 Carla 中的一个模块，用于从客户端控制模拟中的某些车辆。[`carla.Vehicle.set_autopilot`](https://carla.readthedocs.io/en/latest/python_api/#carla.Vehicle.set_autopilot) 车辆通过方法或注册到交通管理器command.SetAutopilot。每辆车的控制是通过 [不同阶段](adv_traffic_manager.md#stages) 的循环来管理的，每个阶段都在不同的线程上运行。

__用于：__

- 用真实的城市交通状况填充模拟。
- [自定义交通行为](adv_traffic_manager.md#general-considerations) 以设置特定的学习环境。 
- 开发与阶段相关的功能和数据结构，同时提高计算效率。

<div class="build-buttons">
<p>
<a href="https://openhutb.github.io/doc/tuto_G_traffic_manager/" target="_blank" class="btn btn-neutral" title="Go to Traffic Manager">
转至交通管理器</a>
</p>
</div>

---

## Scenario Runner 和 OpenScenario <span id="scenario-runner-and-openscenario"></span>

Scenario Runner 提供开箱即用的 [预定义交通场景](https://carla-scenariorunner.readthedocs.io/en/latest/list_of_scenarios/) ，还允许用户使用 Python 或 [OpenSCENARIO 1.0 标准](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_foreword) [定义自己](https://carla-scenariorunner.readthedocs.io/en/latest/creating_new_scenario/) 的场景。

OpenSCENARIO 的主要用途是描述涉及多辆车的复杂操作。用户可以在 [这里](https://carla-scenariorunner.readthedocs.io/en/latest/openscenario_support/) 查看 Scenario Runner 支持 OpenSCENARIO 的哪些功能。这些功能包括演习、动作、条件、故事和故事板。

Scenario Runner 必须与主 Carla 包 [分开](https://github.com/carla-simulator/scenario_runner) 安装。

__用于：__

- 创建复杂的交通场景和路线，为自动驾驶代理做好准备，以便在 [Carla 排行榜](https://leaderboard.carla.org/) 上进行评估。
- 定义可针对场景模拟的记录运行的定制 [指标](https://carla-scenariorunner.readthedocs.io/en/latest/metrics_module/) ，从而无需重复运行模拟。

<div class="build-buttons">
<p>
<a href="https://carla-scenariorunner.readthedocs.io" target="_blank" class="btn btn-neutral" title="Go to Scenario Runner">
转到 Scenario Runner</a>
</p>
</div>

---



## Scenic <span id="scenic"></span>

[Scenic](https://scenic-lang.readthedocs.io) 是一种特定领域的概率编程语言，用于对机器人和自动驾驶汽车等网络物理系统的环境进行建模。Scenic 提供了一个 [专门领域](https://scenic-lang.readthedocs.io/en/latest/modules/scenic.simulators.carla.html) 来促进 Scenic 脚本在 模拟器上的执行。

Scenic 场景定义易于阅读和构建。 [此处](tuto_G_scenic.md) 提供了创建简单场景的教程。

__用于：__

- 使用单个场景定义生成多个不同的场景。
- 为动态代理定义概率策略，以便随着时间的推移采取行动以响应世界状况。

<div class="build-buttons">
<p>
<a href="https://carla.readthedocs.io/en/latest/tuto_G_scenic/" target="_blank" class="btn btn-neutral" title="Go to Scenic Tutorial">
转至 Scenic 教程</a>
</p>
</div>

---

## SUMO <span id="sumo"></span>

[SUMO](https://sumo.dlr.de/docs/SUMO_at_a_Glance.html) 是一种开源、微观、多模式交通模拟。在 SUMO 中，每辆车都经过明确建模，有自己的路线，并通过网络单独移动。默认情况下，模拟是确定性的，但有多种选项可以引入随机性。

Carla 提供了与 SUMO 的联合模拟功能，允许在两个模拟器之间分配任务。车辆可以通过 SUMO 在 Carla 中产生，并由 SUMO 进行管理，就像交通管理器所做的那样。

__用于：__

- 在一个软件包中利用 Carla 和 SUMO 的功能。

<div class="build-buttons">
<p>
<a href="https://carla.readthedocs.io/en/latest/adv_sumo/" target="_blank" class="btn btn-neutral" title="Go to SUMO Co-Simulation">
转至 SUMO 协同模拟</a>
</p>
</div>

---
## Inverted AI <span id="inverted_ai"></span>

[Carla 0.10.0](ue5/index.md) 引入了基于生成式人工智能的交通模拟技术 [Inverted AI](https://github.com/inverted-ai) 。

---


如果您对 Carla 中模拟交通的不同选项有任何疑问，请随时在论坛或 [Discord](https://discord.gg/8kqACuC) 中发帖。

<div class="build-buttons">
<p>
<a href="https://github.com/OpenHUTB/doc/issues" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
讨论页面</a>
</p>
</div>

