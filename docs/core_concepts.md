# 核心概念

本页面介绍了 Carla 的主要功能和模块。不同主题的详细说明可以在相应的页面中找到。

为了了解 API 中不同的类和方法，请查看[Python API参考](python_api.md)。

*   **[第一步](#first-steps)**
  *   **[世界和客户端](#1st-world-and-client)**
  *   **[演员和蓝图](#2nd-actors-and-blueprints)**
  *   **[地图和导航](#3rd-maps-and-navigation)**
  *   [**传感器和数据**](#4th-sensors-and-data)
  *   **[进阶步骤](#advanced-steps)**

!!! 重要
    **本文档引用的是Carla 0.9.X**. <br>
    与以前的版本(0.8.X)相比，API发生了重大变化。在这里可以找到关于这些版本的另一个文档[here](https://carla.readthedocs.io/en/stable/getting_started/). 

---
## 第一步

### 世界和客户端

__客户端__ 是用户运行的模块，用于请求仿真中的信息或更改。客户端使用IP和特定端口运行。它通过终端与服务器通信。可以有许多客户机同时运行。高级多客户端管理需要对 Carla 和**[同步](adv_synchrony_timestep.md)**有透彻的理解。

__世界__ 是代表仿真的对象。它作为一个抽象层，包含了生成角色、改变天气、获取当前世界状态等主要方法。每个仿真只有一个世界。当地图更改时，它将被销毁并替换为新的。 

### 演员和蓝图
演员是在仿真中扮演角色的任何东西。

*   车辆
*   行人
*   传感器
*   观众
*   交通信号和交通灯

__蓝图__ 是生成角色所必需的已经制作好的角色布局。基本上，是带有动画和一组属性的模型。其中一些属性可以由用户自定义，而另一些则不能。有一个**[Blueprint库](bp_library.md)**，其中包含所有可用的蓝图以及有关它们的信息。

### 地图和导航

__地图__ 是代表仿真世界的对象，主要是城镇。有八张地图可供选择。它们都使用opdrive 1.4标准来描述道路。

__道路、车道和路口__ 由Python API管理，以便从客户端访问。这些与 __waypoint__ 类一起使用，为车辆提供导航路径。

__交通标志__ 和 __交通灯__ 是可访问的[**carla.Landmark**](#python_api.md#carla.landmark)。这些地标对象包含关于它们的 OpenDRIVE 定义的信息。此外，仿真器在运行时会自动生成停车标志、让行标志和交通信号灯对象，使用 OpenDRIVE 文件中的信息。这些对象会放置在道路上，有边界框包围。一旦车辆进入它们的边界框，它们就会意识到这些对象。

### 传感器和数据

__传感器__ 等待一些事件发生，然后从仿真中收集数据。它们调用定义如何管理数据的函数。根据不同的情况，传感器会检索不同类型的传感器数据。

传感器是附着在父车辆上的参与者。它跟随车辆，收集周围环境的信息。可用的传感器由**[Blueprint库](bp_library.md)**中的蓝图定义。

*   相机(RGB，深度和语义分割)
*   碰撞检测器
*   Gnss 传感器
*   IMU传感器
*   激光雷达射线投射
*   车道侵入检测器
*   障碍检测器
*   雷达
*   RSS.  

---
## 进阶步骤  

Carla 提供了广泛的功能，超出了本仿真器介绍的范围。这里列出了一些最引人注目的。然而，在开始 __进阶步骤__ 之前，强烈建议阅读整个“第一步”部分。

*   **[OpenDRIVE独立模式](adv_opendrive.md)**。仅使用OpenDRIVE文件生成道路网格。允许在 Carla 中加载任何OpenDRIVE地图，无需创建资源。
*   **[PTV-Vissim协同仿真](adv_ptv.md)**。在 Carla 和PTV-Vissim交通仿真器之间运行同步仿真。
*   [**录制器**](adv_recorder.md)。保存仿真状态的快照，以便以精确的精度重新执行仿真。
*   **[渲染选项](adv_rendering_options.md)**。包括图形质量设置、离屏渲染和无渲染模式。
*   [**RSS**](adv_rss.md)（Responsibility Sensitive Safety）：集成用于根据安全检查修改车辆轨迹的责任敏感安全性**[C++库](https://github.com/intel/ad-rss-lib)**。
*   [**仿真时间和同步**](adv_synchrony_timestep.md)。关于仿真时间和服务器-客户端通信的所有内容。
*   **[SUMO协同仿真](adv_sumo.md)**：在 Carla 和SUMO交通仿真器之间运行同步仿真。
*   **[交通管理器](adv_traffic_manager.md)**：该模块负责所有设置为自动驾驶模式的车辆。它仿真城市中的交通，使仿真看起来像一个真实的城市环境。

---
这是对 Carla 基本知识的总结。下一步将更仔细地查看世界和连接到它的客户端。

继续阅读以了解更多。访问论坛发表任何疑问或建议，在阅读过程中想到的。 

<div text-align: center>
<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="CARLA forum">
Carla 论坛</a>
</p>
</div>
<div class="build-buttons">
<p>
<a href="../core_world" target="_blank" class="btn btn-neutral" title="1st. World and client">
世界和客户端</a>
</p>
</div>
</div>
