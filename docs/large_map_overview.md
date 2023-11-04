# 大型地图概述

- [__大地图概述__](#large-maps-overview)
- [__瓦片流__](#tile-streaming)
- [__休眠的参与者__](#dormant-actors)

---

## 大型地图概述

Carla 中的大型地图功能允许用户进行大范围的仿真。在 Carla 中，大地图被划分为不大于 2kmx2km 的方形瓦片。瓦片根据它们与自主车辆的接近度（流距离）流入和流出服务器。地图上的其他参与者也根据他们与自主车辆的流距离进行管理。

---

## 瓦片流

自主车辆对于地图瓦片的装载和卸载是不可或缺的。根据距自主车辆的流距离值，瓦片被流式传输进和传出服务器。例如，位于流距离之外的瓦片将不会在仿真中渲染，而将渲染流距离内的瓦片。渲染的瓦片将随着英雄车辆的移动而变化。


要将车辆设置为自主，请使用如下的 [`set_attribute`](python_api.md#carla.ActorBlueprint.set_attribute) 方法：

```py
blueprint.set_attribute('role_name', 'hero' )
world.spawn_actor(blueprint, spawn_point)
```

使用下面的代码片段设置流距离，以便在自主车辆的 2 公里半径内加载瓦片：

```py
settings = world.get_settings()
settings.tile_stream_distance = 2000
world.apply_settings(settings)
```

您还可以使用以下 `config.py` 命令设置流传输距离:

```sh
cd PythonAPI/util
python3 config.py --tile-stream-distance 2000
```

!!! 注意
    大型地图目前一次仅支持一辆自主车辆。

---

## 休眠参与者

大地图功能向 Carla 引入了休眠参与者的概念。休眠参与者仅存在于大地图的上下文中。休眠参与者是仿真中位于自主车辆的 __参与者活动距离__ 之外的非自主车辆参与者，例如，远离自主车辆的车辆。参与者活动距离可以等于或小于流距离。

如果参与者发现自己在自主车辆的参与者活动距离之外，它将进入休眠状态。参与者仍然存在，但不会被渲染。尽管仍然可以[设置](python_api.md#carla.Actor.set_location)位置和[变换](python_api.md#carla.Actor.set_transform)，但不会计算物理量（除非通过流量管理器以混合模式运行） 。一旦休眠的参与者再次进入自主车辆的参与者活动距离内，它将醒来，并且其渲染和物理将恢复正常。

由交通管理器控制的参与者具有不同的行为，可以在大地图中操作时进行配置。请阅读[交通管理器](adv_traffic_manager.md#traffic-manager-in-large-maps)文档以了解其工作原理。

参与者将进入休眠状态或在 [world.tick()](python_api.md#carla.World.tick) 时醒来。

要将参与者活动距离设置为自主车辆周围 2 公里半径：

```py
settings = world.get_settings()
settings.actor_active_distance = 2000
world.apply_settings(settings)
```

您还可以使用以下 `config.py` 方法设置参与者活动距离

```sh
cd PythonAPI/util
python3 config.py --actor-active-distance 2000
```

要检查参与者是否处于休眠状态，您可以使用 Python API：

```py
actor.is_dormant
```

---

If you have any questions about large maps, then you can ask in the [forum](https://github.com/carla-simulator/carla/discussions).

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
CARLA forum</a>
</p>
</div>
