# 自定义地图：分层地图

利用自定义地图中的级别可以让多人同时在一张地图上工作。它还允许您在模拟过程中使用 Python API 在地图上加载和卸载图层，就像 [分层的 CARLA 地图](core_map.md#layered-maps) 一样。

本指南将解释如何添加新关卡、如何向关卡添加资源以及如何将关卡配置为始终加载或不加载。

- [__添加新关卡__](#add-a-new-level)
- [__将资产添加到关卡__](#add-assets-to-a-level)
- [__配置关卡加载选项__](#configure-level-loading-options)
- [__下一步__](#next-steps)

---

## 添加新关卡

地图中的所有新关卡都将嵌套在父关卡中，称为“持久关卡”。要创建新关卡：

__1. 打开关卡面板。__

1. 在虚幻编辑器中，从菜单栏中打开“窗口”。
2. 单击“关卡”。

__2. 创建新关卡。__

1. 在“关卡”面板中，单击“关卡”并选择“新建......”。
2. 选择“空关卡”。
3. 将关卡保存在 `Content/Carla/Maps/Sublevels/<map_name>/`。 要将关卡与 CARLA Python API 集成，请使用命名约定 `<map_name>_<layer_name>`，比如,  `TutorialMap_Buildings`。有关可用关卡的列表，请查看 [此处](core_map.md#carla-maps)。

>>>>>>![create new level](./img/new_level.png)

---

## 将资产添加到关卡

__1. 选择您要将资产添加到的关卡__。

在“关卡”面板中，双击要添加资源的“关卡”。通过切换锁图标确保关卡已解锁。 

__2. 选择要添加的资产。__

1. 选择您想要添加到关卡的所有资产。
2. 右键单击并转到“关卡”。
3. 单击 _将选择移动到当前关卡_。

__3. 保存关卡。__

如果某个关卡有待保存的更改，您将在“关卡”面板中看到该关卡旁边有一个铅笔图标。单击此按钮保存更改。

![moving assets](./img/move_assets.png)

---

## 配置关卡加载选项

关卡可以配置为能够切换或始终加载。要配置任一选项的级别：

1. 右键单击“关卡”面板中的关卡，然后转到“修改流送方法”。
2. 选择所需的设置：
    1. _固定加载_：__无法__ 通过 Python API 切换关卡。
    2. _蓝图_: __可以__ 通过 Python API 切换关卡。关卡名称旁边会出现一个蓝点。

无论设置如何，您仍然可以通过按眼睛图标来切换编辑器中的蓝图。

>>>>>>![levels](./img/levels.png)

---

## 下一步

使用以下工具和指南继续自定义您的地图：

- [添加和配置交通灯和标志。](tuto_M_custom_add_tl.md)
- [A使用程序构建工具添加建筑物。](tuto_M_custom_buildings.md)
- [使用道路画家工具自定义道路。](tuto_M_custom_road_painter.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用连续网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以 [生成行人导航信息](tuto_M_generate_pedestrian_navigation.md)。

---

如果您对流程有任何疑问，可以在 [论坛](https://github.com/carla-simulator/carla/discussions)中提问。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
Carla 论坛</a>
</p>
</div>
