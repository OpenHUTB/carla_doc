# 自定义地图：程序化建筑

- [__程序化建筑__](#add-serial-meshes)
	- [建筑结构](#building-structure)
	- [结构修改](#structure-modifications)
- [__下一步__](#next-steps)

---

## 程序化建筑 <span id="add-serial-meshes"></span>

程序化建筑工具允许您创建由不同楼层组成的矩形建筑。每个关卡都是使用可配置的网格阵列或单个蓝图构建的。如果使用网格阵列，则每个网格将沿关卡随机重复以提供多样性。网格体创建一次，每个重复都将是该网格体的一个实例。这可以提高地图的性能。

### 建筑结构 <span id="building-structure"></span>

要开始建造您的建筑：

1. 在虚幻引擎编辑器的 _内容浏览器_ 中，导航至 `Content/Carla/Blueprints/LevelDesign`。
2. 将 `BP_Procedural_Building` 拖入场景中。

在 _Details_ 面板中，您将看到可用于自定义建筑物的所有选项。每次在此处进行更改时，随着关键网格体的更新，建筑物将从场景视图中消失。单击 _Create Building_ 以查看新结果，或启用自动创建以避免重复此步骤。

关键网格是建筑物结构的一部分。它们分为四类：

- __Base:__ 底层。
- __Body:__ 中层。
- __Top:__ 最高层。
- __Roof:__ 覆盖顶层的屋顶。

对于每个模型，除了屋顶 __Roof__ 之外，都有一个网格体填充地板的中心，还有一个 __Corner__ 网格体将放置在地板的两侧。下图代表了全局结构。

![bp_procedural_building_visual](./img/map_customization/BP_Procedural_Building_Visual.jpg)
<div style="text-align: center"><i>建筑结构的可视化。</i></div>

基本参数 __Base parameters__ 设置尺寸。

- __楼层数：__ 建筑物的楼层数。身体网格 __Body__ 的重复。
- __长度 X 和 长度 Y：__ 建筑物的长度和宽度。建筑物每一侧的中央网格重复。

![bp_procedural_building_full](./img/map_customization/BP_Procedural_Building_Full.jpg)
<div style="text-align: center"><i>BP_Procedural_Building 示例。</i></div>

### 结构修改 <span id="structure-modifications"></span>

还有一些额外的选项可以修改建筑物的总体结构。

- __Disable corners:__ 如果选择，将不使用角网格。
- __Use full blocks:__ 如果选择此选项，建筑物的结构每层将仅使用一个网格。每层楼不会出现任何角落或重复。
- __Doors:__ 出现在底层、位于中央网格物体正前方的网格物体。门的数量及其位置可以设置。`0` 是初始位置、`1`是下一个底层重复等。 
- __Walls:__ 替代建筑物一侧或多侧的网格。例如，平面网格可用于绘制建筑物的一侧。

![bp_procedural_building_extras](./img/map_customization/BP_Procedural_Building_Extras.jpg)
<div style="text-align: center"><i>左边是一座没有角、只有一扇门的建筑。 <br> 右侧是一座建筑物，其一侧应用了墙壁。墙壁是没有防火通道的纹理。</i></div>

---

## 下一步 <span id="next-steps"></span>

使用以下工具和指南继续自定义您的地图：

- [在地图中实现子蓝图。](tuto_M_custom_layers.md)
- [添加和配置交通信号灯和交通标志。](tuto_M_custom_add_tl.md)
- [使用道路画家工具自定义道路。](tuto_M_custom_road_painter.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用序列化网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以 [生成行人导航信息](tuto_M_generate_pedestrian_navigation.md) 。

---

如果您对文档有任何疑问，可以在 [讨论页面](https://github.com/OpenHUTB/doc/issues) 中提问。


