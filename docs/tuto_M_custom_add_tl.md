# 自定义地图：交通灯和标志

本指南介绍了如何将交通灯和标志添加到自定义地图、配置每个交通灯和标志的影响区域，以及如何在路口将交通灯配置为一组。此选项仅适用于有权访问虚幻引擎编辑器的用户。

- [__红绿灯__](#traffic-lights)
- [__交通标志__](#traffic-signs)
- [__下一步__](#next-steps)

---

## 红绿灯

要将红绿灯添加到新地图：

__1.__ 从 _内容浏览器_，导航至 `Content/Carla/Static/TrafficLight/StreetLights_01`。您会发现几种不同的交通灯蓝图可供选择。

__2.__ 将交通灯拖到场景中并将其放置在所需位置。按键盘上的空格键可在定位、旋转和缩放工具之间切换。

__3.__ 通过在 _Details_ 面板中选择 _BoxTrigger_ 组件并调整 _Transform_ 部分中的值，调整每个交通灯的 [`trigger volume`][triggerlink] 。这将确定交通灯的影响区域。 

>>![ue_trafficlight](./img/ue_trafficlight.jpg)

__4.__ 对于连接点，将`BP_TrafficLightGroup`参与者拖入关卡中。通过将交汇处的所有交通灯添加到“详细信息”面板中的“交通灯”数组，将它们分配给交通灯组。

>>![ue_tl_group](./img/ue_tl_group.jpg)

__5.__ 交通灯计时只能通过 Python API 进行配置。请参阅 [此处](core_actors.md#traffic-signs-and-traffic-lights) 的文档以获取更多信息。 

>>![ue_tlsigns_example](./img/ue_tlsigns_example.jpg)

> _例如：交通标志、交通信号灯和转弯停车。_

[triggerlink]: python_api.md#carla.TrafficSign.trigger_volume

---

## 红绿灯标志

要将红绿灯添加到新地图：

__1.__ 从 _Content Browser_，导航至 `Content/Carla/Static/TrafficSign`。您会发现几种不同的交通灯蓝图可供选择。

__2.__ 将交通灯拖到场景中并将其放置在所需位置。按键盘上的空格键可在定位、旋转和缩放工具之间切换。

__3.__ 通过在“详细信息”面板中选择 _BoxTrigger_ 组件并调整 _Transform_ 部分中的值，调整[`trigger volume`][triggerlink]每个交通标志的。这将确定交通灯的影响区域。并非所有交通标志都有触发音量。此类标志包括让行标志、停车标志和限速标志。 

---

## 下一步

使用以下工具和指南继续自定义您的地图：

- [在地图中实现子关卡。](tuto_M_custom_layers.md)
- [使用程序化构建工具添加建筑物。](tuto_M_custom_buildings.md)
- [使用道路画家工具自定义道路。](tuto_M_custom_road_painter.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用序列化网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以生成 [行人导航信息](tuto_M_generate_pedestrian_navigation.md) 。

---

如果您对流程有任何疑问，可以在 [论坛](https://github.com/carla-simulator/carla/discussions) 中提问。
