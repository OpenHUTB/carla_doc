# [自定义地图：交通信号灯和交通标志](https://carla.readthedocs.io/en/latest/tuto_M_custom_add_tl/)

本指南介绍了如何将交通信号灯和交通标志添加到自定义地图、配置每个交通信号灯和交通标志的影响区域，以及如何在路口将交通信号灯配置为一组。此选项仅适用于有权访问虚幻引擎编辑器的用户。

- [__交通信号灯__](#traffic-lights)
- [__交通标志__](#traffic-signs)
- [__下一步__](#next-steps)

---

## 交通信号灯 <span id="traffic-lights"></span>

要将交通信号灯添加到新地图：

__1.__ 从 _内容浏览器_，导航至 `Content/Carla/Static/TrafficLight/StreetLights_01`，您会发现几种不同的交通信号灯蓝图可供选择。

__2.__ 将交通信号灯拖到场景中并将其放置在所需位置。按键盘上的空格键可在放置、旋转和缩放工具之间切换。

__3.__ 通过在 _细节(Details)_ 面板中选择 _BoxTrigger_ 组件并调整 _变换(Transform)_ 部分中的值，调整每个交通信号灯的 [`trigger volume`][triggerlink] （位于交通信号灯的对面、用于设置交通信号灯影响最大范围的触发体积）。这将确定交通信号灯的影响区域。

>>![ue_trafficlight](./img/ue_trafficlight.jpg)

__4.__ 对于岔路口，将`BP_TrafficLightGroup`参与者拖入关卡中。通过将岔路口的所有交通信号灯添加到“细节(Details)”面板中的“Traffic Lights”数组，将它们分配给交通信号灯组。

>>![ue_tl_group](./img/ue_tl_group.jpg)

__5.__ 交通信号灯计时只能通过 Python API 进行配置。请参阅 [此处](core_actors.md#traffic-signs-and-traffic-lights) 的文档以获取更多信息。 

>>![ue_tlsigns_example](./img/ue_tlsigns_example.jpg)

> _例如：交通标志、交通信号灯和转弯停车。_

[triggerlink]: python_api.md#carla.TrafficSign.trigger_volume

示例：[交通信号灯的配置和使用](tuto_G_traffic_light.md) 。

---

## 红绿标志 <span id="traffic-signs"></span>

要将交通信号灯添加到新地图：

__1.__ 从 _内容浏览器(Content Browser)_，导航至 `Content/Carla/Static/TrafficSign`。您会发现几种不同的交通标志蓝图可供选择。

__2.__ 将交通标志拖到场景中并将其放置在所需位置。按键盘上的空格键可在定位、旋转和缩放工具之间切换。

__3.__ 通过在“细节”面板中选择 _BoxTrigger_ 组件并调整 _Transform_ 部分中的值，调整每个交通标志的[`trigger volume`][triggerlink]。这将确定交通信号灯的影响区域。并非所有交通标志都有触发体积(volume)。此类标志包括让行标志、停车标志和限速标志。 

---

## 下一步 <span id="next-steps"></span>

使用以下工具和指南继续自定义您的地图：

- [在地图中实现子关卡。](tuto_M_custom_layers.md)
- [使用程序化构建工具添加建筑物。](tuto_M_custom_buildings.md)
- [使用道路画家工具自定义道路。](tuto_M_custom_road_painter.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用序列化网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以生成 [行人导航信息](tuto_M_generate_pedestrian_navigation.md) 。

---

如果您对文档有任何疑问，可以在 [讨论页面](https://github.com/OpenHUTB/carla_doc/issues) 中提问。
