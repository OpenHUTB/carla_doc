# 从源代码构建的 Carla 中导入地图

本节描述从源代码构建的 CARLA 中导入地图的过程。如果您使用 CARLA 的包（二进制）版本来导入地图，请按照 [此处][package_ingest] 的指南进行操作。

导入过程涉及通过将相关地图文件编译到包中来导入它们。然后可以在虚幻引擎编辑器中打开该包并进行自定义，然后生成行人导航文件并最终将其添加到包中。


[package_ingest]: tuto_M_add_map_package.md

- [__在你开始之前__](#before-you-begin)
- [__地图导入__](#map-ingestion)
- [__下一步__](#next-steps)

---

## 在你开始之前

- 确保您使用的是从源代码构建的 CARLA 版本。如果您使用的是 CARLA 的打包（二进制）版本，请按照 [此处][import_map_package] 的教程进行操作。
- 您应该至少有两个文件，`<mapName>.xodr`和`<mapName>.fbx` ，是由地图编辑器（例如 RoadRunner）生成的。
- 这些文件应具有相同的值`<mapName>`，以便被识别为相同的地图。
- 您可以将多个地图提取到同一个包中。每张地图都应该有一个唯一的名称。

[import_map_package]: tuto_M_add_map_package.md
[rr_generate_map]: tuto_M_generate_map.md

---
## 地图导入

__1.__ 将需要导入的地图文件放入 CARLA 根目录下的`Import`文件夹中。

__2.__ 运行以下命令来提取文件：

```sh
make import
```

__请注意，可以设置两个可选参数标志__:

- `--package=<package_name>` 指定包的名称。默认情况下，此项设置为`map_package`。两个包不能具有相同的名称，因此使用默认值将导致后续摄取时出错。__强烈建议更改包的名称__。通过运行以下命令来使用此标志：

```sh
make import  ARGS="--package=<package_name>"
```

- `--no-carla-materials` 指定您不想使用默认的 CARLA 材质（道路纹理等）。您将改用 RoadRunner 材料。__仅当您不提供__ [自己的`.json`文件](tuto_M_manual_map_package.md) 时才需要此标志。`.json`文件中的任何值都将覆盖此标志。通过运行以下命令来使用此标志：

```sh
make import  ARGS="--no-carla-materials"
```

将使用地图包的名称创建一个文件夹 `Unreal/CarlaUE4/Content` 。它将包含配置文件、超速挡、静态资产信息和导航信息。

---

## 下一步

您现在可以在虚幻编辑器中打开地图并运行仿真。从这里，您将能够自定义地图并生成行人导航数据。我们建议在所有自定义完成后生成行人导航，这样就不会有障碍物阻挡行人路径。

CARLA 提供了多种工具和指南来帮助自定义地图：

- [在地图中实现子关卡。](tuto_M_custom_layers.md)
- [添加和配置交通灯和标志。](tuto_M_custom_add_tl.md)
- [使用程序化构建工具添加建筑物。](tuto_M_custom_buildings.md)
- [使用道路画家工具自定义道路。](tuto_M_custom_road_painter.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用序列化网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以 [生成行人导航信息](tuto_M_generate_pedestrian_navigation.md) 。

---

如果您对流程有任何疑问，可以在 [论坛](https://github.com/carla-simulator/carla/discussions) 中提问。
