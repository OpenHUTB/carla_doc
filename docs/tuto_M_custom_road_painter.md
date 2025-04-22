# 自定义地图：道路画家

本指南解释了什么是道路绘制工具，如何使用它通过组合不同的纹理来自定义道路的外观，如何添加贴花和网格以及如何根据道路纹理更新车道标记的外观。

- [__什么是道路画家？__](#what-is-the-road-painter)
- [__在你开始之前__](#before-you-begin)
- [__建立道路画家、掌握材质和渲染目标__](#establish-the-road-painter-master-material-and-render-target)
- [__准备主材质__](#prepare-the-master-material)
    - [__描绘道路__](#paint-the-road)
- [__更新车道线的外观__](#update-the-appearance-of-lane-markings)
- [__下一步__](#next-steps)

---

## 什么是道路画家？ <span id="what-is-the-road-painter"></span>

道路画家工具是一个使用 OpenDRIVE 信息快速绘制道路的蓝图。它采用主材质并将其应用到道路的渲染目标以用作画布。主材质由一系列材质组成，这些材质可以使用画笔混合并用作蒙版。无需应用光度测定技术，也无需考虑几何体的 UV。

---

## 在你开始之前 <span id="before-you-begin"></span>

道路油漆工使用 OpenDRIVE 信息来油漆道路。确保您的`.xodr`文件与地图同名，这样才能正常工作。

---

## 建立道路画家、掌握材质和渲染目标 <span id="establish-the-road-painter-master-material-and-render-target"></span>

__1. 创建 `RoadPainter` 参与者。__

1. 在 _内容浏览器_ 中，导航至 `Content/Carla/Blueprints/LevelDesign`。
2. 将 `RoadPainter` 拖入场景中。

__2. 创建渲染目标。__

1. 在 _内容浏览器_ 中，导航至 `Content/Carla/Blueprints/LevelDesign/RoadPainterAssets`。
2. 右键单击 `RenderTarget` 文件并选择 `Duplicate`。
3. 重命名为 `Tutorial_RenderTarget`。

__3. 创建主材质实例。__

1. 在 _内容浏览器_ 中，导航至 `Game/Carla/Static/GenericMaterials/RoadPainterMaterials`。
2. 右键单击 `M_RoadMaster` 并选择 _Create Material Instance_ 。
3. 重命名为 `Tutorial_RoadMaster`。

__4. 重新校准 _Map Size (Cm)_ ，使其等于地图的实际尺寸。__

1. 选择 `RoadPainter` 场景中的参与者。
2. 转到 _Details_ 面板并按 _Z-Size_ 按钮。您将看到 _Map Size (Cm)_ 中的值发生变化。

>>>>>![map size](./img/map_size.png)

__5. 同步 `RoadPainter` 和 `Tutorial_RoadMaster` 之间的地图尺寸。__

1. 在 _内容浏览器_ 中，打开 `Tutorial_RoadMaster`。
2. 复制上一步中 _Map Size (Cm)_ 的值，并将其粘贴到 `Tutorial_RoadMaster` 窗口中的 _Global Scalar Parameter Values -> Map units (CM)_ 。
3. 点击保存。

>>>>>>![img](./img/map_size_sync.png)

__6. 创建道路画家和主材质之间的通信链接。__

`Tutorial_RenderTarget` 将是道路画家和 `Tutorial_RoadMaster` 之间的通信纽带。

1. 在 `Tutorial_RoadMaster`窗口中，将 `Tutorial_RenderTarget` 应用于 _Global Texture Parameter Values -> Texture Mask_ 。
2. 保存并关闭。
3. 在主编辑器窗口中，选择道路画家参与者，转到 _Details_ 面板并将 `Tutorial_RenderTarget` 应用到 _Paint -> Render Target_ 。

---

## 准备主材质 <span id="prepare-the-master-material"></span>

您创建的 `Tutorial_RoadMaster` 材质，包含基础材质、额外材质信息以及将通过您的`Tutorial_RenderTarget`。您可以配置一种基础材料和最多三种附加材料。

>>![master materials](./img/master_material.png)

要配置材料，请双击 `Tutorial_RoadMaster` 文件。在出现的窗口中，您可以根据需要为每种材料选择和调整以下值：

- 亮度(Brightness)
- 色调(Hue)
- 饱和度(Saturation)
- AO Intensity
- 法线贴图强度(NormalMap Intensity)
- 粗糙度对比(Roughness Contrast)
- 粗糙度强度(Roughness Intensity)

您可以通过选择以下值并在搜索框中搜索纹理来更改每种材质的纹理：

- Diffuse
- Normal
- ORMH

探索 `Game/Carla/Static/GenericMaterials/Asphalt/Textures` 中可用的一些 Carla 纹理。

---

### 描绘道路 <span id="paint-the-road"></span>

__1. 在道路画家和道路之间创建链接。__

1. 在主编辑器窗口中，在 _World Outliner_ 搜索框中进行 `Road_Road` 搜索。
2. 按 `Ctrl + A` 选择所有道路。
3. 在 _Details_ 面板中，转到 _Materials_ 部分并应用 `Tutorial_RoadMaster` 到 _Element 0_、 _Element 1_、 _Element 2_ 和 _Element 3_。

__2. 选择要定制的材质。__

我们添加的 `Tutorial_RoadMaster` 单独应用于道路，并使用画笔工具配置应用程序。要应用和自定义材质：

1. 选择道路油画家参与者。
2. 在 _Details_ 面板中，在 _Mask Color_ 下拉菜单中选择要使用的材质。

>>>>>>![choose material](./img/choose_material.png)

__3. 设置画笔和模板参数。__

在 `GenericMaterials/RoadStencil/Alphas` 中有多种模板可供选择。模板用于根据您的需要绘制道路，可以使用以下值进行调整：

- _Stencil size_ — 画笔的尺寸。
- _Brush strength_ — 轮廓的粗糙度。
- _Spacebeween Brushes_ — 笔划之间的距离。
- _Max Jitter_ — 笔划之间画笔的大小变化。
- _Stencil_ — 要使用的画笔。
- _Rotation_ — 应用于笔画的旋转。

>>>>>>![materials_roadpaint_brushes](./img/material_customization/Materials_Brush.jpg)
<div style="text-align: center"><i>刷子面板</i></div>
<br>
![materials_roadpaint_typesofbrushes](./img/material_customization/Materials_Road_Typesofbrushes.jpg)
<div style="text-align: center"><i>Different types of brushes.</i></div>
<br>
__4. 将每种材料涂抹到道路的所需部分。__

通过 _Details_ 面板的 _Default_ 部分中的按钮选择应用所选材质的位置：

* _Paint all roads_ — 粉刷所有道路。
* _Paint by actor_ — 绘画特定的、选定的参与者。
* _Paint over circle_ — 使用圆形图案进行绘制，有助于提供变化。
* _Paint over square_ — 使用正方形图案进行绘制，有助于提供变化。

此部分还包含用于删除已应用更改的选项。

* _Clear all_ — 擦除所有绘制的材质。
* _Clear materials_ — 删除当前活动的材料。
* _Clear material by actor_ — 删除最接近所选参与者的材质。

>>>>>>![materials_roadpaint_brushes](./img/material_customization/Materials_RoadPainter_Default.jpg)
<div style="text-align: center"><i>Different painting and erasing options.</i></div>
<br>
__5. 添加贴花和网格。__

您可以在 `Content/Carla/Static/Decals` 和 `Content/Carla/Static` 中探索可用的贴花和网格。通过扩展并添加到 _Decals Spawn_ 和 _Meshes Spawn_ 数组，将它们添加到道路绘制器中。对于每一项，您都可以配置以下参数：

- _Number of Decals/Meshes_ - 要绘制的每个贴花或网格的数量。
- _Decal/Mesh Scale_ — 每个轴的贴花/网格比例。
- _Fixed Decal/Mesh Offset_ — 每个轴与车道中心的偏差。
- _Random Offset_ — 每个轴与车道中心的最大偏差。
- _Decal/Mesh Random Yaw_ — 最大随机偏航旋转。
- _Decal/Mesh Min Scale_ — 应用于贴花/网格的最小随机比例。
- _Decal/Mesh Max Scale_ — 应用于贴花/网格的最大随机比例。

>>>>>>![materials_](./img/decals_meshes.png)
<div style="text-align: center"><i>贴花和网格面板。</i></div>
<br>

配置好网格和贴花后，通过按`Spawn decals`和生成它们`Spawn meshes`。

!!! 笔记
    确保网格和贴花没有启用碰撞，以免干扰道路上的汽车并将任何边界框降低到道路水平。

__7. 尝试获得您想要的外观。__ 

尝试不同的材质、纹理、设置、贴花和网格以获得您想要的外观。下面是一些示例图像，显示了在绘制每种材质的过程中道路外观如何变化。

![materials_roadpaint_mat00](./img/material_customization/Materials_Road_MaterialBase.jpg)
<div style="text-align: center"><i>基础道路材料示例。</i></div>
<br>

![materials_roadpaint_mat01](./img/material_customization/Materials_Road_Material1.jpg)
<div style="text-align: center"><i>应用材料 1 后的示例。</i></div>
<br>

![materials_roadpaint_mat02](./img/material_customization/Materials_Road_Material2.jpg)
<div style="text-align: center"><i>应用材料 2 后的示例。</i></div>
<br>

![materials_roadpaint_mat03](./img/material_customization/Materials_Road_Material3.jpg)
<div style="text-align: center"><i>应用材料 3 后的示例。</i></div>
<br>

![materials_roadpaint_mat03](./img/material_customization/Materials_Road_Decals.jpg)
<div style="text-align: center"><i>应用贴花后的示例。</i></div>
<br>

![materials_roadpaint_mat03](./img/material_customization/Materials_Road_Meshes.jpg)
<div style="text-align: center"><i>应用网格后的示例。</i></div>
<br>

---

## 更新车道线的外观 <span id="update-the-appearance-of-lane-markings"></span>

绘制道路后，您可以按照以下步骤更新道路标记的外观：

__1.  复制主材料。__

1. 在 _内容浏览器_ ，导航至 `Game/Carla/Static/GenericMaterials/RoadPainterMaterials`。
2. 右键单击 `Tutorial_RoadMaster` 并选择 _Create Material Instance_ 。
3. 重命名为 `Tutorial_LaneMarkings`。

__2. 配置车道线材质。__

1. 在 _内容浏览器_ ，双击 `Tutorial_LaneMarkings`。
2. 在 _Details_ 面板中，转到 _Global Static Switch Parameter Values_ 部分，然后选中 _LaneMark_ 左侧和右侧的框。
3. 转到 _Texture_ 部分并选中 _LaneColor_ 和 _Uv Size_ 复选框。
4. 在 _LaneColor_ 中选择您喜欢的车道标记颜色。
5. _保存_ 并关闭。

__3. 选择车道线网格。__

将材料拖到您想要着色的车道线上。如果需要，对不同颜色的车道线重复整个过程。

---

## 下一步 <span id="next-steps"></span>

使用以下工具和指南继续自定义您的地图：

- [在地图中实现子关卡。](tuto_M_custom_layers.md)
- [添加和配置交通灯和标志。](tuto_M_custom_add_tl.md)
- [使用程序化构建工具添加建筑物。](tuto_M_custom_buildings.md)
- [自定义天气](tuto_M_custom_weather_landscape.md#weather-customization)
- [使用序列化网格自定义景观。](tuto_M_custom_weather_landscape.md#add-serial-meshes)

完成定制后，您可以 [生成行人导航信息](tuto_M_generate_pedestrian_navigation.md) 。

---

如果您对文档有任何疑问，可以在 [讨论页面](https://github.com/OpenHUTB/carla_doc/issues) 中提问。


