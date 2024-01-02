## [内容创作 - 车辆材料](https://carla.readthedocs.io/en/latest/tuto_content_authoring_vehicles_materials/) 

将车辆作为基本资产导入并布置好网格和蓝图后，您现在需要向车辆添加材质，以促进虚幻引擎中的照片级真实感渲染，从而实现机器学习训练数据的最大保真度。

虚幻编辑器拥有全面的材质工作流程，有助于创建高度逼真的材质。然而，这确实显着增加了该过程的复杂性。因此，CARLA 提供了一个大型材料原型库供您使用，而无需从头开始。

### 将材料应用到您的车辆上

CARLA 提供了一种用于复制车辆光泽表面的原型材料，可以模仿多种不同类型的车辆喷漆工作和功能。打开虚幻编辑器并在内容浏览器中，找到`Content > Carla > Static > GenericMaterials > 00_MastersOpt`。基本材质称为*M_CarPaint_Master*。右键单击该材质并从上下文材质中选择*Create Material Instance*  。为其命名并将其移动到存储新车辆内容的文件夹中。

在虚幻编辑器中，将观察则会移动到靠近地板的一点，然后将车辆的骨架网格物体从内容浏览器拖到场景中，车辆的车身现在将出现在那里。

![add_model](img/tuto_content_authoring_vehicles/add_model.gif)

现在，在右侧的详细信息面板中，将新材质实例拖动到*Materials* 部分的*Element 0*位置。您将看到车身呈现出一种新的灰色、有光泽的材料特性。

![apply_material](img/tuto_content_authoring_vehicles/apply_material
.gif)

双击内容浏览器中的材质，我们就可以开始编辑参数了。这里有许多参数可以改变各种属性，这些属性对于模拟现实世界的汽车喷漆工作非常重要。最重要的参数如下。



__颜色(Color)__ - 颜色设置控制汽车的整体颜色。基色只是汽车的主要颜色，它将控制整体颜色：

![change_base_color](img/tuto_content_authoring_vehicles/change_base_color
.gif)

__透明涂层(Clear coat)__ - 透明涂层设置控制饰面的外观及其对光的反应。粗糙度使用纹理将缺陷应用到车辆表面，以更高的值散射更多的光，以创建哑光外观。建议进行微妙的调整和较低的值以获得逼真的外观。一般来说，汽车喷漆作业是光滑且反光的，但是，这种效果可能更广泛地用于对定制喷漆作业的专业哑光饰面进行建模。

![change_roughness](img/tuto_content_authoring_vehicles/change_roughness
.gif)

控制汽车“光泽度”的一个重要参数是透明涂层强度(*Clear Coat Intensity*)。接近 1 的高值将使涂层闪亮且有光泽。

__粒状表面(Orange peel)__ - 真实汽车的饰面（特别是面向一般市场的批量生产的汽车）往往存在缺陷，表现为油漆中的轻微波纹。橙皮效果仿真了这一点，使汽车看起来更真实。

![change_orange_peel](img/tuto_content_authoring_vehicles/change_orange_peel
.gif)

__薄片(Flakes)__ - 有些汽车的油漆工作中包含其他材料的薄片，例如金属或陶瓷，以使汽车具有*metallic*或 *pearlescant* 外观，增加额外的闪光和反射，以有吸引力的方式与光反应。薄片参数允许 CARLA 仿真这一点。为了模仿金属饰面，它会是  

![flakes](img/tuto_content_authoring_vehicles/flakes
.gif)

__灰尘(Dust)__ - 汽车的车身上经常会积聚油脂和灰尘，从而增加油漆的质感，影响其反射光线的方式。灰尘参数允许您在涂层上添加破坏块，以仿真粘附在油漆上的异物。

![dust](img/tuto_content_authoring_vehicles/dust
.gif)

