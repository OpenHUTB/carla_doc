# 手动包准备

地图包遵循特定的文件夹结构，并且必须包含描述该结构的 `.json` 文件。我们的自动地图导入流程会自动创建此`.json`文件，但您也可以选择自行准备。包含您自己的`.json`文件将覆盖传递给`make import`命令的任何参数。

- [__标准地图__](#standard-maps)
    - [为标准地图创建文件夹结构](#create-the-folder-structure-for-the-standard-maps)
    - [为标准地图创建 JSON 描述](#create-the-json-description-for-the-standard-maps)
- [__大地图__](#large-maps)
    - [为大地图创建文件夹结构](#create-the-folder-structure-for-the-large-maps)
    - [为大型地图创建 JSON 描述](#create-the-json-description-for-the-large-maps)

---

## 标准地图
### 为标准地图创建文件夹结构

1. __在 `carla/Import` 里面创建一个文件夹。__ 文件夹的名称并不重要。

2. 为每个要导入的地图 __创建不同的子文件夹__。

3. __将每个地图的文件移动到相应的子文件夹中。__ 子文件夹将包含一组特定的元素： 

-   `.fbx` 文件中地图的网格。  
-   `.xodr` 文件中的 OpenDRIVE 定义。  
-   （可选）资产所需的纹理。  

例如，一个包含两个地图的包的 `Import` 文件夹应具有类似于以下结构的结构。

```sh
Import
│
└── Package01
  ├── Package01.json
  ├── Map01
  │   ├── Asphalt1_Diff.jpg
  │   ├── Asphalt1_Norm.jpg
  │   ├── Asphalt1_Spec.jpg
  │   ├── Grass1_Diff.jpg
  │   ├── Grass1_Norm.jpg
  │   ├── Grass1_Spec.jpg
  │   ├── LaneMarking1_Diff.jpg
  │   ├── LaneMarking1_Norm.jpg
  │   ├── LaneMarking1_Spec.jpg
  │   ├── Map01.fbx
  │   └── Map01.xodr
  └── Map02
      └── Map02.fbx
```

---

### 为标准地图创建 JSON 描述

在包的根文件夹中创建一个 `.json` 文件。在包之后命名文件。 请注意，这将是发行版名称。文件的内容将描述 __地图__ 和 __道具__ 的 JSON 数组，以及每个地图和道具的基本信息。 

__地图__ 需要以下参数：

- 地图的 __name__ 。这必须和 `.fbx` 和 `.xodr` 文件相同。 
- `.fbx` 文件的 __source__ 路径。  
- __use_carla_materials__. 如果为 __True__，地图将使用 Carla 材质。否则，它将使用 RoadRunner 材质。
- __xodr__ `.xodr` 文件的路径。  

__Props__ 不属于本教材的一部分。该字段将留空。还有另一个关于如何 [添加新道具](tuto_A_add_props.md) 的教程。  

生成的 `.json` 文件应类似于以下内容：

```json
{
  "maps": [
    {
      "name": "Map01",
      "source": "./Map01/Map01.fbx",
      "use_carla_materials": true,
      "xodr": "./Map01/Map01.xodr"
    },
    {
      "name": "Map02",
      "source": "./Map02/Map02.fbx",
      "use_carla_materials": false,
      "xodr": "./Map02/Map02.xodr"
    }
  ],
  "props": [
  ]
}
```

---

如果您对流程有任何疑问，可以在 [论坛](https://github.com/carla-simulator/carla/discussions) 中提问。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
Carla 论坛</a>
</p>
</div>