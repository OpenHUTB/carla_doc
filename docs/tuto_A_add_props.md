# [添加新道具](https://carla.readthedocs.io/en/latest/tuto_A_add_props/) 

除了地图和车辆之外，道具是场景中的资产。其中包括路灯、建筑物、树木等等。仿真器可以随时通过简单的过程导入新道具。这对于在地图中创建自定义环境非常有用。

* [__准备包__](#prepare-the-package)  
	*   [创建文件夹结构](#create-the-folder-structure)  
	*   [创建 JSON 描述](#create-the-json-description)  
*   [__导入到 Carla 包中__](#ingestion-in-a-carla-package)  
*   [__在源代码构建中导入__](#ingestion-in-a-build-from-source)  

---
## 准备包

### 创建文件夹结构

__1. 在 `carla/Import` 里面创建一个文件夹 .__ 文件夹的名称不相关。  

__2. 创建子文件夹。__ 应该有一个通用子文件夹来存放所有道具，并且在其中包含与要导入的道具一样多的子文件夹。

__3. 将各个道具的文件移动到相应的子文件夹中。__ 道具子文件夹将包含`.fbx` 网格物体，以及它所需的纹理（可选）。

例如，包含两个独立包的 `Import` 文件夹应具有类似于以下结构的结构。

```sh
Import
│
├── Package01
│   ├── Package01.json
│   └── Props
│       ├── Prop01
│       │   ├── Prop01_Diff.png
│       │   ├── Prop01_Norm.png
│       │   ├── Prop01_Spec.png
│       │   └── Prop01.fbx
│       └── Prop02
│           └── Prop02.fbx
└── Package02
    ├── Packag02.json
    └── Props
        └── Prop03
            └── Prop03.fbx
```

### 创建 JSON 描述

在包的根文件夹中创建一个 `.json` 文件。在包之后命名文件。请注意，这将是发行版名称。文件的内容将描述地图 __maps__ 和道具 __props__ 的 JSON 数组，以及每个地图和道具的基本信息。

__Maps__ 不是本教程的一部分，因此该定义将为空。有一个具体的教程来 [__添加新地图__](tuto_M_custom_map_overview.md) 。

__Props__ 需要以下参数。 

*   道具的 __name__ 。这必须与 `.fbx` 相同。
*   `.fbx` 的 __source__ 路径。
*   道具的 __size__ 估计。此处列出了可能的值。
	*   `tiny`  
	*   `small`  
	*   `medium`  
	*   `big`  
	*   `huge`  
*   语义分割的 __tag__ 。如果标签拼写错误，它将被读取为 `Unlabeled`. 
	*   `Bridge`
	*   `Building`
	*   `Dynamic`
	*   `Fence`
	*   `Ground`
	*   `GuardRail`
	*   `Other`
	*   `Pedestrian`
	*   `Pole`
	*   `RailTrack`
	*   `Road`
	*   `RoadLine`
	*   `SideWalk`
	*   `Sky`
	*   `Static`
	*   `Terrain`
	*   `TrafficLight`
	*   `TrafficSign`
	*   `Unlabeled`
	*   `Vegetation`
	*   `Vehicles`
	*   `Wall`
	*   `Water`

最后，`.json`应该与下面的类似。 

```json
{
  "maps": [
  ],
  "props": [
    {
      "name": "MyProp01",
      "size": "medium",
      "source": "./Props/Prop01/Prop01.fbx",
      "tag": "SemanticSegmentationTag01"
    },
    {
      "name": "MyProp02",
      "size": "small",
      "source": "./Props/Prop02/Prop02.fbx",
      "tag": "SemanticSegmentationTag02"
    }
  ]
}
```
!!! 警告
    具有相同名称的包将产生错误。

---
## 导入到 Carla 包中

这是用于将道具导入到 Carla 包（例如 Carla 0.9.8）中的方法。 

将创建虚幻引擎的 Docker 映像。它充当一个黑盒子，自动将包导入到 Carla 镜像中，并生成分发包。Docker 镜像需要 4 小时和 400GB 才能构建。然而，这仅是第一次需要。

__1. 构建虚幻引擎的 Docker 镜像。__ 请按照 [以下说明](https://github.com/carla-simulator/carla/tree/master/Util/Docker) 构建映像。

__2. 运行脚本来烘培道具。__ `~/carla/Util/Docker` 文件夹中有一个脚本，它与之前创建的 Docker 映像连接，并自动进行导入。它只需要输入和输出文件的路径以及要导入的包的名称。

```sh
python3 docker_tools.py --input ~/path_to_package --output ~/path_for_output_assets  --package=Package01
```

__3. 定位包__。 Docker 应该已经在输出路径中生成了包 `Package01.tar.gz`。这是资产的独立包。

__4. 将包导入 Carla。__  

*   __在 Windows 上，__ 将包解压到 `WindowsNoEditor` 文件夹中。

*   __在 Linux 上，__ 将包移动到 `Import` 文件夹，然后运行脚本将其导入。

```sh
cd Util
./ImportAssets.sh
```

!!! 笔记
    Linux 上有一个替代方案。将包移动到`Import`文件夹，并运行脚本`Util/ImportAssets.sh`以提取包。


---
## 在源代码构建中导入

这是从源代码将道具导入到源代码构建 Carla 的方法。


将读取 JSON 文件以将道具放入虚幻引擎 `Content` 中。此外，它将在包的`Config`文件夹中创建一个 `Package1.Package.json` 文件。这将用于定义蓝图库中的道具，并在 Python API 中公开它们。如果包作为 [独立包](tuto_A_create_standalone.md) 导出，也将使用它。

一切准备就绪后，运行命令。

```sh
make import
```

!!! 警告
    确保该包位于 Carla 的 `Import` 文件夹内。

---

这就是关于将新道具导入 Carla 的不同方法的全部信息。如果有任何疑问，请随时在论坛中发布。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
Carla 论坛</a>
</p>
</div>


