# [创建资产发行包](https://carla.readthedocs.io/en/latest/tuto_A_create_standalone/) 

使用独立包管理资产是 Carla 的常见做法。将它们放在一边可以减少构建的大小。这些资源包可以随时轻松导入到 Carla 包中。它们对于以有组织的方式轻松分配资产也非常有用。

- [__在源代码构建的 Carla 中导出包__](#export-a-package-from-the-ue4-editor)  
- [__使用 Docker 导出包__](#export-a-package-using-docker)
- [__将资源导入 Carla 包__](#import-assets-into-a-carla-package)  

---
## 在源代码构建的 Carla 中导出包 <span id="export-a-package-from-the-ue4-editor"></span>

将资产导入虚幻后，用户可以为其生成 __独立的包__。这将用于将内容分发到 Carla 包，例如 0.9.8。

要导出包，只需运行以下命令即可。

```sh
make package ARGS="--packages=Package1,Package2"
```

这将为列出的每个包创建一个压缩在 `.tar.gz` 文件中的独立包。在 Linux 上文件将保存在 `Dist`，在 Windows 上文件保存在 `/Build/UE4Carla/` 文件夹下。

!!! 笔记
    在虚幻编辑器的菜单“编辑->项目设置->项目->打包->(点下三角打开折叠的选项)打包版本中要包含的地图列表”中可以删除不需要的地图，加入需要打包的自定义地图。

---

## 使用 Docker 导出包 <span id="export-a-package-using-docker"></span>

虚幻引擎和 Carla 可以构建在 Docker 映像中，然后可以使用该映像创建包或导出资源以在包中使用。

要创建 Docker 映像，请按照 [此处](build_docker_unreal.md) 的教程进行操作。

准备好镜像后：

1. 导航至 `Util/Docker`。
2. 通过运行以下命令之一创建 Carla 包或准备在包中使用的资源：

```sh
# 创建独立的包
./docker_tools.py --output /output/path

# 烘培在 Carla 包中使用的资产
./docker_tools.py --input /assets/to/import/path --output /output/path --packages PkgeName1,PkgeName2
```

---
## 将资源导入 Carla 包 <span id="import-assets-into-a-carla-package"></span>

独立包包含在`.tar.gz`文件中。提取方式取决于平台。

*   __在 Windows 上，__ 将压缩文件解压到主根 Carla 文件夹中。
*   __在 Linux 上，__ 将压缩文件移至`Import`文件夹并运行以下脚本。

```sh
cd Import
./ImportAssets.sh
```

!!! 笔记
    独立包无法直接导入到 Carla 构建中。按照教程导入 [道具](tuto_A_add_props.md) 、[地图](tuto_M_custom_map_overview.md) 或 [车辆](tuto_A_add_vehicle.md)。

---

这总结了如何在 Carla 中创建和使用独立包。如果有任何意外问题，请随时在论坛中发帖。

<div class="build-buttons">
<p>
<a href="https://github.com/OpenHUTB/carla_doc/issues" target="_blank" class="btn btn-neutral" title="Go to the CARLA forum">
讨论页面</a>
</p>
</div>