# 在 Carla 包中导入地图

本节介绍将地图引入 __Carla 包（二进制）版本__ 的过程。如果您使用的是从源代码构建的 Carla 版本来导入地图，请遵循 [此处][source_ingest] 的指南。

此过程仅适用于 Linux 系统。导入过程涉及运行虚幻引擎的 Docker 映像来导入相关文件，然后将它们导出为独立包，然后可以将其配置为在 Carla 中使用。构建 Docker 镜像大约需要 4 小时和 600-700 GB 的时间。仅在第一次构建映像时才需要这样做。

- [__在你开始之前__](#before-you-begin)
- [__Carla 包中的地图导入__](#map-ingestion-in-a-carla-package)

---

## 在你开始之前

- 您需要满足以下系统要求：
    - Ubuntu 16.04+ 中的 64 位版本 [Docker](https://docs.docker.com/engine/install/) 
    - 至少 8GB RAM
    - 用于构建容器映像的至少 700 GB 可用磁盘空间
    - [Git](https://git-scm.com/downloads) 版本控制
- 确保您使用的是 Carla 的软件包（二进制）版本。如果您使用的是从源代码构建的 Carla 版本导入地图，请遵循 [此处][source_ingest] 的指南。
- 您应该至少使用地图编辑器（例如 RoadRunner）已 [生成][rr_generate_map] 的两个文件`<mapName>.xodr` 和 `<mapName>.fbx`。
- 这些文件应具有相同的 `<mapName>` 值，以便被识别为同一地图。


[source_ingest]: tuto_M_add_map_source.md
[import_map_package]: tuto_M_add_map_package.md
[rr_generate_map]: tuto_M_generate_map.md

---
## Carla 包中的地图导入

__1.__ Carla 提供了在 Docker 映像中构建虚幻引擎以及使用该映像编译 Carla 的所有实用程序。这些工具可以在 GitHub 上的源代码中找到。使用以下命令克隆存储库：

```sh
    git clone https://github.com/carla-simulator/carla
```

__2.__ 按照 [这些说明](https://github.com/carla-simulator/carla/tree/master/Util/Docker) 构建虚幻引擎的 Docker 映像。 

__3.__ 创建 `input_folder`。这是您放置要导入的文件的位置。 Docker 会自动创建一个 `.json` 文件来描述包文件夹结构。更改 `input_folder` 的权限才能成功创建：

```sh
    #Go to the parent folder, where the input folder is contained
    chmod 777 input_folder
```

> !!! 笔记
    如果包是 [手动准备的](tuto_M_manual_map_package.md) ，并且已包含 `.json` 文件，则无需执行此操作。

__4.__ 创建 `output_folder` 。这是 Docker 镜像在烘焙地图后写入输出文件的地方。

__5.__ 导航至 `~/carla/Util/Docker`。这是导入脚本所在的位置。该脚本需要 `input_folder` 和 `output_folder` 的路径以及要导入包的名称。如果提供了 `.json` 文件，则该文件的名称是包名称，如果未提供  `.json`，则名称必须为 `map_package`：

```sh
    python3 docker_tools.py --input ~/path_to_input_folder --output ~/path_to_output_folder --packages map_package
```

> !!! 警告
    如果未提供参数 `--packages map_package` ，Docker 镜像将制作 Carla 包。 

__6.__ 包将在 `output_folder` 中生成为 `<map_package>.tar.gz`. 。这是独立包，现在可以导入到 Carla 中。将包移动到 Carla 根目录（您将在其中使用地图的包/二进制版本）中的 `Import` 文件夹，然后从根目录运行以下脚本将其导入：

```sh
        ./ImportAssets.sh
```

__7.__ 要使用新地图运行仿真，请运行 Carla，然后使用 `config.py` 文件更改地图：

```sh
    cd PythonAPI/util
    python3 config.py --map <mapName>
```
<br>

---

Your map is now ready to run simulations in Carla. If you have any questions about the process then you can ask in the [forum](https://github.com/carla-simulator/carla/discussions) or you can try running some of our [example scripts](https://github.com/carla-simulator/carla/tree/master/PythonAPI/examples) on your new map to test it out.