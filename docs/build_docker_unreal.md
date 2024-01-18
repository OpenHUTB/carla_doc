# 在 Docker 中构建虚幻引擎和 Carla

本指南解释了如何使用 Docker 从头开始构建虚幻引擎和 Carla。然后，生成的映像可用于创建 Carla 包或准备在 Carla 包中使用的资源。此过程不应与用于在多个服务器或没有显示器的情况下运行 Carla 的预构建 Carla Docker 映像相混淆。可以在 [此处](build_docker.md) 找到相关文档。

- [__在你开始之前__](#before-you-begin)
    - [__系统要求__](#system-requirements)
    - [__软件要求__](#software-requirements)
- [__构建镜像__](#building-the-images)
- [__下一步：打包__](#next-steps-packages)

---

## 在你开始之前

##### 系统要求

您需要满足以下系统要求：

- 64位版本的Docker是Ubuntu 16.04+
- 至少 8GB 内存
- 用于初始容器构建过程的至少 600GB 可用磁盘空间

##### 软件要求

__Docker:__ 

按照 [此处](https://docs.docker.com/engine/install/) 的安装说明安装 Docker 。

__Python__: 


您需要安装 Python 3.6 或更高版本并在系统路径中正确设置。有关安装说明和 Python 文档，请查看 [此处](https://www.python.org/downloads/) 。

__虚幻引擎 GitHub 访问__:

从版本 0.9.12 开始，Carla 使用虚幻引擎 4.26 的修改版。该分支包含 Carla 特定的补丁。这将在 Docker 构建过程中下载。对于此下载，您需要有一个与虚幻引擎帐户关联的 GitHub 帐户。如果您没有进行此设置，请先按照 [本指南](https://www.unrealengine.com/en-US/ue4-on-github) 进行操作，然后再继续操作。在构建过程中您需要登录您的帐户。或者直接从 [开源湖工商仓库](https://github.com/OpenHUTB/engine) 进行下载。

__Carla:__

为 Carla 构建虚幻引擎以及 Carla 本身所需的 Dockerfile 和工具位于`Util/DockerCARLA` 源存储库的目录中。

如果您还没有，请使用以下命令下载存储库：

```sh
git clone https://github.com/carla-simulator/carla
```

---

## 构建镜像

以下步骤将花费很长时间。

__1. 构建 CARLA 先决条件镜像。__

以下命令将使用`Prerequisites.Dockerfile`构建一个名为`carla-prerequisitesusing`的映像。在此版本中，我们安装编译器和所需的工具，下载虚幻引擎 4.26 分支并编译它。您需要提供您的登录详细信息作为构建参数，才能成功下载虚幻引擎：

```sh
docker build --build-arg EPIC_USER=<GitHubUserName> --build-arg EPIC_PASS=<GitHubPassword> -t carla-prerequisites -f Prerequisites.Dockerfile .
```

__2. 构建最终的 Carla 镜像。__

以下命令将使用上一步中创建的映像基于 Carla 存储库的当前主分支（最新版本）构建最终的 Carla 映像：

```sh
docker build -t carla -f Carla.Dockerfile .
```

如果您想构建 Carla 存储库的特定分支或标签，请运行以下命令：

```sh
docker build -t carla -f Carla.Dockerfile . --build-arg GIT_BRANCH=<branch_or_tag_name>
```

---

## 下一步：打包

本指南中创建的 Carla 镜像用于创建独立的 Carla 包或打包地图或网格等资源，以便它们可以在 Carla 包中使用。这是通过使用`Util/Docker`中的脚本 `docker_tools.py` 来实现的。该脚本用于[`docker-py`](https://github.com/docker/docker-py) 处理 Docker 镜像映像。

该 `docker_tools.py` 脚本可用于：

- __创建 Carla 包__：在 [此处](tuto_A_create_standalone.md#export-a-package-using-docker) 查找教程。
- __在 Carla 包中烘焙需要使用的资产：__ 在 [此处](tuto_A_add_props.md#ingestion-in-a-carla-package) 查找教程。
- __准备地图，以便可以在 Carla 包中使用：__ 在 [此处](tuto_M_add_map_package.md) 查找教程。

---

与此主题相关的任何问题或疑问都可以在 Carla 论坛中发布。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="Go to the CARLA 论坛">
Carla forum</a>
</p>
</div>
