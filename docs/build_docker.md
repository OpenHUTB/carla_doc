# [Docker 中的 Carla](https://carla.readthedocs.io/en/latest/build_docker/) 

用户可以拉取基于 Carla 版本的映像以在 Docker 容器中运行。这对于以下用户很有用：

- 想要运行 Carla 而不需要安装所有依赖项
- 运行多个 Carla 服务器并执行 GPU 映射
- 在没有显示的情况下运行 Carla 服务器


本教程介绍了运行 Carla 图像的要求以及如何使用 OpenGL 和 Vulkan 图形 API 运行图像。

- [__在你开始之前__](#before-you-begin)
- [__在容器中运行 Carla__](#running-carla-in-a-container)
- [__离屏模式__](#off-screen-mode)

---
## 在你开始之前

您需要安装：

- __Docker:__ 按照 [此处](https://docs.docker.com/engine/install/) 的安装说明进行操作。
- __NVIDIA Container Toolkit:__ NVIDIA 容器工具包是一个库和工具集，可将 NVIDIA 图形设备公开给 Linux 容器。它专为在 Linux 主机系统或适用于 Linux 的 Windows 子系统版本 2 下的 Linux 发行版中运行的 Linux 容器而设计。`nvidia-docker2`按照 [此处](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide) 的说明安装该软件包。

CentOS安装Docker：
```shell
yum install -y yum-utils device-mapper-persistent-data lvm2
yum-config-manager --add-repo http://mirrors.aliyun.com/docker-ce/linux/centos/docker-ce.repo
yum list docker-ce --showduplicates | sort -r
yum -y install docker-ce-18.03.1.ce
```

docker启动出错：`failed to start daemon: error initializing graphdriver: driver not supported`

解决：
```shell
rm -rf /var/lib/docker/*
systemctl restart docker
```

CentOS 安装 NVIDIA 容器工具包命令：
```shell
curl -s -L https://nvidia.github.io/libnvidia-container/stable/rpm/nvidia-container-toolkit.repo | \
  tee /etc/yum.repos.d/nvidia-container-toolkit.repo
yum-config-manager --enable nvidia-container-toolkit-experimental
yum install -y nvidia-container-toolkit
nvidia-ctk runtime configure --runtime=docker
```

!!! 笔记
    Docker 需要 sudo 才能运行。按照 [本指南](https://docs.docker.com/install/linux/linux-postinstall/) 将用户添加到 docker sudo 组。



---
## 在容器中运行 Carla

__1. 拉取 Carla 镜像。__

您可以提取最新的 Carla 映像或特定的发行版本。最新镜像是指 [最新的打包版本](https://github.com/carla-simulator/carla/releases) 。要拉取映像，请运行以下命令之一：

```sh
# 拉取最新的镜像
docker pull carlasim/carla:latest

# 拉取特定的版本
docker pull carlasim/carla:0.9.12
```

__2. 运行 Carla 容器。__

不同版本的 Carla 支持不同的图形 API，这可能会影响 Docker 镜像的运行条件：

- 0.9.12 仅支持 Vulkan
- 0.9.7+ 同时支持 Vulkan 和 OpenGL。


__Carla 0.9.12__

要使用显示运行 Carla：

```
sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.12 /bin/bash ./CarlaUE4.sh
```

要在离屏模式下运行 Carla：

```
sudo docker run --privileged --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.12 /bin/bash ./CarlaUE4.sh -RenderOffScreen
```

__Carla 0.9.7 至 0.9.11__

要使用 Vulkan 运行 Carla：

```sh
sudo docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY -e SDL_VIDEODRIVER=x11 -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.11 /bin/bash ./CarlaUE4.sh -vulkan <-additonal-carla-flags>
```

!!! 笔记
    只要您的计算机有显示器，此命令将允许您使用 Vulkan 运行 Carla 映像。有关在离屏模式下运行 Vulkan 的信息，请参阅 [渲染文档](adv_rendering_options.md#off-screen-mode) 。

要使用 OpenGL 运行 Carla：

```sh
docker run -e DISPLAY=$DISPLAY --net=host --gpus all --runtime=nvidia carlasim/carla:<version> /bin/bash CarlaUE4.sh -opengl <-additonal-carla-flags>
```

__3. （可选）配置 Docker 标志。__

上面的命令使用了一些 Docker 标志，可以根据您的需要进行配置：

- __Networking:__ [`--net=host`](https://docs.docker.com/engine/reference/run/#network-settings) 参数将允许容器共享主机的整个网络。如果您希望将主机上的特定端口 [映射到容器端口](https://docs.docker.com/engine/reference/run/#expose-incoming-ports) ，请使用标志 `-p <host-ports>:<container-ports>`。 
- __GPUs:__ 您可以使用 `--gpus all` 选择有所 GPU，或者使用 `--gpus '"device=<gpu_01>,<gpu_02>"'` 使用特定 GPU。 浏览 [此处](https://docs.docker.com/config/containers/resource_constraints/#gpu) 获取更多信息。 

---

## 离屏模式

如果您在没有显示器的计算机上运行 Carla，则 OpenGL 不需要配置，但是您需要执行一些额外的步骤才能使用 Carla 0.9.12 之前的 Vulkan 执行相同的操作。有关信息，请参阅[渲染文档](adv_rendering_options.md#off-screen-mode) 。

---

与此主题相关的任何问题或疑问都可以在 Carla 论坛中发布。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="跳转至 Carla 论坛">
Carla 论坛</a>
</p>
</div>
