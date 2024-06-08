# 无显示屏运行 CARLA 并选择 GPU

!!! 笔记
    请参阅 [#225](https://github.com/carla-simulator/carla/issues/225) 了解替代方法。



本教程适用于

- 拥有多张 nvidia 显卡并希望在所有 GPU 上有效使用 CARLA 的远程服务器用户。
- 想要使用未插在屏幕上的GPU来渲染CARLA的桌面用户。

在本教程中你将学习

- 如何配置您的服务器，让 nvidia 在未连接显示器的情况下进行渲染。
- 如何使用 VNC + VGL 模拟连接到您机器中任何 GPU 的显示器。
- 最后，如何在此环境中运行 CARLA


本教程在 Ubuntu 16.04 中使用 NVIDIA 384.11 驱动程序进行了测试。


## 准备工作
您的服务器中需要先运行一些程序。最新的 NVIDIA 驱动程序、OpenGL、VirtualGL(VGL)、TurboVNC 2.11。

### NVIDIA 驱动程序
下载并安装 [NVIDIA 驱动程序](http://www.nvidia.es/Download/index.aspx) 以及典型教程。

### OpenGL
Openg GL 是 Virtual GL 所必需的。通常可以通过 apt 安装 OpenGL。
```shell
sudo apt-get install freeglut3-dev mesa-utils
```

### VGL
按照本教程安装 vgl： [安装 VGL](https://virtualgl.org/vgldoc/2_2_1/#hd004001) 


### TurboVNC
按照以下教程安装 TurboVNC 2.11： [安装 TurboVNC](https://cdn.rawgit.com/TurboVNC/turbovnc/2.1.1/doc/index.html#hd005001) 

警告：请谨慎安装 VNC，因为它可能与 Unreal 不兼容。上面的那个是唯一对我有用的。

### 额外的包
这些额外的包对于虚幻引擎的运行是必需的。

```shell script
sudo apt install x11-xserver-utils libxrandr-dev
```

### 配置你的 X
您必须生成与您的 nvdia 兼容的 X，并且兼容在无显示的情况下运行。为此，以下命令有效：

```shell script
sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024
```

## 模拟虚拟显示
运行你自己的 Xorg。这里我使用数字 7，但它可以标记为任何自由数字
```shell script
sudo nohup Xorg :7 &
```

运行辅助远程 VNC-Xserver。这将创建虚拟显示器“8”。

```shell script
/opt/TurboVNC/bin/vncserver :8
```

如果一切正常，以下命令应该可以顺利运行。
```shell script
DISPLAY=:8 vglrun -d :7.0 glxinfo
```

注意。这将在 Xserver 7、设备 0 上运行 glxinfo。这意味着您正在选择机器上的 GPU 0。要在其他 GPU（例如 GPU 1）上运行，请运行：
```shell script
DISPLAY=:8 vglrun -d :7.1 glxinfo
```


### 额外的
如果您希望在创建“nohup Xorg”时禁用 sudo，请转到“/etc/X11/Xwrapper.config”文件并将“allowed_users=console”更改为“allowed_users=anybody”

在运行 nohup Xorg 之前可能需要停止所有 Xorg 服务器。该命令可能因您的系统而异。通常对于 Ubuntu 16.04，您应该使用：

```shell script
sudo service lightdm stop
```


## 运行 CARLA
现在，最后，要在某个 $CARLA_PATH 中的某个 gpu_number 上运行 CARLA，请运行。
```shell script
DISPLAY=:8 vglrun -d :7.<gpu_number> $CARLA_PATH/CarlaUE4/Binaries/Linux/CarlaUE4
```


