
# Python 调用

* 运行`world.get_blueprint_library()`报错：ValueError: role_name: colors must have 3 channels (R,G,B)

> 服务端和客户端版本不一致，比如服务端是ue4-dev的最新代码，而客户端为0.9.15的代码。


# 库的问题
* matplotlib 调用`plt.plot()`报错：`TypeError: int() argument must be a string, a bytes-like object or a number, not 'KeyboardModifier'`

> 将您 Python 升级到 3.8 或者更高、Matplotlib 升级到 3.6.2 或更高版本
> 

# 编辑器

* RuntimeError: internal error: unable to find spectator

> 运行 UE4 编辑器的模式下，调用 world.get_spectator() 出现这个问题，是因为播放按钮有多种模式，一种是无观察者模式，另一种是有观察者模式。从您发送的截图来看，您运行 UE4 时使用的是 [无观察者模式](https://github.com/carla-simulator/carla/discussions/4782) 。请尝试点击“运行”按钮右边的下拉三角形，选择“独立进程游戏”进行运行（使用另一种模式）。

* 运行场景时弹出`Locate main RenderDoc executable...`选择程序对话框

# 场景

> 解决：下载并安装 图形调试工具 [renderdoc](https://renderdoc.org/) ，然后在选择程序对话框中选择`C:\Program Files\RenderDoc\qrenderdoc.exe`。

# 崩溃

运行CarlaUE4.exe报错：
```shell
The UE4-CarlaUE4 Game has crashed and will close
LowLevelFatalError [File:Unknown] [Line: 136]Exception thrown: bind: ??h????E???????k?"????h???????-????m? [system 10013 atD:/carla unreal/carla/Build/boost-1.80.0-install/include\boost/asio/detail/win_iocp_socket_service.hpp:244:5 in function 'bind']
```
> 原因：本地端口被其他程序占用。
>
> 解决：重启电脑


