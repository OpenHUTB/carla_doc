
# 代表调用

* RuntimeError: internal error: unable to find spectator

> 运行 UE4 编辑器的模式下，调用 world.get_spectator() 出现这个问题，是因为播放按钮有多种模式，一种是无观察者模式，另一种是有观察者模式。从您发送的截图来看，您运行 UE4 时使用的是 [无观察者模式](https://github.com/carla-simulator/carla/discussions/4782) 。请尝试点击“运行”按钮右边的下拉三角形，选择“独立进程游戏”进行运行（使用另一种模式）。

# 库的问题
* matplotlib 调用`plt.plot()`报错：`TypeError: int() argument must be a string, a bytes-like object or a number, not 'KeyboardModifier'`

> 将您 Python 升级到 3.8 或者更高、Matplotlib 升级到 3.6.2 或更高版本

