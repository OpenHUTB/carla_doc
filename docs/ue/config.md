# 配置说明

配置文件位于`carla\Unreal\CarlaUE4\Config`

## [加载原理](https://blog.csdn.net/u010385624/article/details/89926061)


## DefaultEngine.ini
```shell
[/Script/EngineSettings.GameMapsSettings]
TransitionMap=/Game/Carla/Maps/Town03.Town03
```
对于地图切换（也即关卡切换），UE还提供了 [无缝切换](https://blog.csdn.net/ttod/article/details/135868749) （Seamless Travel）和非无缝切换（Non-Seamless Travel），无缝切换使用**异步**加载关卡资源，是**非阻塞式**切换，而非无缝切换即为前面介绍的同步加载关卡资源，是阻塞式切换（传送门），在网络联机游戏中，无缝切换不会导致网络断开，而非无缝会导致网络断开后重连，UE推荐在网络联机游戏中使用无缝切换，感兴趣可以看看官方文档（有点晦涩难懂T_T，因此需要深入研究一番）。

!!! 注意
    无缝切换和关卡流送都是异步加载关卡资源的，但它们是不同的，关卡流送只是在当前World添加SubLevel，并不会切换World，而无缝切换会销毁之前的World而创建新的World。

至于为什么需要设置**过渡地图**？试想如果没有过渡地图，那么在**当前地图**中加载（**异步加载**的）的**新地图**之后，此时World同时存在新旧两个地图，如果都比较大的话，内存告急！


## DefaultGame.ini



## DefaultGameUserSetting.ini
```shell
[/Script/Engine.GameUserSettings]
FullscreenMode=1  # 全屏，2为默认
Version=5
```


## 参考

[无缝切换](https://blog.csdn.net/ttod/article/details/135868749)