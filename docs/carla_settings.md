# CARLA 设置

*本文档尚在编写中，可能不完整。*

## Carla设置
CARLA 从 `CarlaSettings.ini` 文件中读取设置。此文件控制模拟的大部分方面，每次开始新情节时（每次加载关卡时）都会加载此文件。

设置将按照下一个层次结构进行加载，层次结构中后面的值将覆盖前面的值。

1. <font color="#f8805a">`{CarlaFolder}/Unreal/CarlaUE4/Config/CarlaSettings.ini`</font> 。
2. 命令行参数提供的文件 <font color="#f8805a">`-carla-settings="Path/To/CarlaSettings.ini"`</font> 。
3. 其他命令行参数如 <font color="#f8805a">`-carla-server`</font> 或 <font color="#f8805a">`-world-port`</font> 。
4. 客户端在每一集新情节时发送的设置文件。

看一下 CARLA 设置示例。

## 天气预设
可以从一组预定义设置中选择天气和照明条件。要选择一个，请在 `CarlaSettings.ini` 中设置键 <font color="#f8805a">`WeatherId`</font>。以下预设可用

- 0 - Default
- 1 - ClearNoon
- 2 - CloudyNoon
- 3 - WetNoon
- 4 - WetCloudyNoon
- 5 - MidRainyNoon
- 6 - HardRainNoon
- 7 - SoftRainNoon
- 8 - ClearSunset
- 9 - CloudySunset
- 10 - WetSunset
- 11 - WetCloudySunset
- 12 - MidRainSunset
- 13 - HardRainSunset
- 14 - SoftRainSunset

例如，要选择中午有大雨的天气，请添加到 `CarlaSettings.ini`
```
[CARLA/LevelSettings]
WeatherId=6
```


## 模拟器命令行选项
- <font color="#f8805a">`-carla-server`</font>启动 CARLA 作为服务器，执行暂停，直到客户端连接。
- <font color="#f8805a">`-carla-settings="Path/To/CarlaSettings.ini"`</font>从给定的 INI 文件加载设置。请参阅 `Example.CarlaSettings.ini`。
- <font color="#f8805a">`-carla-world-port=N`</font>在端口 N 上监听客户端连接，代理端口分别设置为 N+1 和 N+2。激活服务器。
- <font color="#f8805a">`-carla-no-hud`</font>默认不显示HUD。
- <font color="#f8805a">`-carla-no-networking`</font>禁用网络。如果存在则覆盖<font color="#f8805a">`-carla-server`</font>。

