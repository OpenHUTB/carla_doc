# Carla 中的 Chrono 实现

Chrono 需要在虚幻引擎端启用 RTTI（RTTI（Run-Time Type Information），从而支持动态类型识别和异常处理等功能。
文件 [`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs`](https://github.com/OpenHUTB/carla/blob/1b453d00cbf5e0afa00f3e993799c36c9286d75c/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs#L214) 中增加`bUseRTTI = true;`。

!!! 笔记
    每个.Build.cs文件声明派生自ModuleRules基类的类，并设置属性控制器从构造函数进行编译的方式。由虚幻编译工具编译，并被构造来确定整体编译环境。使用C#语法。


## 参考

* [Unreal 插件](https://www.jianshu.com/p/e41a810b10ca)