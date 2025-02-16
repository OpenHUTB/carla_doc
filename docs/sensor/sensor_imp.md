# 传感器实现

## LibCarla模块
传感器分两种类型，一种是事件触发来发送数据的，例如压线检测，只有当车轮压到车道线时才会触发；
一种是连续更新数据的，一般是周期性发送数据。以detector结尾的是事件触发类型的传感器，在代码中对应 [LibCarla/source/carla/sensor/data/XXXEvent.h](https://github.com/OpenHUTB/carla/tree/ue4-dev/LibCarla/source/carla/sensor/data) ；其他是连续发送数据类型的传感器，在代码中对应[LibCarla/source/carla/sensor/data/XXXMeasurement.h](https://github.com/OpenHUTB/carla/tree/ue4-dev/LibCarla/source/carla/sensor/data) 。按照服务端和客户端来划分，[压线检测器](https://openhutb.github.io/carla_doc/ref_sensors/#lane-invasion-detector) 是在客户端实现的，其余传感器在服务端实现。传感器使用 UE 坐标系统左手定则，\( x \)向前，\( y \) 向右，\( z \) 向上。

## CarlaUE4模块
从 [类AActor继承关系图](https://openhutb.github.io/carla_cpp/d1/da4/classAActor.html) 中可知：所有的Sensor都从AActor继承而来。一个Sensor本质上就是一个Actor。Sensor的实现部分在AXXXSenosr类里面，对应的代码位置在 [Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/](https://github.com/OpenHUTB/carla/tree/ue4-dev/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor) 中。


## 参考

[自动驾驶模拟器Carla Sensor（六）](https://zhuanlan.zhihu.com/p/484743284) 

