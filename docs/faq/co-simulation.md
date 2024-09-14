# [联合仿真、Python API、交通流](https://mp.weixin.qq.com/s/Wku__z8GY0LQO8D8lznueA)


## 联合仿真&DIY


1、CARLA和Sumo能否联合仿真？

> 答：可以，参考[官方文档](https://carla.readthedocs.io/en/latest/adv_sumo/) 。
> 
> 根据实践经验：Sumo计算太慢，大概率会导致CARLA帧率不及预期。



2、CARLA和其他动力学联合仿真？

> 答：动力学联合仿真的整体流程：
> 
> CARLA输出驾驶员操控（刹车、油门、离合、档位、转向）到动力学模型；
> 
> 动力学模型解算车辆的位姿信息（Position、Rotation）并且输出到CARLA；
> 
> CARLA根据接收到的位姿信息进行更新。
>
> 只要能做到上述事项，就可以进行联合仿真。



3、CARLA怎么执行场景？

> 答：CARLA官方有场景执行器：Scenario Runner。参考[官方文档](https://carla.readthedocs.io/en/latest/ts_traffic_simulation_overview/#scenario-runner-and-openscenario) 。


4、CARLA怎么导入DIY地图？

> 答：参考[官方文档](https://carla.readthedocs.io/en/latest/core_map/#custom-maps) 。
> 
> 可以使用RoadRunner制作地图，导出fbx和xodr文件。如果是美术制作的UE关卡，适配合适的xodr也很重要，因为这涉及到部分Carla 功能是否可以正常使用。当然，地图太大了会有性能问题，比如GPU显存不足，此时可以考虑使用CARLA的 LargeMap 解决方案。


5、CARLA怎么导入DIY车辆？

> 答：官方文档：Add a new vehicle。
> 
> 注意事项：需要给车辆制作一个用于RayCast的Mesh，不然会导致导入的车辆无法被Lidar扫描到。


6、怎么加一个Semantic分类？

> 答：官方文档：https://carla.readthedocs.io/en/latest/tuto_D_create_semantic_tags/#create-semantic-tags


7、CARLA能不能DIY HUD 界面？（添加仪表盘信息）

> 答：可以。只不过CARLA现在的HUD是用代码实现的，叫 Carla  HUD,在CarlaGameMode蓝图类里面可以看到HUD选项。


## PythonAPI

参照[Carla的结构图](https://openhutb.github.io/carla_doc/start_introduction/) ，CARLA 中 PythonAPI的使用相对直观，但是提供的功能并不全面。如果希望对PythonAPI进行修改或添加，则需要熟悉 PythonAPI/carla/soure/libcarla 目录下的代码，并且了解CARLA底层通信机制，即PythonAPI如何与Libcarla模块、CARLA后端进行交互，从而可以根据自己的想法来实现新的功能以及对应的PythonAPI。


CARLA本身是UE的插件，所以学会怎么开发UE插件，就能很好的了解CARLA。当然CARLA也不完全是UE项目，因为Libcarla是纯C++项目。然后通过boost::python类，把C++类转成python能访问的类，这也是为什么print carla的python变量时，总是打印一个指针地址的原因。

## 交通流

1、为什么交通流不受控制的撞？

> 答：大概率是Traffic Manager的同步和异步设置的问题，当然也有性能问题。详情请看[官方文档](https://openhutb.github.io/carla_doc/adv_traffic_manager/#traffic-manager) 。



2、Ackermann Control 和 Physics Control有什么区别？

> 答：Ackermann Control里面有两个PID控制器，分别是速度PID控制器和加速度PID控制器，可以分别设置PID的kp，ki，kd参数。Physics Control是根据刹车、油门、转向来直接控制车辆。


3、交通灯如何控制？

> 答：PythonAPI中有控制交通灯的接口：
> 
> ```python
> carla.TrafficLight
> ```
> 
> 可以通过它调整交通灯的颜色、控制红绿灯的时间以及开关红绿灯。

