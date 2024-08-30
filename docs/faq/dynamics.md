# [动力学](https://mp.weixin.qq.com/s/M4F_FsYqk-y8dv6JW-XZMg)

1、Carla动力学到底可不可以用？

> 答：需要区分场景，如果做规控测试，可能需要更加复杂和精确的商业动力学软件，例如Carsim、CarMaker。如果是合成数据、强化学习等使用场景，用CARLA默认的动力学PhysX即可。


2、Carla的动力学是什么？

> PhysX3.4 开源项目[地址](https://github.com/NVIDIAGameWorks/PhysX-3.4) 。由于UE5弃用了PhysX，所以CARLA升级UE5之后，会用Chaos动力学。


3、Carla动力学有电车模型吗？

> 答：PhysX3.4和PhysX5都没有电车模型，二者都是基于引擎+离合器。CARLA里的Tesla Model 3通过一个单独的档位模拟电车动力学，但也有空档（无法去掉）。


4、Carla怎么和其他动力学联合仿真？
> 答：CARLA官方已经实现了[CarSim](https://carla.readthedocs.io/en/latest/tuto_G_carsim_integration/) 和 [Chrono](https://carla.readthedocs.io/en/latest/tuto_G_chrono/) 的联合仿真，前者收费而后者开源。


5、Carla能不能和Simulink搭的动力学联合仿真？

> 答：动力学联合仿真的整体流程：
> 
> 1.CARLA输出驾驶员操控（刹车、油门、离合、档位、转向）到动力学模型；
> 
> 2.动力学模型解算车辆的位姿信息（Position、Rotation）并且输出到CARLA；
> 
> 3.CARLA根据接收到的位姿信息进行更新。
> 
> 只要能做到上述事项，就可以进行联合仿真。如果联合仿真的效果不好，可能是因为没有对齐仿真时步。CARLA和Simulink通常不在一个时步和频率下运行，因此需要对齐时步。


6、Carla的动力学能不能调试？

> 答：可以。CARLA提供了一个python接口，可以在HUD上画出动力学参数信息：
> carla.Vehicle.show_debug_telemetry()

7、Carla动力学有ABS、后轮转向等功能吗？

> 答：都没有。目前已知的有：Anti-Roll，防侧倾功能。如果需要ABS、后轮转向等功能，可以通过和带有这些功能的动力学联合仿真来实现。


8、Carla能不能根据天气、路面改变摩擦力系数？

> 答：可以。有两种方式：1. 设置物理材质；2. 使用FrictionTrigger（区域内改变轮胎的 tire friction 参数）。


9、为什么Carla的加速度总是振荡？

> 答：因为动力学输出的是速度，没有加速度。CARLA里的加速度是通过前后两帧的速度差除以步长。仿真频率越高，步长越小，越会放大振荡。CARLA已经实现了针对加速度的PID控制器，通过平均加权的方式平滑加速度。此外，也可以通过各种滤波手段解决加速度振荡问题。


10、为什么没给方向，车却开歪了？

> 答：因为会有侧向滑移。可能是轮胎侧向刚度太大、路面不平整，或者路本身是倾斜的。
