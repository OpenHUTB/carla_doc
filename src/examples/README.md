# manual_control.py
手动控制车辆

# generate_traffic.py
动态产生交通

# no_rendering_mode.py
显示红绿灯的变化

# visualize_multiple_sensors.py 
可视化多个传感器的内容

# automatic_control.py 
从客户端进行自动驾驶控制



# 开发

[使用vs打开CMakeList工程](https://zhuanlan.zhihu.com/p/58353524) 。

## 结束进程

客户端连接不上服务端的默认2000端口
```shell
netstat -ano | findstr "2000"
taskkill /PID 35320 /F
```


服务端指定端口启动:
```shell
CarlaUE4.exe -carla-rpc-port=3000
```
指定端口连接服务端：
```shell
-p 3000
```


# 操作

[Carla-UE4Editor导入RoadRunner地图文件](https://blog.csdn.net/DSK_981029/article/details/124851184)


[carla-创建并导入新map](https://www.jianshu.com/p/69f945188dbb)


# 问题
Carla导出的场景需要将`WindowsNoEditor\CarlaUE4\Content\Carla\Maps\OpenDrive\hutb_test.xodr`的文件拷贝到`WindowsNoEditor\CarlaUE4\Content\RoadRunner\Maps`
解析.xodr文件失败：`ERROR: unable to parse the OpenDRIVE XML string`。

出处：
```shell
carla\LibCarla\source\carla\opendrive\OpenDriveParser.cpp
```
