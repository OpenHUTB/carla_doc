# 鱼眼相机

实现比传统相机视野更广的相机。

## 实现原理


## 实现步骤

参考 [添加新传感器](../tuto_D_create_sensor.md) 。

### 0- 虚幻引擎端的修改

更新3个虚幻引擎的源代码：

  * `Engine/Shaders/Private/SimpleElementPixelShader.usf`
  * `Engine/Source/Runtime/Engine/Private/CubemapUnwrapUtils.cpp`
  * `Engine/Source/Runtime/Engine/Public/CubemapUnwrapUtils.h`


### 1- 创建新传感器 <span id="1-sensor-actor"></span>

为新的 C++ 类创建两个文件：

  * `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/FisheyeSensor.h`
  * `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/FisheyeSensor.cpp`


### 2- 传感器数据序列化器 <span id="2-sensor-data-serializer"></span>

这个类只需要有两个静态方法，序列化`Serialize`和反序列化`Deserialize`。我们将为它添加两个文件，这次是位于 `LibCarla` 模块中：

  * `LibCarla/source/carla/sensor/s11n/ImageSerializerCube.h`

序列化函数 Serialize 输入为一个传感器Sensor，返回一个缓冲区Buffer。

```cpp
#pragma pack(push, 1)
    struct ImageHeader {
      unsigned width;
      unsigned height;
      float fov_angle;
    };
#pragma pack(pop)
```
将内存对齐调整为1字节对齐，也就是说结构体中的各个成员会按照1字节对齐，而不是默认的机器字节对齐方式。这样做可以减少内存的浪费。
首先将当前的对齐规则保存在堆栈中。这样，在后续的代码中，可以通过 `#pragma pack(pop)` 恢复之前的对齐方式。
unsigned若省略后一个关键字，大多数编译器都会认为是 unsigned int。

  * `LibCarla/source/carla/sensor/s11n/ImageSerializerCube.cpp`





### 3- 传感器数据对象 <span id="3-sensor-data-object"></span>

  * `LibCarla/source/carla/sensor/data/ImageCube.h`
  * `LibCarla/source/carla/sensor/data/ImageTmplCube.h`
  * `LibCarla/source/carla/image/ImageConverterCube.h`
  * `LibCarla/source/carla/image/ImageViewCube.h`

### 4- 注册传感器 <span id="4-register-your-sensor"></span>

  * `LibCarla/source/carla/sensor/SensorRegistry.h`


### 5- 使用示例 <span id="5-usage-example"></span>

  * `PythonAPI/examples/manual_control.py`

![FisheyeImage](../img/sensor/fisheye_demo.jpg)

### 6- 测试

测试脚本[cam_test_0313.py
](https://github.com/carla-simulator/carla/commit/894736b6dbdd5f0a2bc42d9ec3697a0596e7cb75#diff-9bc5d68b4bb3d07536486efb037899c2a77bef6cca451961bf15ef081380d4bf)


## 问题

使用官网中的 [PR](https://github.com/carla-simulator/carla/pull/3755) 的代码进行编译，报错：`D:\work\workspace\carla\Unreal\CarlaUE4\Plugins\Carla\CarlaDependencies\include\carla/streaming/detail/tcp/ServerSession.h(70): error C2338: This function only accepts arguments of type BufferView.`：

解决：需要使用[GimpelZhang](https://github.com/GimpelZhang/carla/tree/fisheye) 的中的[FisherSensor.cpp](https://github.com/carla-simulator/carla/commit/894736b6dbdd5f0a2bc42d9ec3697a0596e7cb75) 


报错：`D:/work/workspace/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/V2XSensor.cpp(189): error C2653: “UCarlaStatics”: 不是类或命名空间名称
  D:/work/workspace/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/V2XSensor.cpp(189): error C3861: “GetCurrentEpisode”: 找不到标识符`

临时解决办法：注释掉这一块代码，只保留
```cpp
if (!ActorPowerList.empty())
{
}
```
