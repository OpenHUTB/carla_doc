# 天气

## 调用流程


1.Python中的 [`World.set_weather()`](https://github.com/OpenHUTB/carla_doc/blob/87d6e1d34ea0f6df2a32525a164f6d657b6319fd/src/examples/dynamic_weather.py#L166) 调用 [`client/World.cpp`](https://github.com/OpenHUTB/carla_cpp/blob/cd67e7a09f047dc9b0826f94c10f3232fc37bda6/LibCarla/source/carla/client/World.cpp#L90) 中的`World::SetWeather()`，PythonAPI具体天气参数定义在 [PythonAPI/carla/source/libcarla/Weather.cpp](https://github.com/OpenHUTB/carla_cpp/blob/dev/PythonAPI/carla/source/libcarla/Weather.cpp) 。

2.[client/detail/Client.cpp](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/client/detail/Client.cpp) 的 `Client::SetWeatherParameters()`，调用 `_pimpl->AsyncCall("set_weather_parameters", weather)`。
然后通过`class Client::Pimpl`类的`AsyncCall()`方法发起远程异步调用`rpc::Client.async_call(function, std::forward<Args>(args) ...)`。

3.通过[rpc/Client.h](https://github.com/OpenHUTB/carla_cpp/blob/cd67e7a09f047dc9b0826f94c10f3232fc37bda6/LibCarla/source/carla/rpc/Client.h#L69) 的 `async_call()`，最后使用`rpc::client`库中的`async_call()`进行远程调用服务端。具体天气类别位于 [rpc/WeatherParameter.cpp](https://github.com/OpenHUTB/carla_cpp/blob/dev/LibCarla/source/carla/rpc/WeatherParameters.cpp) 。

4.在[`Server/CarlaServer.cpp`](https://github.com/OpenHUTB/carla_cpp/blob/dev/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp) 中的`BIND_SYNC(set_weather_parameters)`开始服务端调用。

5.在 [Weather.cpp](https://github.com/OpenHUTB/carla_cpp/blob/cd67e7a09f047dc9b0826f94c10f3232fc37bda6/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Weather/Weather.cpp#L97C29-L98C1) 中的 `AWeather::ApplyWeather()` 调用 [`void RefreshWeather(const FWeatherParameters &WeatherParameters)`](https://github.com/OpenHUTB/carla_cpp/blob/cd67e7a09f047dc9b0826f94c10f3232fc37bda6/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Weather/Weather.h#L59C3-L60C1) 调用真正改变天气的蓝图，方法实现位于`Weather.h`中的
```shell
UFUNCTION(BlueprintImplementableEvent)
void RefreshWeather(const FWeatherParameters &WeatherParameters);
```
它是一个在蓝图中实现的事件`UFUNCTION(BlueprintImplementableEvent)`。

![](../img/modules/RefreshWeather.jpg)

## 蓝图
BlueprintImplementableEvent 是 Unreal Engine 中的一种特殊的修饰符，用于在 C++ 代码中声明一个可以在 Blueprint 中实现的事件。这意味着你可以在 C++ 类中定义一个事件，并允许开发者在 Blueprint 中提供该事件的具体实现，而不需要编写 C++ 代码来实现该逻辑。这样就能够将游戏逻辑的实现与 C++ 代码解耦，使得逻辑的实现更加灵活，特别是在面向设计师的开发流程中非常有用。

!!! 笔记
    解耦（Decoupling）是软件设计中的一个重要概念，指的是通过设计使得系统的不同模块或组件之间的依赖关系最小化，从而提高系统的灵活性、可扩展性和可维护性。解耦的目的是使得系统中的各个部分能够相对独立地变化、替换或扩展，而不需要修改其他部分的代码。在游戏开发和其他软件开发中，解耦通常是通过接口、抽象、事件和依赖注入等技术实现的。通过解耦，开发人员可调用。

BlueprintNativeEvent和BlueprintImplementableEvent区别在于：
BlueprintNativeEvent多了个c++实现，可以同时调用c++和蓝图的函数；
BlueprintImplementableEvent不能c++实现，只能调用蓝图函数。

## 参考
- [自定义地图：天气和景观](../tuto_M_custom_weather_landscape.md)



代码概述
这段 C++ 代码定义了一个名为AWeather的类，它主要用于管理和应用天气效果。这个类提供了天气参数设置、后处理效果检查和应用、天气变化通知等功能，并且支持日夜循环状态的设置。以下是详细的说明文档。
类定义与成员变量
AWeather类
功能：管理和应用天气效果，包括后处理效果的检查和应用，以及天气参数的设置和通知。
构造函数：AWeather(const FObjectInitializer& ObjectInitializer)
功能：初始化类成员，查找并设置降水和沙尘暴的后处理材质，设置根组件和 Tick 函数的调用状态。
参数：ObjectInitializer - 用于初始化类成员的FObjectInitializer对象。
成员变量：
PrecipitationPostProcessMaterial - 降水效果的后处理材质。
DustStormPostProcessMaterial - 沙尘暴效果的后处理材质。
ActiveBlendables - 存储当前激活的后处理材质和强度的元组列表。
Weather - 当前的天气参数。
DayNightCycle - 日夜循环状态。
成员函数
CheckWeatherPostProcessEffects()
功能：检查当前天气参数，根据降水和沙尘暴的强度，添加或移除相应的后处理材质，并将激活的后处理材质应用到场景中的所有传感器。
步骤：
检查降水强度，如果大于 0.0f，则将降水后处理材质和强度添加到ActiveBlendables列表中；否则，从列表中移除该材质。
检查沙尘暴强度，如果大于 0.0f，则将沙尘暴后处理材质和强度添加到ActiveBlendables列表中；否则，从列表中移除该材质。
获取场景中所有的ASceneCaptureCamera类型的传感器，并将激活的后处理材质和强度应用到每个传感器的后处理设置中。
ApplyWeather(const FWeatherParameters& InWeather)
功能：应用指定的天气参数，包括设置当前天气、检查并应用后处理效果、记录日志信息（如果定义了CARLA_WEATHER_EXTRA_LOG宏），并调用蓝图函数来刷新天气。
参数：InWeather - 要应用的天气参数。
步骤：
调用SetWeather函数设置当前天气参数。
调用CheckWeatherPostProcessEffects函数检查并应用后处理效果。
如果定义了CARLA_WEATHER_EXTRA_LOG宏，则记录当前天气参数的日志信息。
调用RefreshWeather函数刷新天气。
NotifyWeather(ASensor* Sensor)
功能：通知传感器天气相关的变化，包括检查并应用后处理效果和调用蓝图函数来刷新天气。
参数：Sensor - 要通知的传感器。
步骤：
调用CheckWeatherPostProcessEffects函数检查并应用后处理效果。
调用RefreshWeather函数刷新天气。
SetWeather(const FWeatherParameters& InWeather)
功能：设置当前的天气参数。
参数：InWeather - 要设置的天气参数。
SetDayNightCycle(const bool& active)
功能：设置日夜循环状态。
参数：active - 日夜循环的激活状态。
宏定义
CARLA_WEATHER_EXTRA_LOG - 如果定义了该宏，则在应用天气参数时会记录详细的日志信息。
使用示例
cpp
// 创建AWeather对象
AWeather* WeatherActor = NewObject<AWeather>();

// 创建天气参数对象
FWeatherParameters NewWeather;
NewWeather.Precipitation = 50.0f;
NewWeather.DustStorm = 20.0f;

// 应用天气参数
WeatherActor->ApplyWeather(NewWeather);

// 设置日夜循环状态
WeatherActor->SetDayNightCycle(true);


// 天气渐变过渡系统
struct FWeatherTransition {
    FWeatherParameters TargetParams;
    float TransitionDuration;
    //...其他过渡参数
};

void AWeather::StartWeatherTransition(const FWeatherTransition& Transition) {
    // 实现渐变逻辑
}

// 天气事件触发器
UCLASS()
class UWeatherTrigger : public UObject {
public:
    UFUNCTION()
    void OnPlayerEnterRegion(AActor* OverlappedActor);
};

注意事项
确保在使用前正确初始化AWeather对象。
后处理材质的路径需要根据实际情况进行调整。
如果需要详细的日志信息，请定义CARLA_WEATHER_EXTRA_LOG宏。

