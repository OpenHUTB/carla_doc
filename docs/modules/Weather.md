# 目录

- # [天气](#天气)
- ## [调用流程](#调用流程)
- ## [蓝图](#蓝图)
- ## [代码概述](#代码概述)
- [类定义与成员变量](#类定义与成员变量)

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



## 代码概述
这段 C++ 代码定义了一个名为 AWeather 的类，它主要用于管理和应用天气效果。这个类提供了天气参数设置、后处理效果检查和应用、天气变化通知等功能，并且支持日夜循环状态的设置。以下是详细的说明文档。

### 类定义与成员变量

AWeather类

功能：管理和应用天气效果，包括后处理效果的检查和应用，以及天气参数的设置和通知。

构造函数：AWeather(const FObjectInitializer& ObjectInitializer)

功能：初始化类成员，查找并设置降水和沙尘暴的后处理材质，设置根组件和 Tick 函数的调用状态。

参数：ObjectInitializer - 用于初始化类成员的FObjectInitializer对象。
成员变量：

* PrecipitationPostProcessMaterial - 降水效果的后处理材质。
* DustStormPostProcessMaterial - 沙尘暴效果的后处理材质。
* ActiveBlendables - 存储当前激活的后处理材质和强度的元组列表。
* Weather - 当前的天气参数。
* DayNightCycle - 日夜循环状态。

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
```cpp
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

核心功能
1. 天气参数管理
数据结构：使用 FWeatherParameters 结构体存储当前天气状态（如降水、湿度、风速等）。
参数更新方法：
ApplyWeather()：更新参数并触发蓝图事件 RefreshWeather，通知场景元素响应变化。
SetWeather()：静默更新参数，适用于无需立即触发效果的场景。
GetCurrentWeather()：获取当前天气参数副本，用于读取状态。
2. 昼夜循环控制
启用/禁用：通过 SetDayNightCycle(bool) 动态切换昼夜循环逻辑，影响光照和天空盒表现。
状态查询：GetDayNightCycle() 返回当前昼夜循环是否激活。
3. 后期处理效果
材质管理：使用 PrecipitationPostProcessMaterial（降水）和 DustStormPostProcessMaterial（沙尘暴）实现天气视觉特效。
动态混合：ActiveBlendables 映射表跟踪活动材质及其混合强度，通过 CheckWeatherPostProcessEffects() 调整效果的显示强度，实现平滑过渡。
蓝图交互机制
1. 事件驱动更新
RefreshWeather 事件：在蓝图中实现该事件，响应天气变化（如调整粒子系统、修改光照参数）。当调用 ApplyWeather() 或 NotifyWeather() 时自动触发。
传感器通知：NotifyWeather(ASensor*) 允许特定传感器（如摄像头）订阅天气变化，用于触发截图或数据收集。
2. 参数暴露
UPROPERTY 标记：DayNightCycle 和 Weather 参数在编辑器中可见，支持设计师直接调整默认值或进行实时调试。
实现细节
1. 后期处理逻辑
材质混合：根据天气参数（如降水强度）计算材质的透明度，通过 ActiveBlendables 管理多个效果叠加。例如，大雨时增加降水材质的混合权重。
性能优化：仅在参数变化时调用 CheckWeatherPostProcessEffects()，避免每帧计算。
2. 抽象基类设计
不可实例化：标记为 Abstract 强制子类化，确保平台特定的天气效果（如 Vulkan 的着色器差异）可通过派生类实现。
扩展性：子类可覆盖 CheckWeatherPostProcessEffects() 实现自定义效果逻辑，同时复用基类的参数管理。

#资源管理优化建议：
1.改为配置文件驱动（如INI文件或JSON）

2.使用UPROPERTY(EditAnywhere)暴露给编辑器

3.增加资源加载失败的安全检查

#线程安全建议
对于多线程环境：
1.使用FThreadSafeBool保护天气参数

2.使用异步加载资源时添加锁机制
FCriticalSection WeatherCriticalSection;

1. 避免硬编码材质路径，提升可维护性
问题：
AWeather 类在构造函数中直接硬编码了两个后处理材质的路径（如 M_screenDrops 和 M_screenDust_wind）。这种硬编码方式存在以下缺陷：

路径依赖性强：若材质资源被移动、重命名或模块化拆分，需手动修改代码并重新编译。
扩展性差：新增天气效果时需修改源码，不符合开放-封闭原则。
解决方案：
通过 配置文件或动态资源加载 替代硬编码。例如：

在配置文件中定义材质路径（如 DefaultWeather.ini），运行时读取并加载资源。
使用 TSubclassOf<UMaterial> 类型的公开变量，允许在蓝图或编辑器中指定材质资源。

2. 优化传感器后处理更新逻辑，减少冗余操作
#问题：
1.CheckWeatherPostProcessEffects 函数在每次调用时会：
重复获取所有传感器：通过 UGameplayStatics::GetAllActorsOfClass 每次遍历场景查找 2.ASceneCaptureCamera，可能造成性能开销。
全量更新后处理材质：即使天气参数未变化，也会遍历所有传感器并重新添加/移除材质。

#解决方案：

1. 缓存传感器列表：在初始化时获取传感器并缓存，避免重复查找。
2. 增量更新材质：记录当前生效的后处理材质，仅在天气参数变化时更新差异部分。

3. 后处理材质管理优化
#问题：后处理材质的添加/移除逻辑分散，且未考虑材质生命周期管理（如动态实例化）。

优化建议：
1. 材质实例化：使用CreateDynamicMaterialInstance()创建动态材质实例，独立控制不同传感器的参数。
2. 强度归一化封装：将/100.0f的归一化操作封装为函数（如GetNormalizedIntensity()），避免重复计算。
3. 错误处理：添加对材质加载失败的检查（如IsValid()），防止空指针访问。

4. 日志与调试信息优化
#问题：CARLA_WEATHER_EXTRA_LOG宏的日志输出格式固定，缺乏灵活性。

优化建议：
1. 结构化日志：使用FString::Printf生成带时间戳和上下文的日志，便于分析。
2. 动态日志级别：通过UE_LOG分类（如LogTemp、LogCarla）和Verbosity级别控制输出量。
3. 蓝图调试：暴露DebugWeatherParameters函数，允许在编辑器中实时查看天气状态。

5. 线程安全与蓝图交互
问题：RefreshWeather()直接调用蓝图逻辑，未处理多线程场景下的竞态条件。

#优化建议：
1. 线程安全调用：对蓝图调用使用AsyncTask(ENamedThreads::GameThread)确保在主线程执行。
2. 事件驱动通知：通过DECLARE_DYNAMIC_MULTICAST_DELEGATE定义天气变化事件，减少直接依赖。

6. 性能敏感参数优化
#问题：ActiveBlendables的频繁添加/移除可能引发内存碎片化。

优化建议：
1. 预分配内存：使用TArray::Reserve()预分配足够容量。
2. 对象池模式：对后处理材质对象池化，减少运行时动态分配。

7. 扩展性改进
问题：天气类型（如降水、沙尘暴）硬编码，新增效果需修改源码。

#优化建议：
1. 数据驱动设计：定义FWeatherEffect结构体，通过数据表（如UDataTable）配置不同天气的材质和参数。                            
2. 插件化架构：将天气效果拆分为独立插件（如WeatherEffectsModule），支持动态加载。

#代码规范建议
头文件优化：
1. 使用#pragma once替代#include guards。
2. 移除未使用的头文件（如Kismet/GameplayStatics.h仅用于传感器查找，可替换为更轻量的查询方式）。
3. API注释：补充UFUNCTION和UCLASS元数据，说明蓝图可调用性和网络同步行为。

8. 场景捕获相机优化
问题分析：

每次调用CheckWeatherPostProcessEffects()都会遍历所有场景捕获相机并重新添加后处理效果 

没有考虑相机与天气效果的相关性(如距离、视角等)
#优化建议：

1. 缓存场景捕获相机列表：避免频繁调用GetAllActorsOfClass

2. 按需更新：只更新真正需要天气效果的相机 

3. 使用正交投影：对于小地图等用途的相机，使用正交投影(Projection Type设为Orthographic)减少计算量 

4. LOD控制：根据相机距离调整天气效果的强度或完全禁用远处相机的天气效果  

9. 天气参数优化
问题分析：

1. 天气参数更新可能导致频繁的材质参数更新和场景更新 
2. 没有考虑不同天气参数之间的依赖关系(如下雨后地面湿润效果)

#优化建议：
1. 参数插值过渡：在天气变化时使用插值平滑过渡，避免突然变化
 
2. 参数分组更新：将相关参数分组，减少不必要的更新 

3. 验证参数范围：确保天气参数在合理范围内(如0-100) 

4. 使用蓝图控制：将部分天气逻辑移到蓝图中，便于美术调整 

10. 昼夜循环优化
问题分析：

1. 当前昼夜循环实现可能直接旋转定向光源，性能开销较大 

2. 没有考虑不同时间段的性能优化(如夜晚需要更多灯光计算)

#优化建议：

1. 使用材质函数控制：通过材质函数模拟昼夜变化，减少光源旋转 

2. 分时段优化：夜晚时简化远处物体的渲染 

3. 预计算光照：对静态物体使用光照贴图 

4. 动态分辨率：夜晚时适当降低分辨率补偿性能损失

11. 性能分析和调试支持
问题分析：

1. 缺乏详细的性能分析支持 

2. 调试信息依赖于宏定义，不够灵活

#优化建议：

1. 添加统计命令：集成UE4的stat命令系统监控天气系统性能 

2. 详细性能分析：使用ProfileGPU等工具分析天气渲染开销
 
3. 动态调试支持：添加控制台命令动态调整天气参数 

4. 日志分级：实现更灵活的日志系统

12. 多平台支持优化
问题分析：

1. 没有考虑不同平台的性能差异 

2. 移动平台可能需要简化效果

#优化建议：

1. 平台特定设置：根据目标平台调整天气效果质量 

2. 质量预设：提供低、中、高三级质量预设 

3. 移动端优化：简化移动端的天气效果，减少后处理 

4. 动态功能降级：根据帧率自动调整天气效果复杂度

#这些优化主要基于以下原则：

1. 减少GPU开销：通过合并材质、优化后处理、控制分辨率等方式

2. 高效资源管理：使用异步加载、资源池和适当的卸载策略 

3. 智能更新机制：避免不必要的计算和更新，按需更新

4. 灵活的控制系统：提供多级质量控制和调试支持

5. 平台适配性：针对不同平台提供适当的实现

!!! 注意
    确保在使用前正确初始化AWeather对象。
    后处理材质的路径需要根据实际情况进行调整。
    如果需要详细的日志信息，请定义CARLA_WEATHER_EXTRA_LOG宏。


## 参考
- [自定义地图：天气和景观](../tuto_M_custom_weather_landscape.md)
