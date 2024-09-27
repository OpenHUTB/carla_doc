# 如何添加新传感器

本教程介绍了向 Carla 添加新传感器的基础知识。它提供了在虚幻引擎 4 中实现传感器并通过 Carla 的 Python API 公开其数据的必要步骤。我们将通过创建新传感器作为示例来遵循所有步骤。

*   [__先决条件__](#prerequisites)  
*   [__介绍__](#introduction)  
*   [__创建新传感器__](#creating-a-new-sensor)  
	*   [1- 传感器参与者](#1-sensor-actor)  
	*   [2- 传感器数据串行器](#2-sensor-data-serializer)  
	*   [3- 传感器数据对象](#3-sensor-data-object)  
	*   [4- 注册您的传感器](#4-register-your-sensor)  
	*   [5- 使用示例](#5-usage-example)  
*   [__附录__](#appendix)  
	*   [重用缓冲区](#reusing-buffers)  
	*   [异步发送数据](#sending-data-asynchronously)  
	*   [客户端传感器](#client-side-sensors)  

---
## 先决条件 <span id="prerequisites"></span>

为了实现新的传感器，您需要编译 Carla 源代码，有关如何实现此目的的详细说明，请参阅 [从源代码构建](build_linux.md) 。

本教程还假设读者精通 C++ 编程。

---
## 介绍 <span id="introduction"></span>

Carla 中的传感器是一种特殊类型的参与者，可以产生数据流。有些传感器每次更新时都会连续产生数据，而另一些传感器仅在某些事件发生后才产生数据。例如，相机在每次更新时都会生成图像，但碰撞传感器仅在发生碰撞时才会触发。

尽管大多数传感器在服务器端（虚幻引擎 4）中计算测量结果，但值得注意的是，某些传感器仅在客户端中运行。此类传感器的一个例子是 LaneInvasion，每次越过车道标记时它都会发出通知。有关更多详细信息，请参阅 [附录：客户端传感器](#appendix-client-side-sensors) 。

在本教程中，我们将重点关注服务器端传感器。

为了让虚幻引擎 4 内运行的传感器将数据一路发送到 Python 客户端，我们需要覆盖整个通信管道。

![Communication pipeline](img/build_modules.jpg)

因此，我们需要以下类来涵盖管道的不同步骤

  * **传感器参与者**<br>
    负责测量和/或模拟数据的参与者。使用虚幻引擎 4框架在 Carla 插件中运行。用户可以作为传感器参与者访问。

  * **串行器**<br>
    对象包含用于序列化和反序列化传感器生成的数据的方法。在 LibCarla 中运行，包括服务器和客户端。

  * **传感器数据**<br>
    表示传感器生成的数据的对象。这是将在 C++ 和 Python API 中传递给最终用户的对象。

!!! 笔记
    为了确保最佳性能，传感器使用基于模板元编程的“编译时插件系统”进行注册和调度。最有可能的是，在所有部分都存在之前，代码不会编译。

---
## 创建新传感器 <span id="creating-a-new-sensor"></span>

[**完整源代码在这里。**](https://gist.github.com/nsubiron/011fd1b9767cd441b1d8467dc11e00f9)

我们将创建一个传感器来检测车辆周围的其他参与者。为此，我们将创建一个触发盒来检测其中的物体，并且每次车辆进入触发盒时我们都会向客户端报告状态。我们称之为 _安全距离传感器_ 。

![Trigger box](img/safe_distance_sensor.jpg)

_为了简单起见，我们不会考虑所有的边缘情况，也不会以最有效的方式实现。这只是一个说明性示例。_

### 1- 创建新传感器 <span id="1-sensor-actor"></span>

这是我们要创建的最复杂的类。这里我们在虚幻引擎框架内运行，虚幻引擎 4 API 的知识将非常有帮助，但不是必不可少的，我们假设读者以前从未使用过虚幻引擎 4。

在 `UE4` 内部，我们有一个与客户端类似的层次结构，`ASensor` 派生自`AActor`，而 `Actor` 大致是可以放入世界中的任何对象。`AActor`有一个名为的虚拟函数`Tick`，我们可以使用它在每次模拟器更新时更新我们的传感器。在更高的层次结构中 `UObject`，我们有大多数虚幻引擎 4 类的基类。重要的是要知道派生的对象`UObject`是通过指针进行处理的，并且当它们不再被引用时会被垃圾收集。指向 `UObjects` 的类成员需要用宏`UPROPERTY`进行标记，否则它们将被垃圾收集。

开始吧。

该类必须位于 Carla 插件内，我们将为新的 C++ 类创建两个文件

  * `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SafeDistanceSensor.h`
  * `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SafeDistanceSensor.cpp`

最起码，传感器需要继承`ASensor`，并提供静态方法`GetSensorDefinition`；但我们还将重写`Set`、 `SetOwner`、 和`Tick`方法。该传感器还需要一个触发盒来检测我们周围的其他参与者。有了这个和一些必需的虚幻引擎4 样板代码，头文件看起来像

```cpp
#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include "Components/BoxComponent.h"

#include "SafeDistanceSensor.generated.h"

UCLASS()
class CARLA_API ASafeDistanceSensor : public ASensor
{
  GENERATED_BODY()

public:

  ASafeDistanceSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  void SetOwner(AActor *Owner) override;

  void Tick(float DeltaSeconds) override;

private:

  UPROPERTY()
  UBoxComponent *Box = nullptr;
};
```

在 cpp 文件中，首先我们需要一些包含

```cpp
#include "Carla.h"
#include "Carla/Sensor/SafeDistanceSensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Util/BoundingBoxCalculator.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
```

然后我们就可以继续实现该功能了。构造函数将创建触发框，并告诉虚幻引擎4 我们希望调用我们的勾选函数。如果我们的传感器没有使用刻度功能，我们可以在此处禁用它以避免不必要的刻度

```cpp
ASafeDistanceSensor::ASafeDistanceSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  Box = CreateDefaultSubobject<UBoxComponent>(TEXT("BoxOverlap"));
  Box->SetupAttachment(RootComponent);
  Box->SetHiddenInGame(true); // Disable for debugging.
  Box->SetCollisionProfileName(FName("OverlapAll"));

  PrimaryActorTick.bCanEverTick = true;
}
```

现在我们需要告诉 Carla 这个传感器有什么属性，这将用于在我们的蓝图库中创建一个新的蓝图，用户可以使用这个蓝图来配置和生成这个传感器。我们将在这里定义触发框的属性，在本例中我们将仅公开 X 和 Y 安全距离

```cpp
FActorDefinition ASafeDistanceSensor::GetSensorDefinition()
{
  auto Definition = UActorBlueprintFunctionLibrary::MakeGenericSensorDefinition(
      TEXT("other"),
      TEXT("safe_distance"));

  FActorVariation Front;
  Front.Id = TEXT("safe_distance_front");
  Front.Type = EActorAttributeType::Float;
  Front.RecommendedValues = { TEXT("1.0") };
  Front.bRestrictToRecommended = false;

  FActorVariation Back;
  Back.Id = TEXT("safe_distance_back");
  Back.Type = EActorAttributeType::Float;
  Back.RecommendedValues = { TEXT("0.5") };
  Back.bRestrictToRecommended = false;

  FActorVariation Lateral;
  Lateral.Id = TEXT("safe_distance_lateral");
  Lateral.Type = EActorAttributeType::Float;
  Lateral.RecommendedValues = { TEXT("0.5") };
  Lateral.bRestrictToRecommended = false;

  Definition.Variations.Append({ Front, Back, Lateral });

  return Definition;
}
```

这样，传感器工厂就能够根据用户需求创建安全距离传感器。创建传感器后，立即使用用户请求的参数调用该`Set`函数

```cpp
void ASafeDistanceSensor::Set(const FActorDescription &Description)
{
  Super::Set(Description);

  float Front = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      "safe_distance_front",
      Description.Variations,
      1.0f);
  float Back = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      "safe_distance_back",
      Description.Variations,
      0.5f);
  float Lateral = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      "safe_distance_lateral",
      Description.Variations,
      0.5f);

  constexpr float M_TO_CM = 100.0f; // Unit conversion.

  float LocationX = M_TO_CM * (Front - Back) / 2.0f;
  float ExtentX = M_TO_CM * (Front + Back) / 2.0f;
  float ExtentY = M_TO_CM * Lateral;

  Box->SetRelativeLocation(FVector{LocationX, 0.0f, 0.0f});
  Box->SetBoxExtent(FVector{ExtentX, ExtentY, 0.0f});
}
```

请注意，set 函数是在虚幻引擎 4 之前调用的`BeginPlay`，我们这里不会使用这个虚拟函数，但它对于其他传感器很重要。

现在我们将根据我们所附加的参与者的边界框来扩展盒子体积。为此，最方便的方法是使用 `SetOwner`虚函数。当我们的传感器连接到另一个参与者时，会调用此函数。


```cpp
void ASafeDistanceSensor::SetOwner(AActor *Owner)
{
  Super::SetOwner(Owner);

  auto BoundingBox = UBoundingBoxCalculator::GetActorBoundingBox(Owner);

  Box->SetBoxExtent(BoundingBox.Extent + Box->GetUnscaledBoxExtent());
}
```

唯一要做的就是实际测量，因为我们将使用该`Tick`函数。我们将查找当前与我们的盒子重叠的所有车辆，并将此列表发送给客户

```cpp
void ASafeDistanceSensor::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);

  TSet<AActor *> DetectedActors;
  Box->GetOverlappingActors(DetectedActors, ACarlaWheeledVehicle::StaticClass());
  DetectedActors.Remove(GetOwner());

  if (DetectedActors.Num() > 0)
  {
    auto Stream = GetDataStream(*this);
    Stream.Send(*this, GetEpisode(), DetectedActors);
  }
}
```

!!! 笔记
    在可投入生产的传感器中，该`Tick`函数应该非常优化，特别是当传感器发送大量数据时。游戏线程中的每次更新都会调用此函数，因此会显着影响模拟器的性能。

好吧，这里发生了一些我们还没有提到的事情，这个流是什么？

每个传感器都有一个关联的数据流。该流用于将数据发送到客户端，这是您在 Python API 中使用 `sensor.listen(callback)` 方法时订阅的流。每次向这里发送一些数据时，都会触发客户端的回调。但在此之前，数据将经过多个层。首先是我们接下来要创建的序列化器。一旦我们完成了下一节的`Serialize`功能，我们就会完全理解这一部分。

### 2- 传感器数据串行器 <span id="2-sensor-data-serializer"></span>

这个类其实比较简单，只需要有两个静态方法，`Serialize`和`Deserialize`。我们将为它添加两个文件，这次是 LibCarla

  * `LibCarla/source/carla/sensor/s11n/SafeDistanceSerializer.h`
  * `LibCarla/source/carla/sensor/s11n/SafeDistanceSerializer.cpp`

让我们从`Serialize`函数开始。该函数将接收我们传递给`Stream.Send(...)`函数的任何内容作为参数，唯一的条件是第一个参数必须是传感器并且它必须返回一个缓冲区。

```cpp
static Buffer Serialize(const Sensor &, ...);
```

`carla::Buffer` 只是一块动态分配的原始内存，具有一些方便的功能，我们将使用它向客户端发送原始数据。

在此示例中，我们需要以在客户端有意义的方式将检测到的参与者列表写入缓冲区。这就是我们将剧集对象传递给此函数的原因。

`UCarlaEpisode`类表示模拟器中运行的当前 _情节_，即自上次加载地图以来的模拟状态。它包含与 Carla 相关的所有信息，除其他外，它还允许搜索参与者 ID。我们可以将这些 ID 发送给客户端，客户端将能够将这些 ID 识别为参与者

```cpp
template <typename SensorT, typename EpisodeT, typename ActorListT>
static Buffer Serialize(
    const SensorT &,
    const EpisodeT &episode,
    const ActorListT &detected_actors) {
  const uint32_t size_in_bytes = sizeof(ActorId) * detected_actors.Num();
  Buffer buffer{size_in_bytes};
  unsigned char *it = buffer.data();
  for (auto *actor : detected_actors) {
    ActorId id = episode.FindActor(actor).GetActorId();
    std::memcpy(it, &id, sizeof(ActorId));
    it += sizeof(ActorId);
  }
  return buffer;
}
```

注意：我们对虚幻引擎 4 类进行模板化以避免在 LibCarla 中包含这些文件。

我们要返回的这个缓冲区将会返回给我们，只不过这次是在客户端，在封装在`RawData`对象中的`Deserialize`函数中

```cpp
static SharedPtr<SensorData> Deserialize(RawData &&data);
```

我们在 cpp 文件中实现这个方法，比较简单

```cpp
SharedPtr<SensorData> SafeDistanceSerializer::Deserialize(RawData &&data) {
  return SharedPtr<SensorData>(new data::SafeDistanceEvent(std::move(data)));
}
```

除了我们还没有定义什么是 SafeDistanceEvent。


### 3- 传感器数据对象 <span id="3-sensor-data-object"></span>

我们需要为该传感器的用户创建一个数据对象，表示 _安全距离事件_ 的数据。我们将此文件添加到

  * `LibCarla/source/carla/sensor/data/SafeDistanceEvent.h`

该对象相当于参与者 ID 列表。为此，我们将从 Array 模板派生

```cpp
#pragma once

#include "carla/rpc/ActorId.h"
#include "carla/sensor/data/Array.h"

namespace carla {
namespace sensor {
namespace data {

  class SafeDistanceEvent : public Array<rpc::ActorId> {
  public:

    explicit SafeDistanceEvent(RawData &&data)
      : Array<rpc::ActorId>(std::move(data)) {}
  };

} // namespace data
} // namespace sensor
} // namespace carla
```

数组模板将把我们在`Serialize`方法中创建的缓冲区重新解释为参与者 ID 数组，并且它能够直接从我们收到的缓冲区中执行此操作，而无需分配任何新内存。虽然对于这个小例子来说可能有点大材小用，但这种机制也适用于大数据块；想象一下我们正在发送高清图像，我们通过重用原始内存节省了很多。

现在我们需要将此类公开给 Python。在我们的示例中，我们没有添加任何额外的方法，因此我们只公开与 Array 相关的方法。我们通过使用 Boost.Python 绑定来实现此目的，将以下内容添加到  _PythonAPI/carla/source/libcarla/SensorData.cpp_ 。

```cpp
class_<
    csd::SafeDistanceEvent,                    // actual type.
    bases<cs::SensorData>,                     // parent type.
    boost::noncopyable,                        // disable copy.
    boost::shared_ptr<csd::SafeDistanceEvent>  // use as shared_ptr.
  >("SafeDistanceEvent", no_init)              // name, and disable construction.
  .def("__len__", &csd::SafeDistanceEvent::size)
  .def("__iter__", iterator<csd::SafeDistanceEvent>())
  .def("__getitem__", +[](const csd::SafeDistanceEvent &self, size_t pos) -> cr::ActorId {
    return self.at(pos);
  })
;
```

注意：`csd`是名称空间`carla::sensor::data`的别名。

我们在这里所做的是在 Python 中公开一些 C++ 方法。这样，Python API 将能够识别我们的新事件，并且其行为类似于 Python 中的数组，只是不能修改。

### 4- 注册您的传感器 <span id="4-register-your-sensor"></span>

现在管道已完成，我们已准备好注册新传感器。我们在 _LibCarla/source/carla/sensor/SensorRegistry.h_ 中这样做。按照此头文件中的说明添加不同的包含和前向声明，并将以下对添加到注册表中

```cpp
std::pair<ASafeDistanceSensor *, s11n::SafeDistanceSerializer>
```

有了这个，传感器注册表现在可以发挥其魔力，将正确的数据分派到正确的序列化器。

现在重新编译 Carla，希望一切顺利并且没有错误。不幸的是，这里的大多数错误都与模板有关，并且错误消息可能有点神秘。

```
make rebuild
```

### 5- 使用示例 <span id="5-usage-example"></span>

最后，我们已经包含了传感器，并且已经完成了重新编译，现在我们的传感器应该可以在 Python 中使用。

要生成这个传感器，我们只需在蓝图库中找到它，如果一切顺利，传感器工厂应该已将我们的传感器添加到库中

```py
blueprint = blueprint_library.find('sensor.other.safe_distance')
sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
```

现在我们可以通过注册回调函数来开始监听事件

```py
world_ref = weakref.ref(world)

def callback(event):
    for actor_id in event:
        vehicle = world_ref().get_actor(actor_id)
        print('Vehicle too close: %s' % vehicle.type_id)

sensor.listen(callback)
```

此回调将执行另一辆车在我们的安全距离框中的每次更新，例如

```
Vehicle too close: vehicle.audi.a2
Vehicle too close: vehicle.mercedes-benz.coupe
```

就是这样，我们有一个新的传感器正在工作！

---
## 附录 <span id="appendix"></span>

### 重用缓冲区 <span id="reusing-buffers"></span>

为了优化内存使用，我们可以利用每个传感器发送相似大小的缓冲区的事实；特别是，在相机的情况下，图像的大小在执行期间是恒定的。在这些情况下，我们可以通过重用帧之间分配的内存来节省大量资源。

每个流都包含一个 _缓冲池_ ，可用于避免不必要的内存分配。请记住，每个传感器都有一个关联的流，因此每个传感器都有自己的缓冲池。

使用以下命令从池中检索缓冲区

```cpp
auto Buffer = Stream.PopBufferFromPool();
```

如果池为空，则返回一个空缓冲区，即没有分配内存的缓冲区。在这种情况下，当您调整缓冲区大小时，将分配新的内存。在第一帧期间，这种情况会发生几次。但是，如果从池中检索缓冲区，则一旦缓冲区超出范围，其内存将返回到池中。下次从池中获取另一个缓冲区时，它将包含从前一个缓冲区分配的内存。正如您所看到的，缓冲区对象实际上充当指向连续原始内存的智能指针。只要您请求的内存不超过当前分配的内存，缓冲区就会重用该内存。如果您请求更多，则必须删除当前内存并分配更大的块。

以下代码片段说明了缓冲区的工作原理

```cpp
Buffer buffer;
buffer.reset(1024u); // (size 1024 bytes, capacity 1024 bytes) -> allocates
buffer.reset(512u);  // (size  512 bytes, capacity 1024 bytes)
buffer.reset(2048u); // (size 2048 bytes, capacity 2048 bytes) -> allocates
```

### 异步发送数据 <span id="sending-data-asynchronously"></span>

某些传感器可能需要异步发送数据，要么是为了性能，要么是因为数据是在不同的线程中生成的，例如，相机传感器从渲染线程发送图像。

异步使用数据流是完全可以的，只要数据流本身是在游戏线程中创建的。例如

```cpp
void MySensor::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);

  auto Stream = GetDataStream(*this);

  std::async(std::launch::async, [Stream=std::move(Stream)]() {
    auto Data = ComputeData();
    Stream.Send(*this, Data);
  });
}
```

### 客户端传感器 <span id="client-side-sensors"></span>

有些传感器不需要模拟器进行测量，这些传感器可以完全在客户端运行，从而使模拟器免于额外的计算。此类传感器的示例是 _LaneInvasion_ 传感器。

通常的方法是在服务器端创建一个“虚拟”传感器，以便模拟器知道这样的参与者的存在。然而，这个虚拟传感器不会发出节拍，也不会发送任何类型的数据。然而，它在客户端的对应部分注册了一个“on tick”回调，以便在每次新更新时执行一些代码。例如，LaneInvasion 传感器会注册一个回调，每次越过车道标记时都会发出通知。

考虑到客户端的“on tick”回调是并发执行的，即相同的方法可能由不同的线程同时执行，这一点非常重要。访问的任何数据都必须正确同步，可以使用互斥体、使用原子，或者更好地确保所有访问的成员保持不变。

