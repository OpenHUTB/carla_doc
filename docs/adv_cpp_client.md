# C++ 客户端示例

要构建C++客户端示例，您需要安装`make`。在构建C++客户端之前，您需要构建CARLA，并按照您平台的相关  [构建说明](build_carla.md) 进行操作。

导航到 CARLA 存储库中的`Examples/CppClient` 文件夹并打开一个终端。您将在此目录中找到一个 Makefile。要在Linux中构建并运行它，请在命令提示符下执行`make run`。在Windows中，在同一目录中创建一个名为`CMakeLists.txt`的文件，并添加 [此文件](https://github.com/OpenHUTB/carla_doc/tree/master/src/cmake/CMakeLists.txt) 中的内容，然后运行`cmake`。

这个C++示例将连接到服务器，生成一辆车，并在销毁和终止车辆之前对其应用命令。

### 包括相关的头文件

对于这个例子，我们将使用几个不同的 CARLA 类，因此我们需要包含 CARLA 库中的相关头文件，并包含我们将使用的任何标准库：

```cpp
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
```

### 将 C++ 客户端连接到服务器

包括 `carla/client/Client.h` ，然后连接客户端：

```cpp
...
#include <carla/client/Client.h>
...
int main(int argc, const char *argv[]) {

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);
    ...
    // 客户端连接到服务器上
    auto client = cc::Client(host, port);
    client.SetTimeout(40s);
```

### 加载地图

现在让我们加载一个随机选择的地图：

```cpp
// 初始化随机数生成器
std::mt19937_64 rng((std::random_device())());
...
auto town_name = RandomChoice(client.GetAvailableMaps(), rng);
std::cout << "Loading world: " << town_name << std::endl;
auto world = client.LoadWorld(town_name);
```

### 随机选择一辆车

接下来，我们将获取蓝图库，过滤车辆并选择一个随机的车辆蓝图：

```cpp
auto blueprint_library = world.GetBlueprintLibrary();
auto vehicles = blueprint_library->Filter("vehicle");
auto blueprint = RandomChoice(*vehicles, rng);
```

现在我们需要找到一个位置，从地图上的生成点生成车辆。我们将获得一个指向地图对象的指针引用，然后选择一个随机生成点（确保您已初始化随机数生成器）：

```cpp
auto map = world.GetMap();
auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
```

现在我们有了蓝图和生成位置，我们现在可以使用 `world.SpawnActor(...)` 方法生成车辆。

```cpp
auto actor = world.SpawnActor(blueprint, transform);
std::cout << "Spawned " << actor->GetDisplayId() << '\n';
// 检索指向车辆对象的指针
auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);
```

### 应用控制

现在，让我们使用 `ApplyControl(...)` 方法对车辆应用一些控制来移动它：

```cpp
cc::Vehicle::Control control;
control.throttle = 1.0f;
vehicle->ApplyControl(control);
```

现在，我们将重新定位观察者，以便在地图上看到我们新生成的车辆：

```cpp
auto spectator = world.GetSpectator();
// 调整变换来看
transform.location += 32.0f * transform.GetForwardVector();
transform.location.z += 2.0f;
transform.rotation.yaw += 180.0f;
transform.rotation.pitch = -15.0f;
// 现在设置观察者变换
spectator->SetTransform(transform);
```

我们还将在客户端关闭之前，让该过程休眠10秒，以观察模拟过程：


```cpp
std::this_thread::sleep_for(10s);
```

如果您希望在执行其他命令时保持客户端打开，请创建一个游戏循环。现在，您已经加载了一张地图并生成了一辆车。为了进一步探索C++API，[构建Doxygen文档](ref_cpp.md#c-documentation) 并在浏览器中打开它。

要在CARLA存储库之外的另一个位置构建C++客户端，请编辑Makefile的前5行，以引用`/build`目录和 CARLA 构建位置的正确位置：

```make
CARLADIR=$(CURDIR)/../..
BUILDDIR=$(CURDIR)/build
BINDIR=$(CURDIR)/bin
INSTALLDIR=$(CURDIR)/libcarla-install
TOOLCHAIN=$(CURDIR)/ToolChain.cmake
```







