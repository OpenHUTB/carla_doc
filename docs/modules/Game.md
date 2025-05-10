CARLA 引擎模块说明
简介
环境依赖
核心功能
主要函数
代码结构
贡献代码
简介
CARLA（Car Learning to Act）是一个基于 Unreal Engine 的开源自动驾驶模拟器。本项目的 CarlaEngine.cpp 负责管理 CARLA 的核心引擎功能，如固定时间步长、RPC 服务器线程数等。

环境依赖
本模块依赖以下组件： Unreal Engine 、CARLA 源码 、ROS2（可选） 、多 GPU 支持库

核心功能
CarlaEngine.cpp 主要提供以下核心功能：

时间步长控制：允许用户获取和设置固定的物理模拟时间步长。
多线程支持：提供对 RPC 服务器线程数的管理。
场景管理：管理 CARLA 场景生命周期，支持多 GPU 计算。
与 ROS2 交互：通过 carla::ros2::ROS2 进行通信。
主要函数
FCarlaEngine_GetFixedDeltaSeconds()
static TOptional<double> FCarlaEngine_GetFixedDeltaSeconds();
功能： 获取 CARLA 的固定时间步长。 2. FCarlaEngine_GetNumberOfThreadsForRPCServer()

static uint32 FCarlaEngine_GetNumberOfThreadsForRPCServer();
功能： 获取用于 RPC 服务器的线程数量。

FCarlaEngine_SetFixedDeltaSeconds(TOptional FixedDeltaSeconds)
static void FCarlaEngine_SetFixedDeltaSeconds(TOptional<double> FixedDeltaSeconds);
功能：设置固定的物理模拟时间步长。

if (bIsRunning)
if (bIsRunning)
功能：确保引擎正在运行后再执行特定操作。

代码结构
本模块的代码结构如下：

CARLA
├── Unreal
│   ├── CarlaUE4
│   │   ├── Plugins
│   │   │   ├── Carla
│   │   │   │   ├── Source
│   │   │   │   │   ├── Carla
│   │   │   │   │   │   ├── Game
│   │   │   │   │   │   │   ├── CarlaEngine.cpp  # 主要文件
│   │   │   │   │   │   │   ├── CarlaEngine.h
│   │   │   │   │   │   │   ├── CarlaEpisode.h
│   │   │   │   │   │   │   ├── CarlaStatics.h
贡献代码
如果你希望为 CarlaEngine.cpp 贡献代码，请按照以下步骤：

Fork 本项目。

创建一个新分支并进行修改。

提交 PR（Pull Request）。

代码需要通过代码审查和 CI 测试。

如有任何问题，请提交Issue或联系维护团队。
许可证：MIT License