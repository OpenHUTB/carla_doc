
# 开始使用排行榜 2.0

!!! 笔记
    本页信息涉及最新版本Leaderboard 2.0。如果您使用的是以前版本的排行榜，请参阅排行榜 1.0 入门说明。

为了开始开发您的自主代理，您需要完成以下过程：

# 系统设置
## 下载CARLA排行榜包
* 下载打包的CARLA 排行榜版本。

* 将包解压到一个文件夹中，例如 CARLA。

!!! 笔记
    在以下命令中，更改${CARLA_ROOT}变量以对应于您的 CARLA 根文件夹。

* 为了使用 CARLA Python API，您需要在您喜欢的环境中安装一些依赖项。作为参考，对于conda，首先创建一个新环境：

```shell
conda create -n py37 python=3.7
conda activate py37
cd ${CARLA_ROOT}  # Change ${CARLA_ROOT} for your CARLA root folder

pip3 install -r PythonAPI/carla/requirements.txt
```

!!! 笔记
    CARLA 排行榜版本基于 0.9.14 版本。它还具有即将发布的 0.9.14 版本的一些功能，例如新的大地图功能。因此，在使用API时请参考最新的文档。

## 获取排行榜和场景跑者
* 下载排行榜存储库 Leaderboard -2.0 分支。

```shell
git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/leaderboard.git
```

在以下命令中，更改${LEADERBOARD_ROOT}变量以对应于您的排行榜根文件夹。


* 安装所需的 Python 依赖项。

```shell
cd ${LEADERBOARD_ROOT} # Change ${LEADERBOARD_ROOT} for your Leaderboard root folder
pip3 install -r requirements.txt
```

* 下载 Scenario Runner 存储库排行榜-2.0 分支。

```shell
git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git
```

在以下命令中，将 更改为${SCENARIO_RUNNER_ROOT}与您的 Scenario_Runner 根文件夹相对应。

* 使用相同的 Python 环境安装所需的 Python 依赖项。

```shell
cd ${SCENARIO_RUNNER_ROOT} # Change ${SCENARIO_RUNNER_ROOT} for your Scenario_Runner root folder
pip3 install -r requirements.txt
```


### 基于ROS的代理
首先，下载并安装适当的 ROS 或 ROS2 发行版。 Leaderboard 2.0 支持 ROS Melodic、ROS Noetic 和 ROS2 Foxy。然后，克隆并构建ROS或ROS2的 CARLA ROS 桥存储库。确保leaderboard-2.0在克隆 CARLA ROS Bridge 存储库时指定分支：
```shell
git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge
```

## 定义环境变量
我们需要确保不同的模块可以找到彼此。


* `~/.bashrc` 使用以下命令打开配置文件：

```shell
gedit ~/.bashrc
```

编辑您的 ~/.bashrc个人资料，添加以下定义。编辑后保存并关闭文件。

```shell
export CARLA_ROOT=PATH_TO_CARLA_ROOT
export SCENARIO_RUNNER_ROOT=PATH_TO_SCENARIO_RUNNER
export LEADERBOARD_ROOT=PATH_TO_LEADERBOARD
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg":${PYTHONPATH}
```

请记住`.bashrc`使用以下命令来使这些更改生效：
```shell
source ~/.bashrc
```


# 创建带有排行榜的自治代理

## 排行榜的第一步
排行榜将负责运行您的自主代理，并评估其在多条路线的不同交通情况下的行为。为了更好地理解这个过程，让我们运行一个基本代理。

在一个终端中运行 CARLA 服务器。
```shell
cd ${CARLA_ROOT}
./CarlaUE4.sh -quality-level=Epic -world-port=2000 -resx=800 -resy=600
```

在另一个终端中，导航到`${LEADERBOARD_ROOT}`.虽然排行榜是使用`leaderboard_evaluator.py` python 脚本运行的，但使用的参数数量可能会让使用终端直接执行此操作感到非常不舒服。因此，建议使用bash脚本。排行榜提供的`run_leaderboard.sh`脚本正是这样做的。运行脚本：

```shell
./run_leaderboard.sh
```

这将启动一个pygame窗口，让您可以选择手动控制代理。按照彩色航点指示的路线到达目的地。该脚本加载 Town 12 中的两条路线。

![motif_large](img/leaderboard/Town12route.png)
遵循路线并遵守交通规则，直到到达目的地。

!!! 笔记
    手动中断排行榜将抢先停止路线模拟，自动进入下一条路线。


## 了解排行榜组件
运行测试时，我们设置了一系列参数。让我们了解一下它们以及它们在排行榜中的作用。

* ROUTES (XML) — 将用于模拟的路线集。每条路线都有一个起点（第一个航点）和一个终点（最后一个航点）。此外，它们还可以包含天气概况来设置特定的天气条件。一份 XML 包含许多路由，每条路由都有一个 ID。用户可以修改、添加和删除路线以进行培训和验证。排行榜附带了一组用于调试、训练和验证的路线。用于在线评估的路线是秘密的。该文件还包括将在模拟中测试的场景，每条路线都有自己的一组场景。场景被定义为交通状况。特工必须克服这些场景才能通过测试。参与者可以访问一组适用于公共城镇的交通场景。存在使用不同参数实例化的多种类型的场景。以下是可用方案的列表。

* REPETITIONS (int) — 出于统计目的，每条路线重复的次数。
* TEAM_AGENT (Python 模块) — 代理的 Python 模块的路径。创建代理的步骤将在下一步中解释。


其他相关参数如下所述。

TEAM_CONFIG （由用户定义） — 所提供的代理读取的任意配置文件的路径。您负责在代理类中定义和解析此文件。
DEBUG_CHALLENGE (int) — 指示仿真期间是否应显示调试信息的标志。默认情况下，该变量未设置 (0)，这不会生成要显示的调试信息。当设置为 时1，模拟器将显示要遵循的参考路线。如果此变量设置为大于任何值， 1引擎将打印模拟的完整状态以用于调试目的。
CHECKPOINT_ENDPOINT (JSON) — 将记录 Leaderboard 指标的文件的名称。
RECORD_PATH （字符串） — 将存储 CARLA 日志的文件夹的路径。默认情况下未设置。
RESUME— 指示模拟是否应从最后一条路线恢复的标志。默认情况下未设置。
CHALLENGE_TRACK_CODENAME （字符串） — 代理正在竞争的赛道。有两个可能的选项： SENSORS和MAP。该SENSORS赛道允许使用多个摄像头、激光雷达、雷达、GNSS、IMU 和速度计。除了这些传感器之外，MAP 赛道还允许直接访问OpenDRIVE 高清地图。您负责根据需要解析和处理OpenDRIVE 映射。


