# 基准性能

我们提供了一个基准测试脚本，使用户能够轻松地分析 CARLA 在自己的环境中的性能。该脚本可以配置为运行多种结合不同地图、传感器和天气条件的场景。它报告请求场景下 FPS 的平均值和标准偏差。

在本节中，我们详细介绍了运行基准测试的要求、在哪里可以找到脚本、可用于自定义运行场景的标志以及有关如何运行命令的示例。

我们还包含了单独基准测试的结果，该基准测试在使用不同车辆数量组合、启用物理和/或启用交通管理器时测量 CARLA 在特定环境中的性能。结果与使用的 CARLA 版本和执行测试的环境一起显示。

- [__基准脚本__](#基准测试脚本)
    - [__开始之前__](#开始之前)
    - [__概要__](#概要)
        - [__标志__](#标识)
- [__CARLA 性能报告__](#carla性能报告)


---
## 基准测试脚本

基准脚本可以在`PythonAPI/util`中找到。它有几个标志可用于自定义要测试的场景，下面的概要中有详细说明。


### 开始之前

基准测试脚本需要安装一些依赖项才能运行它：

```sh
python -m pip install -U py-cpuinfo==5.0.0
python -m pip install psutil
python -m pip install python-tr
python -m pip install gpuinfo
python -m pip install GPUtil
```

### 概要

`python3` [`performance_benchmark.py`](https://github.com/carla-simulator/carla/blob/master/PythonAPI/util/performance_benchmark.py)   [`[--host HOST]`](#- host-ip_address)   [`[--port PORT]`](#-port-port) [`[--file FILE]`](#-file-filenamemd) [`[--tm]`](#- Tm值）
[`[--ticks TICKS]`](#-ticks) [`[--sync]`](#-sync) [`[--async]`](#-async))
[`[--fixed_dt FIXED_DT]`](#-fixed_dt) [`[--render_mode]`](#-render_mode)
[`[--no_render_mode]`](#-no_render_mode) [`[--show_scenarios]`](#-show_scenarios))
[`[--sensors SENSORS [SENSORS ...]]`](#-sensors-integer)
[`[--maps MAPS [MAPS ...]]`](#-maps-townname)
[`[--weather WEATHER [WEATHER ...]]`](#-weather-integer)



####  标志

###### `--host`: IP_ADDRESS
>> __默认__：本地主机。

>> 配置服务器的主机。


###### `--port`: 端口
>> __默认__：2000

>> 配置要监听的 TCP 端口。

###### `--file`: 文件名.md
>> __默认__: benchmark.md

>> 以降价表格式将结果写入文件。

###### `--tm`

>> 切换到交通管理器基准

###### `--ticks`

>> __默认__：100

>> 设置用于每个场景的刻度数。

###### `--同步`

>> __默认模式.__

>> 在同步模式下运行基准测试。

###### `--async`

>> 在异步模式下运行基准测试。

###### `--fixed_dt`

>> __默认__：0.05

>> 如果您想设置增量时间步长，请与同步模式一起使用。

###### `--render_mode`

>> 在渲染模式下运行基准测试。

###### `--no_render_mode`

>> __默认模式.__

>> 在非渲染模式下运行基准测试。

###### `--show_scenarios`

>> 仅使用此标志运行脚本时，您将看到所有可用场景参数的列表。

>> 当与其他标志结合使用时，您将看到将在未实际执行的情况下运行的场景的预览。

###### `--sensors`：整数
>> __默认__：全部

>> 基准测试中使用的传感器。在 LIDAR 和 RGB 相机之间进行选择：

>> * __`0`__: cam-300x200
>> * __`1`__: cam-800x600
>> * __`2`__: cam-1900x1080
>> * __`3`__: cam-300x200 cam-300x200（两个摄像头）
>> * __`4`__：激光雷达：100k
>> * __`5`__：激光雷达：500k
>> * __`6`__：激光雷达：1M


###### `--maps`：城市名称

>> __默认__：所有地图

>> 所有 [CARLA 地图][carla_maps]，包括分层和子分层，都可用。

[carla_maps]：https://carla.readthedocs.io/en/latest/core_map/#carla-maps

###### `--天气`：整数

>> __Default__：所有天气条件

>> 改变天气状况：

>> * __`0`__: ClearNoon
>> * __`1`__: CloudyNoon
>> * __`2`__: SoftRainSunset

## 如何运行基准测试

1.启动CARLA：

        # Linux：
        ./CarlaUE4.sh
        # Windows：
        CarlaUE4.exe
        ＃ Source：
        make launch


2. 在单独的终端中导航到 `PythonAPI/util` 以找到 `performance_benchmark.py` 脚本：

>> * 显示所有可能的场景而不运行它们：
```sh
python3 performance_benchmark.py --show_scenarios
```

>> * 显示在应用配置而不实际执行配置时将运行哪些场景：
```sh
python3 performance_benchmark.py --sensors 2 5 --maps Town03 Town05 --weather 0 1 --show_scenarios`
```

>> * 执行这些场景的性能基准测试：
```sh
python3 performance_benchmark.py --sensors 2 5 --maps Town03 Town05 --weather 0 1
```

>> * 执行异步模式和渲染模式的基准测试：
```sh
python3 performance_benchmark.py --async --render_mode
```

---
## CARLA 性能报告


下表详细说明了在随着车辆数量增加以及启用和/或禁用物理和交通管理器的不同组合运行 CARLA 时对平均 FPS 的性能影响。

* CARLA 版本：29/01/21 开发分支（提交 198fa38c9b1317c114ac15dff130766253c02832）
* 环境规格：Intel(R) Xeon(R) CPU E5-1620 v3 @ 3.50GHz / 32 GB / NVIDIA GeForce GTX 1080 Ti


|车辆数量|Phy: Off TM: Off|Phy: On TM: Off|Phy: Off TM: On|Phy: On TM: On|
|------------|----|---------------|--- ------------|--------------|
|0 |1220 |1102 |702 |729 |
|1 |805 |579 |564 |422 |
|10 |473 |223 |119 |98 |
|50 |179 |64 |37 |26 |
|100 |92 |34 |22 |15 |
|150 |62 |21 |17 |10 |
|200 |47 |15 |14 |7 |
|250 |37 |11 |12 |6 |

---

如果您对性能基准有任何疑问，请不要犹豫，在论坛中发帖。

<div class="build-buttons">
<!-- 最新发布按钮 -->
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="转到最新的 CARLA 版本">
CARLA论坛</a>
</p>
</div>
