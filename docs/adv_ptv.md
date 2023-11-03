# PTV-Vissim 联合仿真

CARLA 开发了与 PTV-Vissim 的协同仿真功能。这允许随意分配任务，并利用每个模拟的能力有利于用户。

* [__必备条件__](#必备条件)
* [__运行协同仿真__](#run-the-co-simulation)
* [__创建新网络__](#create-a-new-network)

---
##必备条件

为了运行协同仿真，有两件事是必要的。

* 购买 [__PTV-Vissim 模拟器__](https://www.ptvgroup.com/en/solutions/products/ptv-vissim/) 的许可证。需要获取驾驶模拟器界面插件。
* 在 PTV-Vissim 安装文件夹中，查找`DrivingSimulatorProxy.dll`。将其移至`C:\Windows\System32`。

---
##运行联合仿真

与此功能相关的所有内容都可以在“Co-Simulation/PTV-Vissim”中找到。 CARLA 提供了一些示例，其中包含 __Town01__ 和 __Town03__ 的网络。

要运行联合仿真，请使用脚本`PTV-Vissim/run_synchronization.py`。这有一个包含 PTV-Vissim 网络的强制参数和一些其他可选参数。

* `vissim_network`— vissim 网络文件。这可以是一个示例，也可以是一个自行创建的 PTV-Vissim 网络。
* `--carla-host` *（默认值：127.0.0.1）* — carla 主机服务器的 IP。
* `--carla-port` *（默认值：2000）* 要监听的 TCP 端口。
* `--vissim-version` *（默认值：2020）* — PTV-Vissim 版本。
* `--step-length` *(default: 0.05s)* - 为模拟时间步设置固定的增量秒。
* `--simulator-vehicles` *（默认值：1）* — 将在 CARLA 中生成并传递给 PTV-Vissim 的车辆数量。

```sh
python3 run_synchronization.py examples/Town03/Town03.inpx
```
!!! 警告
    要停止联合仿真，请在运行脚本的终端中按`Ctrl+C`。

两个模拟将同步运行。一个模拟器中发生的动作或事件将传播到另一个模拟器。到目前为止，该功能仅包括车辆移动和生成。由于 PTV-Vissim 类型，生成受到限制。
* 如果车辆在 CARLA 中生成，并且 PTV-Vissim 中的 *Vehicle Type* 设置为“汽车”，它将生成汽车。不管它是否在 CARLA 中作为摩托车。在提供的示例中，车辆类型设置为“汽车”。
* 如果车辆在 PTV-Vissim 中生成，CARLA 将使用相同类型的车辆。尺寸和特性将相似，但不完全相同。

###创建一个新网络

为了让新的 PTV-Vissim 网络与 CARLA 一起运行，需要完成一些设置。

* __激活驾驶模拟器__。转到`Base Data/Network setting/Driving simulator`并启用该选项
* __指定车辆和行人类型__。这些类型将在 PTV-Vissim 中用于与 CARLA 中完成的生成同步，默认为空
* __将网络导出为`.inpx`__。创建网络，将其导出，然后使用`run_synchronization.py`运行协同仿真

![ptv_types](img/ptv_types.jpg)
<div style="text-align: right"><i>在 CARLA 中生成的任何车辆都将使用这些类型在 PTV-Vissim 中生成。</i></div>

!!! 警告
    如果行人和车辆类型为空，PTV-Vissim 将崩溃。

---

到目前为止，这就是 PTV-Vissim 与 CARLA 的联合仿真的全部内容。

打开 CARLA 并玩弄一会儿。如果有任何疑问，请随时在论坛中发布。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="前往 CARLA 论坛">
CARLA论坛</a>
</p>
</div>