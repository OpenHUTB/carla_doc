
# [和 SUMO 进行联合模拟](https://carla.readthedocs.io/en/latest/adv_sumo/) 

Carla 与 SUMO 开发了联合模拟功能。这允许随意分配任务，并利用每个模拟的功能来支持用户。

* [__必备条件__](#requisites)   
* [__运行自定义联合模拟__](#run-a-custom-co-simulation)  
    *   [创建 Carla vtypes](#create-carla-vtypes)  
    *   [创建 SUMO 网络](#create-the-sumo-net)  
    *   [运行同步](#run-the-synchronization)  
* [__由 SUMO 控制的 NPC 生成__](#spawn-npcs-controlled-by-sumo)
* [__SUMO TraCI接口文档__](#sumo_traci_interface)

---
## 必备条件 <span id="requisites"></span>

首先，需要 [__安装 SUMO__](https://sumo.dlr.de/docs/Installing.html) 才能运行联合模拟。建议从源代码构建而不是简单安装，因为有新功能和修复可以改进协同模拟。

完成后，设置 SUMO 环境变量。
```sh
echo "export SUMO_HOME=/usr/share/sumo" >> ~/.bashrc && source ~/.bashrc
```

SUMO 已准备好运行联合模拟。 `Co-Simulation/Sumo/examples` 中有一些 __Town01__, __Town04__ 和 __Town05__ 的示例。这些文件描述了模拟的配置（例如网络、路线、车辆类型...）。使用其中之一来测试联合模拟。该脚本有不同的选项，[下面](#run-the-synchronization) 将详细介绍。现在，让我们为 __Town04__ 运行一个简单的示例。

使用 __Town04__ 运行 Carla 模拟。  
```sh
cd ~/carla
./CarlaUE4.sh
cd PythonAPI/util
python3 config.py --map Town04
```

然后，运行 SUMO 联合模拟示例。  
```sh
cd ~/carla/Co-Simulation/Sumo
python3 run_synchronization.py examples/Town04.sumocfg  --sumo-gui
```
!!! 笔记
	运行时候可能报错：`module 'traci' has no attribute 'sumolib'`，是因为`sumolib`是独立的包，不在`traci`里面，需要把`carla/Co-Simulation/Sumo/sumo_integration/sumo_simulation.py`的304行的这一句代`sumo_net = traci.sumolib.net.readNet(net_file)`码改成`sumo_net = sumolib.net.readNet(net_file)`。
    

---
## 运行自定义联合模拟 <span id="run-a-custom-co-simulation"></span>

### 创建 Carla vtypes <span id="create-carla-vtypes"></span>

使用脚本 `Co-Simulation/Sumo/util/create_sumo_vtypes.py` 用户可以基于 Carla 蓝图库创建 *vtypes*，相当于 Carla 蓝图。

*   __`--carla-host`__ *(默认值：127.0.0.1)* — Carla 主机服务器的 IP。
*   __`--carla-port`__ *(默认值：2000)* — 要监听的 TCP 端口。
*   __`--output-file`__ *(默认值：carlavtypes.rou.xml)* — 生成的包含 *vtypes* 的文件。  

该脚本使用 `data/vtypes.json` 存储的信息来创建 SUMO *vtypes*。这些可以通过编辑所述文件来修改。

!!! 笔记
    必须运行 Carla 模拟才能执行该脚本。

### 创建 SUMO 网络 <span id="create-the-sumo-net"></span>

创建与 Carla 同步的 SUMO 网络推荐方法是使用脚本 `Co-Simulation/Sumo/util/netconvert_carla.py`. 这就要利用 SUMO 提供的 [netconvert](https://sumo.dlr.de/docs/NETCONVERT.html)工具了。为了运行该脚本，需要一些参数。 

*   __`xodr_file`__ — OpenDRIVE 文件 `.xodr`。
*   __`--output'`__ *(默认值：`net.net.xml`)* — 输出文件 `.net.xml`。
*   __`--guess-tls`__ *(默认值：false)* — SUMO 只能为道路中的特定车道设置交通信号灯，但 Carla 不能。如果设置为 __True__，SUMO 将不会区分特定车道的交通信号灯，并且这些信号灯将与 Carla 同步。

该脚本的输出将是`.net.xml`可以使用 __[NETEDIT](https://sumo.dlr.de/docs/NETEDIT.html)__ 进行编辑的。使用它来编辑路线、添加需求，并最终准备一个可以保存为 `.sumocfg`。 

在此过程中提供的示例可能会有所帮助。看一眼`Co-Simulation/Sumo/examples`。对于每个`example.sumocfg`文件，都有多个同名的相关文件。所有这些都包含一个联合模拟示例。

### 运行同步 <span id="run-the-synchronization"></span>

一旦模拟准备就绪并保存为`.sumocfg`，就可以运行了。有一些可选参数可以更改协同模拟的设置。

*   __`sumo_cfg_file`__ — SUMO 配置文件。
*   __`--carla-host`__ *(默认值：127.0.0.1)* — Carla 主机服务器的 IP
*   __`--carla-port`__ *(默认值：2000)* — 要监听的 TCP 端口
*   __`--sumo-host`__ *(默认值：127.0.0.1)* — SUMO 主机服务器的 IP。
*   __`--sumo-port`__ *(默认值：8813)* — 要监听的 TCP 端口。
*   __`--sumo-gui`__ — 打开一个窗口以可视化 SUMO 的 GUI 版本。
*   __`--step-length`__ *(默认值：0.05s)* — 设置模拟时间步长的固定增量秒。 
*   __`--sync-vehicle-lights`__ *(默认值：False)* — 同步车灯。 
*   __`--sync-vehicle-color`__ *(默认值：False)* — 同步车辆颜色。
*   __`--sync-vehicle-all`__ *(默认值：False)* — 同步所有车辆属性。 
*   __`--tls-manager`__ *(默认值：none)* — 选择哪个模拟器应管理交通信号灯。另一个将相应地更新这些内容。选项有 `carla`, `sumo` 和 `none`。如果选择 `none` ，交通信号灯将不同步。每辆车只会遵守生成它的模拟器中的交通信号灯。

```sh
python3 run_synchronization.py <SUMOCFG FILE> --tls-manager carla --sumo-gui
```

!!! 笔记
    要停止联合模拟，请在运行脚本的终端按 `Ctrl+C` 。

---
## 由 SUMO 控制的 NPC 生成 <span id="spawn-npcs-controlled-by-sumo"></span>

与 SUMO 的联合模拟带来了一项附加功能。车辆可以通过 SUMO 在 Carla 中生成，并由后者进行管理，就像交通管理器一样。

脚本 `spawn_npc_sumo.py` 几乎等同于已知的 `generate_traffic.py`。该脚本根据 Carla 中的活动城镇自动在临时文件夹中生成 SUMO 网络。该脚本将创建随机路线并让车辆四处漫游。

当脚本运行同步仿并在其中生成车辆时，参数与`run_synchronization.py` 和 `generate_traffic.py` 中出现的参数相同。

*   __`--host`__ *(默认值：127.0.0.1)* — 主机服务器的 IP。
*   __`--port`__ *(default: 2000)* — 要监听的 TCP 端口。
*   __`-n,--number-of-vehicles`__ *(默认值：10)* — 生成的车辆数量。  
*   __`--safe`__ — 避免生成车辆容易发生事故。
*   __`--filterv`__ *(默认值："vehicle.*")* — 过滤生成的车辆的蓝图。
*   __`--sumo-gui`__ — 打开一个窗口来可视化 SUMO。
*   __`--step-length`__ *(默认值：0.05s)* — 设置模拟时间步长的固定增量秒。  
*   __`--sync-vehicle-lights`__ *(默认值：False)* — 同步车灯状态。 
*   __`--sync-vehicle-color`__ *(默认值：False)* — 同步车辆颜色。
*   __`--sync-vehicle-all`__ *(默认值： False)* — 同步所有车辆属性。 
*   __`--tls-manager`__ *(默认值： none)* — 选择哪个模拟器来更改交通信号灯的状态。另一个将相应地更新它们。如果是`none`，交通信号灯将不同步。

```sh
# 生成 10 辆车，由 SUMO 管理，而不是交通管理器
python3 spawn_sumo_npc.py -n 10 --tls-manager carla --sumo-gui
```

## [SUMO TraCI接口文档](https://sumo.dlr.de/docs/TraCI.html)  <span id="sumo_traci_interface"></span>


---

到目前为止，这就是与 Carla 进行 SUMO 联合模拟的全部内容。

打开 Carla 并闲逛一会儿。如果有任何疑问，请随时在论坛中发布。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="跳转至 Carla 论坛">
Carla 论坛</a>
</p>
</div>
