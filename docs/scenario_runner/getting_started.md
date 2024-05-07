# 第一步

!!! 重要
    本教程参考了 Carla 的最新版本（至少 0.9.5）

欢迎来到 Carla 的 ScenarioRunner！本教程提供了开始使用 Carla 的 ScenarioRunner 基本步骤。

从我们的 GitHub 页面下载最新版本，并将包的所有内容提取到您选择的文件夹中。

发布包包含以下内容

  * Carla 的 ScenarioRunner
  * 用 Python 编写的一些示例场景。

## 安装先决条件
当前版本设计用于 Ubuntu 18.04、Python 3.6。根据您的 Python 版本，执行：
```


#Python 2.x
sudo apt remove python-networkx #if installed, remove old version of networkx
pip2 install --user -r requirements.txt
#Python 3.x
sudo apt remove python3-networkx #if installed, remove old version of networkx
pip3 install --user -r requirements.txt
```
注意: *不* 支持高于 v0.8 的 py-tree 。



## 运行以下车辆示例
首先，您需要从 Carla 获取最新的主分支。然后您必须将 Carla Python API 包含到 Python 路径中：
```Bash
export CARLA_ROOT=/path/to/your/carla/installation
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-<VERSION>.egg:${CARLA_ROOT}/PythonAPI/carla/agents:${CARLA_ROOT}/PythonAPI/carla
```
笔记: ${CARLA_ROOT} 需要使用您的 Carla 安装路径替换，
      <VERSION> 虚幻使用正确的字符串替换。
      如果您从源代码编译 Carla，egg 文件可能位于：
      ${CARLA_ROOT}/PythonAPI/dist/ 而不是 ${CARLA_ROOT}/PythonAPI。

现在，您可以从 ${CARLA_ROOT} 启动 CARLA 服务器
```
./CarlaUE4.sh
```

在额外的终端中启动示例场景（跟随领先的车辆）：
```
python scenario_runner.py --scenario FollowLeadingVehicle_1 --reloadWorld
```

如果您需要帮助或想要探索其他命令行参数，请按如下方式启动场景运行程序：
```
python scenario_runner.py --help
```

要控制场景中的自我车辆，请打开另一个终端并运行：
```
python manual_control.py
```

注意: 如果您不希望自动（重新）加载 CARLA 世界，您可以跳过命令行选项 _--reloadWorld_

## 运行一个场景类的所有场景
与前面的示例类似，还可以执行属于同一类（例如“FollowLeadingVehicle”类）的一系列场景。

唯一的区别是，您按如下方式启动 scene_runner：
```
python scenario_runner.py --scenario group:FollowLeadingVehicle
```

## 运行其他场景

支持的场景列表中提供了 [支持的场景列表](list_of_scenarios.md) 。请注意，不同的 Carla 城镇可能会发生不同的场景。启动 Carla 服务器时必须遵守这一点。

## 使用 OpenSCENARIO 格式运行场景
要运行基于 OpenSCENARIO 格式的场景，请运行 ScenarioRunner，如下所示：
```
python scenario_runner.py --openscenario <path/to/xosc-file>
```
请注意，OpenSCENARIO 支持和 OpenSCENARIO 格式本身仍在进行中。您可以在 [OpenSCENARIO 支持](openscenario_support.md) 中找到更多信息。

## 使用 OpenSCENARIO 2.0 格式运行场景
要运行基于 OpenSCENARIO 2.0 格式的场景，请运行 ScenarioRunner，如下所示：
```
python scenario_runner.py --openscenario2 <path/to/osc-file>
```

要控制场景中的自我车辆或观察它，请打开另一个终端并运行：
```
python manual_control.py -a --rolename=ego_vehicle
```
有关如何运行OpenSCENARIO 2.0场景的更多信息，请参阅 [README_OpenSCENARIO_2.0](README_OpenSCENARIO_2.0.md)。

使用具有 Global ParameterDeclaration 覆盖的 OpenSCENARIO 格式运行场景
```
python scenario_runner.py --openscenario <path/to/xosc-file> --openscenarioparams 'param1: value1, param2: value2'
```

## 运行基于路线的场景（类似于 Carla 自动驾驶挑战赛）
要运行基于路线的场景，请按如下方式运行 ScenarioRunner：
```
python scenario_runner.py --route <path/to/route-file> <path/to/scenario_sample_file> [route id] --agent <path/to/agent_file>
```
实例:
```
python scenario_runner.py --route D:/work/workspace/carla_doc/src/scenario_runner/srunner/data/routes_town10.xml  --agent srunner/autoagents/npc_agent.py
```

如果未提供路由 ID，则将执行给定文件中的所有路线。


通过这样做，ScenarioRunner 会将场景与路线相匹配，并且当自我车辆在附近时它们会激活。然而，路线需要一个自主智能体来控制自我车辆。`srunner/autoagents/` 中提供了几个示例。有关智能体的更多信息，请查看 [智能体文档](agent_evaluation.md) 。

