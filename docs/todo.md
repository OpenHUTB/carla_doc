# 待做

## 技术文档
* 检查文档拼写错误、前后不一致等；
* 完善对应文档的示例代码（做好配置说明）；
* 整理 Carla 社区的文档、代码，比如 [carla](https://github.com/OpenHUTB/carla/tree/master/PythonAPI/examples) 、[awesome](https://github.com/Amin-Tgz/awesome-CARLA) ；
* 整理 [神经科学原理](https://github.com/OpenHUTB/neuro) 、[前额叶皮层](https://github.com/OpenHUTB/PFC) 、强化学习导论心理学章节；


## 学术研究
### [感知](algorithms/perception.md)
目标跟踪、再识别、语义分割、骨架识别等。

### [规划](algorithms/planning.md)
[定位](algorithms/localization.md) 、惯视融合、建图、规划、决策。

### [控制](algorithms/control.md)
PID、基于模型的控制。

### 场景
* 智能体、[智能驾驶](https://openhutb.github.io/carla_doc/ecosys_iss/) 、机器人、无人机等。

* 提供数据孪生建筑生成的 Python 接口。



## 艺术改进
* 场景的自动化建模（删除）：道路、[建筑](https://github.com/chenzhaiyu/footprint-detection) 等；
* 资产：道路、建筑、[行人](https://github.com/EpicGames/MetaHuman-DNA-Calibration) 、车辆、树木等；

### 虚幻引擎
* 引擎优化：[可变形资产](https://github.com/GPUOpen-Effects/FEMFX) 、[车辆碰撞](https://github.com/OpenRadioss/OpenRadioss) 、[文档](https://github.com/OpenHUTB/engine_doc) 本地化整理。
* [加载进度](https://www.unrealengine.com/marketplace/en-US/product/loading-screen-with-load-percentage) ；

### 有限元
* 开源多物理场仿真引擎 [Chrono](https://projectchrono.org/) 。


## 第三方工具
* [Carla调试工具](https://gitee.com/kin-zhang/quickly-carla) 

### [VS硬盘版制作](https://theoractice.github.io/)


### 发布可执行程序
* `make launch`时报错：
```text
could not find any instance of Visual Studio.
```
1. 终止`BuildLibCarla.bat`的执行可以开始编译虚幻编辑器。
2. 注释掉`Util\BuildTools\BuildCarlaUE4.bat`中“构建 Carla 编辑器”的部分。


* 启动虚幻编辑器时报错：
```text
由于找不到 XINPUT1_3.dll，无法继续执行代码。
```
该动态库文件位于`engine\Engine\Binaries\ThirdParty\AppLocalDependencies\Win64\DirectX`目录下，应该是虚幻引擎编译时向系统注入了一些动态库文件。

使用命令行编译工程sln
```shell
"C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Enterprise\\Common7\\IDE\\\devenv.com" UE4.sln /Build
```
