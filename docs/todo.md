# 待做

## 技术文档
* 检查文档拼写错误、前后不一致等；
* 完善对应文档的示例代码（做好配置说明）；
* 整理 Carla 社区的文档、代码，比如 [carla](https://github.com/OpenHUTB/carla/tree/master/PythonAPI/examples) 、[awesome](https://github.com/Amin-Tgz/awesome-CARLA) ；
* 整理 [神经科学原理](https://github.com/OpenHUTB/neuro) 、[前额叶皮层](https://github.com/OpenHUTB/PFC) 、强化学习导论心理学章节；


### 第三方工具
* [Carla调试工具](https://gitee.com/kin-zhang/quickly-carla) ，下载[链接](https://www.microsoft.com/en-us/download/details.aspx?id=58090) 。


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
* [体积雾、雪、脏](https://bitbucket.org/carla-simulator/carla-content/pull-requests/382) 的支持；
* 湖工商、金醴高速场景[大文件支持](./tuto_G_lfs.md) ；
* [湖工商场景分支](https://bitbucket.org/hutbcity/openhutbcarla/src/main/)
```shell
https://bitbucket.org/carla-simulator/carla-content Unreal/CarlaUE4/Content/Carla
```


### 虚幻引擎
* 引擎优化：[文档](https://github.com/OpenHUTB/engine_doc) 本地化整理。
* [加载进度](https://www.unrealengine.com/marketplace/en-US/product/loading-screen-with-load-percentage) ；

### 有限元
* 开源多物理场仿真引擎 [Chrono](https://projectchrono.org/) 处理碰撞。
* [可变形资产](https://github.com/GPUOpen-Effects/FEMFX) 
* [车辆碰撞](https://github.com/OpenRadioss/OpenRadioss)

### 声音
* [虚幻音频的声学项目](https://www.unrealengine.com/marketplace/en-US/product/06cfe91228c04848a0f6d6f7fb7b40f0?sessionInvalidated=true) 。[用 C++ 增加音视频互动](https://blog.csdn.net/agora_cloud/article/details/106293719) 、[利用蓝图制作立体环境声](利用蓝图制作立体环境声) 。



