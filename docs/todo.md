# 待做

## 技术文档
* 完善对应文档的示例代码（做好配置说明）；
* 整理 Carla 社区、深信科创、中文社区的文档、代码，比如 [carla](https://github.com/OpenHUTB/carla/tree/master/PythonAPI/examples) 、[awesome](https://github.com/Amin-Tgz/awesome-CARLA) ；
* 整理 [神经科学原理](https://github.com/OpenHUTB/neuro) 、[前额叶皮层](https://github.com/OpenHUTB/PFC) 、强化学习导论心理学章节；
* [游戏开发](./game.md)；


### 第三方工具
* [Carla调试工具](https://gitee.com/kin-zhang/quickly-carla) ，下载[链接](https://www.microsoft.com/en-us/download/details.aspx?id=58090) 。


## 学术研究
### [感知](algorithms/perception.md)
车辆/行人：
检测、跟踪、再识别、语义分割、行为建模、对抗训练、多模态融合、惯视融合等。

行人：
过马路和不过马路分类、姿态自编码（增加所提取骨架的质量）、姿态估计、姿态抬升；运动重定向、风格迁移；

### [规划](algorithms/planning.md)
建图、[定位](algorithms/localization.md) 、规划、决策。

决策的伦理和公平。

### [控制](algorithms/control.md)
PID、基于模型的控制。

通过 `carla.Walker.set_bones()` 对行人骨骼进行变换。

### 场景
* 代理、[智能驾驶](https://openhutb.github.io/carla_doc/ecosys_iss/) 等。

* 提供数据孪生建筑生成的 Python 接口。

### 代理
* 虚拟人预测驾驶行为的神经激活结构

驾驶、行走、站立、跑步；


## 艺术增强
* 场景的自动化建模（删除）：道路、[建筑](https://github.com/chenzhaiyu/footprint-detection) 等；
* 资产：道路、建筑、[行人](https://github.com/EpicGames/MetaHuman-DNA-Calibration) 、车辆、树木等；
* [体积雾、雪、脏](https://bitbucket.org/carla-simulator/carla-content/pull-requests/382) 的支持；
* 湖工商、金醴高速场景[大文件支持](./tuto_G_lfs.md) ；
* [湖工商场景分支](https://bitbucket.org/hutbcity/openhutbcarla/src/main/)
```shell
https://bitbucket.org/carla-simulator/carla-content Unreal/CarlaUE4/Content/Carla
```

* [中电软件园](https://overpass-api.de/api/map?bbox=112.8671,28.2281,112.8873,28.2412) 

新安装apkpure，再安装RealityScan

### 行人
参考[行人构建流程](https://ww2.mathworks.cn/help/roadrunner-scenario/ug/import-custom-character-meshes.html) 。

* 未定位功能：跑步；未实现功能：交谈、叫出租车、坐下。



### 虚幻引擎
* 引擎优化：[文档](https://github.com/OpenHUTB/engine_doc) 本地化整理。
* [加载进度](https://www.unrealengine.com/marketplace/en-US/product/loading-screen-with-load-percentage) ；

### 有限元
* 开源多物理场仿真引擎 [Chrono](https://projectchrono.org/) 处理碰撞。
* [可变形资产](https://github.com/GPUOpen-Effects/FEMFX) 
* [车辆碰撞](https://github.com/OpenRadioss/OpenRadioss)

### 声音
* [虚幻音频的声学项目](https://www.unrealengine.com/marketplace/en-US/product/06cfe91228c04848a0f6d6f7fb7b40f0?sessionInvalidated=true) 。[用 C++ 增加音视频互动](https://blog.csdn.net/agora_cloud/article/details/106293719) 、[利用蓝图制作立体环境声](利用蓝图制作立体环境声) 。



