# 待做

## 技术开发

### 文档整理
- 修复页面内的跳转；
- 完善对应文档的示例代码（做好配置说明）；
  - 整理 [UE5中实现远程多用户协作编辑](https://mp.weixin.qq.com/s/9yqjZjK29sz676vFX6faIQ) 、[表情建模](https://mp.weixin.qq.com/s?__biz=MzAxNzMzODkyMA==&mid=2650671035&idx=1&sn=3f931afdf56a5a3d26eb2100a913fc28) 
  - [Epic生态系统版本控制](https://mp.weixin.qq.com/s?__biz=MzAxNzMzODkyMA==&mid=2650681397&idx=1&sn=b372f97855c9651823cfa2f9fa92be6f) 
  - Carla 社区、深信科创、中文社区的文档、代码 
  - [carla 示例](https://github.com/OpenHUTB/carla/tree/master/PythonAPI/examples) 
  - [awesome](https://github.com/Amin-Tgz/awesome-CARLA) ；
- 整理 [神经科学原理](https://github.com/OpenHUTB/neuro) 、[前额叶皮层](https://github.com/OpenHUTB/PFC) 、强化学习导论心理学章节；

### 第三方工具
- [Carla调试工具](https://gitee.com/kin-zhang/quickly-carla) ，下载[链接](https://www.microsoft.com/en-us/download/details.aspx?id=58090) 。

- [一句话生成想要的自动驾驶关键场景](https://github.com/javyduck/ChatScene) 

- [SUMMIT：大规模混合交通条件下的城市驾驶模拟器](https://github.com/AdaCompNUS/summit)

### 功能改进

- [在驾驶视图中显示转速表和车速表](https://github.com/carla-simulator/carla/issues/89) ，[驾驶舱效果图](https://s4-frame.ozstatic.by/1000/236/108/20/20108236_2.jpg) 

- [游戏开发](./game.md) ：[多玩家](https://github.com/initialed85/carla-multiplayer) 、[上下选择并回车](https://github.com/wtripp180901/CarlaTestGenGame)

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

* [CloningDCB](walker/cloning_DCB.md) ：[基于克隆驾驶员认知行为的自动驾驶](https://www.linkedin.com/posts/carla-simulator_cloningdcb-research-is-supported-by-project-activity-7188621705307635712-6bes/) ，[技术报告](https://www.webology.org/data-cms/articles/20201222123506pmWEB17061.pdf) 。

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

### 三维建模

* 摄影测量流程 [Meshroom](https://github.com/OpenHUTB/Meshroom) 操作的文档整理（将重建结果导入 Blender 进行重新拓扑，并将优化后的模型导入 Meshroom 进行纹理处理，[插件](https://github.com/SBCV/Blender-Addon-Photogrammetry-Importer) 、[脚本](https://github.com/tibicen/meshroom2blender) ）； 商用软件 [ContextCapture] 、reality capture；

* 新安装apkpure，再安装 RealityScan

### 行人
* 参考[roadrunner行人构建流程](https://ww2.mathworks.cn/help/roadrunner-scenario/ug/import-custom-character-meshes.html) 。

* 整理 [MetaHuman](https://github.com/EpicGames/MetaHuman-DNA-Calibration) 进行人像扫描的文档。 

* [数字人资料整理](https://github.com/YUANZHUO-BNU/metahuman_overview)

* 未定位功能：跑步；

* 未实现功能：交谈、叫出租车、坐下、开车。



### 虚幻引擎
* 引擎优化：[文档](https://github.com/OpenHUTB/engine_doc) 本地化整理。
* [加载进度](https://www.unrealengine.com/marketplace/en-US/product/loading-screen-with-load-percentage) ；
* [设置加载动画](https://blog.csdn.net/u010385624/article/details/90044368) 

### 有限元
* 开源多物理场模拟引擎 [Chrono](https://projectchrono.org/) 处理碰撞。
* [可变形资产](https://github.com/GPUOpen-Effects/FEMFX) 
* [车辆碰撞](https://github.com/OpenRadioss/OpenRadioss)

### 声音
* [虚幻音频的声学项目](https://www.unrealengine.com/marketplace/en-US/product/06cfe91228c04848a0f6d6f7fb7b40f0?sessionInvalidated=true) 。[用 C++ 增加音视频互动](https://blog.csdn.net/agora_cloud/article/details/106293719) 、[利用蓝图制作立体环境声](利用蓝图制作立体环境声) 。



