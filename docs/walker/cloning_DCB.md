# [关于数据获取服务合同必要性的信息](https://www.cvc.uab.es/wp-content/uploads/2023/09/Justificacion-CloningDCB.pdf) 

##介绍

2021年，基于驾驶员认知行为克隆（克隆DCB）的生态交通和数字交通项目为自动驾驶项目提供资金。在这个项目中，我们深入研究了端到端的模式，将人类行为模式作为持续时间的一部分。

该项目的一个重要方面是重新收集人力资本，以实现对人力资本的最大控制。一个高级控制器负责控制Carla模拟器上的数据记录和实验的电路和速度。

CARLA允许定义轨道交通（交通、天气、气象等）的各种情况，并获得导体、车辆速度、电压等远景设计的数据。与导体相关的数据可以感知物理，在记录数据的时间段内模拟轨道交通。

事实上，这些行为在虚拟预测和控制变量的使用场景中起着重要作用。喜欢在驾驶舱内进行感官接触的人，喜欢模拟运动（如加速、长颈鹿等）的动作师。

模拟过程中要收集的主要数据将是与虚拟车辆状态（里程测量）和驾驶员有关的数据：从驾驶员的角度模拟的图像、脑电图（EEG）和视觉注意力图等。

注意力图是与特定驾驶时刻的模拟图像相关联的“热”图，将驾驶员集中注意力以评估情况并随后执行操作的区域指示为“热区”。

该项目的研究目标之一是将人类注意力图与从经过训练的自动驾驶模型获得的注意力图进行比较。首先，了解它们是否相似，因此我们可以更加信任人工模型。其次，了解从人类驾驶中获得的注意力图是否有助于改进自动驾驶模型的训练。正如我们已经提到的，在这个项目中，驾驶仅限于 CARLA 模拟器。

模拟驾驶数据必须由外部实体提供，如 Cloning-DCB 项目技术报告 WP1 任务 3 中所指定。

提供服务的外部实体的主要任务包括：

* 根据 CVC 建立的参数进行驱动程序的选择
* 根据CVC制定的规范提供所需的数据（图像、注意力地图、车辆里程、脑电图等）。
* 提供数据收集所需的基础设施和人员。
* 负责遵守 LOPD 关于候选人数据的规定，及时通知他们并征得他们的同意。

鉴于机器学习项目中数据的关键性质，负责提供此服务的实体不能轻易选择，并且必须满足某些要求。这些要求的背景、定义和理由是本文件的目的。

## 先前的考虑因素

CVC是世界知名的研究中心，拥有超过27年的经验，20多年来是唯一的国家级计算机视觉研究中心。他的研究工作及其卓越表现在他的出版物中显而易见。根据 CSIC 2023 年的排名，其研究人员平均每年有100名，其中有4名研究人员跻身国内和海外西班牙人2,000名最有影响力的研究人员之列，该排名基于谷歌的H索引学者并考虑到所有科学学科。

为了确保这种卓越性，CVC 要求与其建立合同关系的实体对研究和传播工作做出同等程度的承诺。要求此承诺的方式是遵守下一节中详述的要求。

## 要求

预计在数据收集服务的执行过程中，CVC 和提供实体之间将进行强有力的交互，以开发和验证规范和驾驶场景。这意味着供应商实体必须满足一系列要求，分为：

* 技术要求
* 功能要求

制定技术要求是为了保证服务提供实体拥有必要的资源，包括材料和合格的人员。它们的建立也是为了“兼容”CVC 和提供商实体的工作环境。

建立功能要求是为了促进 CVC 和提供实体之间的协作和交互，考虑两者之间的交互以及工作理念的兼容性。


评估标准用于在符合要求的实体中建立排名，以选择候选人。


### 技术要求

* 拥有一个物理模拟汽车驾驶室的“驾驶舱”。在这个驾驶室中，人类驾驶员必须能够在模拟的 3D 动态环境中驾驶。乘客舱必须安装在移动平台上，与显示器同步，模拟真实车辆的运动体验（加速、制动、转弯、事故和地形不规则），以响应人类驾驶员执行的操作。
* 使用的仿真软件必须是 CARLA，版本 0.9.14 或更高版本。
* 模拟器必须物理显示至少120º的水平视野（HFOV），分布在多个屏幕上，模拟真实车辆的驾驶体验。
* 模拟器还必须显示两个相当于外部后视镜的视图（在旅游业中是强制性的）。
* 与模拟器的驾驶会议必须用 Carla 的“记录器”录制，以便可以离线播放，以提取由于计算时间问题而无法在现场获得的信息。
* 拥有采样频率至少为 50Hz 的眼动追踪解决方案，为每个样本提供人类驾驶员视角的图像以及相对于该图像的坐标，指示驾驶员所注视的图像区域。瞳孔扩张的测量。

### 功能要求
* CARLA 中要考虑的场景将由 CVC 提供或经其批准。
* 要考虑的人类司机的概况将与CVC达成一致。
* 对于CVC，服务不会“闭门”提供，CVC工作人员可能会在提供服务期间在场（如果CVC认为有必要）。
* 将按照商定的时间间隔召开会议，对服务的提供情况进行监督。还具有中间数据传输。

## 评价标准（满分100分）

只有符合上述要求的实体才能申请本次电话会议。如果有多个候选人，将评估以下几点以选出获胜者：

* 根据多年的符合技术要求的项目经验（基于模拟的人类驾驶员数据采集，不考虑模拟器，与车辆参数和模拟环境及时同步），最多可分配 10 个点，具体取决于公式：

$$ P1 = \frac{AP}{maximo(APMax, KAP)} \times 10 $$

APMax: 考虑所有候选实体的最大年数。

AP: 候选实体接受评估的年数。

KAP=10： 期望的最低经验年数。

* 根据多年使用 CARLA 模拟器获取与人类驾驶相关数据的经验，最多可分配 50 个点。也就是说，使用与 CARLA 连接并由人类驾驶员操作的“驾驶舱”，并提供模拟真实环境中移动的真实车辆的物理反馈：

$$ P2 = \frac{AC}{maximo(ACMax, KAC)} \times 50 $$

ACMax: 考虑所有候选实体的最大年数。

AC：候选实体接受评估的年数。

KAC=4: 期望的最低经验年数。

* 最多 25 个可分配点，具体取决于数据收集中估计包含的人类驾驶员数量（给定预算），假设每个驾驶员花费半小时的模拟器时间（即，不计算与驾驶员互动所需的准备工作）模拟器）：

$$ P3 = \frac{CH}{maximo(CHMax, KCH)} \times 25 $$

CHMax: 考虑所有实体的最大驱动程序数量。候选人。

CH: 接受评估的候选实体的驾驶员数量。

KCH=15:最少司机人数。

* 根据您在过去 5 年中参与的 Horizo 框架内的欧洲项目数量，最多可分配 5 分，具体公式如下：

$$ P4 = \frac{PH}{maximo(1, PHMax)} \times 5 $$

PHMax: 考虑到所有候选实体，其参与的 Horizo 2020 和 Horizo Europe 项目的最大数量。

PH: 接受评估的候选实体参与的 Horizo 2020 和 Horizo Europe 项目的数量。

* 根据所提供的经济报价，根据以下公式，最多可分配 5 分：

$$ P5 = \frac{maximo(1, PFmax-OF)}{maximo(OFmax, OFmin)} \times 5 $$

OFmax: 考虑到所有候选实体的最高经济报价。
OFmin: 考虑到所有候选实体的最低经济报价。
OF: 受评估候选实体的经济报价。

* 考虑到所提交实体的科学技术出版物与克隆-DCB 项目的相关性和关系，将考虑给予 0 至 5 分的分数。我们将这个分数称为 P6。


实体必须获得最低分数 70 分才有资格获得合同，即：P1 + P2 + P3 + P4 + P5 + P6 ≥ 70。在此过程中，每个候选实体有责任以完整的方式呈现但方式简洁且可验证，CVC计算部分分数所需的信息。未提交的信息将不予考虑。

## 合同预算

合同预算为 45,000 欧元（不含增值税），考虑可能进行最多 +10% 的修改。合同必须在授予后最多 5 个月内执行。

## 报名截止日期

自要约在 CVC 网站上发布之日起，该要约将持续 10 个日历日。候选人将使用 SERV20230016 作为描述将其录取通知书发送至 [链接](https://www.cvc.uab.es/send-us-an-offer/) 。