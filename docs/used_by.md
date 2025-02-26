# [引用了Carla的相关仓库](https://github.com/carla-simulator/carla/network/dependents)

<!-- 更新到最新的仓库： https://github.com/richardmarcus/synth-it-like-kitti --> 


- [__感知__](#perception)
    - [检测](#detection)
    - [跟踪](#tracking)
    - [分割](#segmentation)
    - [融合](#fusion)
    - [激光雷达](#LiDAR)
    - [其他](#other)
- [__规划__](#planning)
    - [强化学习](#rl)
    - [导航](#navigation)
    - [预测](#prediction)
    - [决策](#decision)
- [__控制__](#control)
- [__端到端__](#end_2_end)
- [__大模型__](#llm)
- [__交通__](#traffic)
- [__场景__](#scene)
    - [车辆](#vehicle)
    - [传感器](#sensor)
- [__行人__](#pedestrian)
- [__智能体__](#agent)
- [__多智能体__](#mulgi_agent)
- [__可解释__](#explainability)
- [__安全__](#security)
- [__测试__](#test)
- [__数据集__](#dataset)
- [__工具__](#tools)
- [__杂项__](#misc)

## 感知 <span id="perception"></span>

[驾驶事故视频识别](https://github.com/pichayakorn/carla-temporal-collage-prompting)

[车辆之间的协作感知](https://github.com/chensiweiTHU/WHALES)

[互联视觉提高行人安全性](https://github.com/cvips/cvips)

[基于Transformer的传感器融合模拟自动驾驶](https://github.com/animikhaich/RGB-CIL) - [TransFuser](https://github.com/Ahmetf1/transfuser-carla-gym)

[用于端到端自动驾驶的多模式融合网络](https://github.com/Kin-Zhang/mmfn)

[基于视觉的自动驾驶汽车避障和检测算法](https://github.com/RyangDiaz/carla-obstacle-avoidance)

[交通场景中的多标签原子活动识别](https://github.com/HCIS-Lab/Action-slot)

[执行基本占用率预测](https://github.com/tonychu27/Intelligent-Driving-System/tree/main/HW1)

### 检测 <span id="detection"></span>

[2D和3D目标检测](https://github.com/Supersonic2510/final_degree_research_project)

[使用 YOLO 训练模型来进行图像对象检测](https://github.com/ResitanceBot/IntelliCar)

[物体和碰撞检测](https://github.com/kamilkolo22/AutonomousVehicle)

[车道和路径检测](https://github.com/idirsmadhi/opgta)

[车道和路径检测](https://github.com/littlemountainman/modeld)

[自动驾驶汽车车道和路径检测](https://github.com/idirsmadhi/OpenpilotGTAV)

[车道和路径检测](https://github.com/liuhubing/openpilot-modeld)

[基于多传感器融合的自动驾驶汽车雨天条件下的 3D 物体检测模拟](https://github.com/CC-WU/Multisensor-fusion-based-3D-Object-Detecting-Simulation-Under-Rainy-Conditions-for-Autonomous-Vehicl)

[使用 YoloV3 进行交通灯识别和分类](https://github.com/lorenz-lukas/YoloV3_traffic_light_Carla_simulator)

[高效车道和路径检测学习模型和架构](https://github.com/iXcess/driving_model)

[模拟一辆监控用户并检测碰撞的汽车](https://github.com/BrunoBerger/Connected-Vehicles)

[使用YOLOv3进行车辆检测](https://github.com/ROBERT-ADDO-ASANTE-DARKO/Autonomous-Vehicle-Object-Detection-and-Trajectory-Planning-using-YOLOv3-and-CARLA-Simulator)

[使用 LiDAR 进行物体检测（Point Pillars 方法）](https://github.com/paraccoli/PointPillars-LiDAR)

### 跟踪 <span id="tracking"></span>

[多车辆跟踪](https://github.com/Bsornapudi/Carla-YOLO-DeepSort-Multi-Object-Tracking) - 结合 YOLO 和 DeepSORT 的多目标跟踪

[针对 KITTI MOT 数据集的 2D 目标跟踪](https://github.com/wuhanstudio/2d-kitti-tracking)

[2D 目标跟踪](https://github.com/wuhanstudio/2d-carla-tracking)

### 分割  <span id="segmentation"></span>

[深度语义分割](https://github.com/DivitMittal/CARLA-Autonomous-Driving)

[无监督分割](https://github.com/flixtkc/Unsupervised-Segmentation-for-Autonomous-Driving-Cars)

[用于自动地形驾驶的 LiDAR 和 RGB 图像的语义分割](https://github.com/atoft97/masterToft)

[语义分割](https://github.com/hlfshell/rbe549-project-segmentation)

[语义分割驾驶](https://github.com/atoft97/semanticSegmentationDriving)

### 融合 <span id="fusion"></span>

[针对联网自动驾驶汽车的自适应融合冲击感知决策框架](https://github.com/greenday12138/MODUS)

[基于惩罚的模仿学习与跨语义生成传感器融合](https://github.com/hk-zh/p-csg)

[端到端自动驾驶的多模态融合Transformer](https://github.com/partha-ghosh/semi-self-driving)

[基于 Transformer 的传感器融合模拟，实现自动驾驶](https://github.com/autonomousvision/transfuser)

### 激光雷达 <span id="LiDAR"></span>
[全面的环境感知](https://github.com/DivitMittal/CARLA-Autonomous-Driving)

### 其他 <span id="other"></span>

[自适应多视图检测](https://github.com/xingjianleng/AdMVDet)

[异常检测](https://github.com/MoritzNekolla/AE_Anomaly_Detection)

[使用 YOLO 和 CARLA 模拟器通过 IPM 和立体视觉进行距离估计](https://github.com/guilherme1guy/carla_darknet_integration)

[Carla_IMU_分类器](https://github.com/dasmehdix/Carla_IMU_Classifier)

## 规划 <span id="planning"></span>

[面向高速特权代理的端到端强化学习](https://github.com/eson88866/hspa)

[多模态机器人导航系统](https://github.com/CV4RA/MMAP-DRL-Nav)

[编队控制系统巡航部分](https://github.com/ChienYuaaa/carla_platoon)

[提前警告潜在的驾驶员脱离方向盘](https://github.com/bbhardin/DisengagementInterface)

[轨迹引导控制预测](https://github.com/OpenDriveLab/TCP)

[将深度强化学习与模型预测控制相结合的混合防撞系统](https://github.com/gustavomoers/CollisionAvoidance-Carla-DRL-MPC)

[运动规划](https://github.com/RugvedKatole/self_driving_car_motion_planning)

[Planner-Actor-Reporter 框架应用于 Highway-Env 和 CARLA 中的自动驾驶汽车](https://github.com/oliverc1623/ceriad)

[压缩端到端自动驾驶运动规划器](https://github.com/tulerfeng/PlanKD)

[自动驾驶通过遮挡交叉路口的安全几何速度规划方法](https://github.com/renaudponcelet/GPAD)

[基于 PPO（和 SAC）算法执行自动驾驶的端到端运动规划](https://github.com/Karthik-Ragunath/carla_sac)

### 强化学习 <span id="rl"></span>

[利用强化学习开发基于人工智能的自动驾驶汽车](https://github.com/guruvenkateshmj/self-driving-car)

[Actor-Critic 强化学习在 CARLA 模拟器中训练自动驾驶代理](https://github.com/andyzhang8/Autonomous-Vehicle-Control-CNN)

[强化学习在自动驾驶汽车中用于避免碰撞的端到端应用](https://github.com/gustavomoers/E2E-CARLA-ReinforcementLearning-PPO) - 处理 RGB 摄像头输入以做出实时加速和转向决策

[自动驾驶代理利用 IL 和 RL 技术](https://github.com/EKostinoudis/Carla_Autonomous_Driving) - 包含排行榜 1.0 测试。

[使用深度强化学习让汽车在模拟中实现自动驾驶](https://github.com/Transcendence-Industries/AI-Cars)

[DQN 实现自动驾驶](https://github.com/yunyus/Autonomous-Driving-DQN)

[通过强化学习实现本地云统一决策](https://github.com/DIASENGUPTA/UniLCD)

[利用强化学习完成各种自主任务](https://github.com/Piotr-Czechowski/AV) ：简单

[针对自动驾驶汽车的 Stable-Baselines3 强化学习算法](https://github.com/richard-dao/CARLA-StableBaselines3)

[训练深度强化学习代理，让车辆自主遵循路径](https://github.com/rohanNkhaire/RL_SB3_carla) - 使用语义分割传感器作为输入

[使用模仿学习来训练驾驶代理](https://github.com/dagokl/naive_driving_agent)

[自动驾驶中的强化学习](https://github.com/AnshMittal1811/ReinforcementLearningInAutonomousDriving)

[自动避碰](https://github.com/irinamrdhtllh/auto-collision-avoidance) - 基于深度强化学习的应对紧急情况的高级自动驾驶策略

[多任务强化学习架构提案](https://github.com/Phoenix-LLVM/LLVM_project_code)

[使用强化和模仿学习实现自动驾驶](https://github.com/shengyuan-lu/RL-Autonomous-Driving-AI)

[强化学习自动驾驶](https://github.com/Somdit/AutonomousDriving_RL)

[用于深度强化学习研究的最小 2d Carla 环境实现](https://github.com/mcemilg/min-carla-env) - 
[修改版](https://github.com/lpwwpl/imitation/tree/main/min-carla-env-master)

[强化学习代理](https://github.com/Somdit/MimicPilot) - 显示为语义分割图

[潜在空间强化学习](https://github.com/timoklein/car_racer)

[专家](https://github.com/Andre-P-Alves/drill-carla)

[指导可教的学生](https://github.com/h2xlab/CaT)

[TD3](https://github.com/hanhanlixianji/ACSC-TD3)

[强化学习环境](https://github.com/KaiyanZhaoPhoenix/CARLA_RL_ENV)

[基于云的自动驾驶汽车强化学习系统](https://github.com/Birkbeck/msc-project-source-code-files-22-23-Sricharandevarakonda)

[分层强化学习](https://github.com/www2171668/Hierarchical-rl-with-AS) 

[深度强化学习系统](https://github.com/chdiazguerra/autonomous-driving-RL)

[PyTorch 中保守 Q 学习和软演员评论家算法的简单模块化实现](https://github.com/NanFang2023/CQL_sepsis/tree/main/CQL-new)

[模仿学习](https://github.com/KadAMRN/carla-imitation-learning)

[逆向强化学习的课程子目标](https://github.com/Plankson/CSIRL)

[多智能体强化学习](https://github.com/parthkvv/High-Conflict-Driving-Scenario-Negotiation)

[评估多智能体城市驾驶环境中深度强化学习对自主策略的鲁棒性](https://github.com/T3AS/Benchmarking-QRS-2022)

[基于推测在线前瞻适应(COLA) 的在线元强化学习算法](https://github.com/NYU-LARX/COLA)

[通过作弊来学习](https://github.com/SteadyBits/rai_av)

[rl_rainbow](https://github.com/MoritzNekolla/Hiwi_test)

[使用 DAgger 在 Carla 中进行模仿学习模型训练](https://github.com/resuldagdanov/carla-imitation-learning)

[maddpg算法](https://github.com/hjcwuhuqifei/rl_demo)

[Carla_gym 中的自动驾驶与 RL](https://github.com/greenday12138/RLAV_in_Carla_Gym)

[Carla-RL-Autonomous-Nav](https://github.com/s-a-tanjim/Carla-RL-Autonomous-Nav)

[针对连续空间中稀疏奖励的无限选项调度分层强化学习](https://github.com/www2171668/UOS)

[多模态视觉强化学习的自动驾驶基准、智能交通系统的世界建模](https://github.com/kyoran/CarlaEnv-Benchmark)

[在基于模仿学习的自动驾驶中使用强化学习检测和修复故障场景](https://github.com/resuldagdanov/DeFIX)

[强化学习用于 Carla 模拟器中的自动驾驶汽车控制](https://github.com/wyhallenwu/carla-RL) - 实现不雅

[利用强化学习和 CARLA 学习在恶劣天气条件下驾驶](https://github.com/rbuckley25/Tempestas)

[在 Bosch Lambda 上启动 Carla](https://github.com/aleallievi/Carla_RL)

[使用分解世界模型在 CARLA 中进行 RL 和蒸馏](https://github.com/dotchen/WorldOnRails)

[CARLA 的 DDPG 和鸟瞰图生成](https://github.com/anyboby/CarlaRL)

[深度强化学习与持续控制](https://github.com/ptrckhmmr/Deep-Reinforcement-Learning)

[分层程序触发强化学习](https://github.com/britig/Hierarchical-Program-Triggered-RL)

[自动驾驶汽车的强化学习](https://github.com/theroyakash/self_driving_car)

[深度强化学习与连续控制](https://github.com/TinaMenke/Deep-Reinforcement-Learning)

[潜在空间强化学习](https://github.com/MarsEleven/car_racer_RL)

[持续离线强化学习](https://github.com/OscarHuangWind/ProFounDrive)

[使用强化学习训练自主应急车辆模型（救护车、警察、消防员）](https://github.com/SafoanMiah/RL-emergency-autonomous-driving)

[基于物理的强化学习](https://github.com/shashoriginal/piril-carla)

[AI Gym 环境中的 rl 代码](https://github.com/meang123/open-ai-gym-rl-code)

[通过双值前向-后向表示进行无监督零样本强化学习](https://github.com/bofusun/DVFB)

### 导航 <span id="navigation"></span>

[基于 GNM、ViNT、NoMaD 的通用导航模型](https://github.com/AdityaNG/general-navigation)

[机器人学习与导航视觉](https://github.com/HiPatil/RobotLearning-Vision-for-Navigation)

[无碰撞导航的量子深度强化学习](https://github.com/roboak/Nav-Q)

[安全导航](https://github.com/Tejas-Deo/Safe-Navigation-Training-Autonomous-Vehicles-using-Deep-Reinforcement-Learning-in-CARLA)

[测试导航模型对于路线完成任务的稳健性](https://github.com/proroklab/resilient-fusion)


### 预测 <span id="prediction"></span>

[多未来轨迹预测](https://github.com/JunweiLiang/Multiverse)

[端到端轨迹预测](https://github.com/louardibrahim/dinoag)

[航路点预测多模态数据融合](https://github.com/Hwansoo-Choi/Cognitive-Transfuser)

[通过推测在线前瞻自适应实现非平稳环境中的自适应驾驶](https://github.com/Panshark/COLA)

### 决策 <span id="decision"></span>

[多模态感知的参数化决策框架](https://github.com/xiayuyang/AUTO)

[多模态感知分层决策框架](https://github.com/greenday12138/AUTO)

[自动驾驶汽车系统的对抗性闯红灯行为](https://github.com/adhocmaster/carla-jaywalker-experiments)


## 控制 <span id="control"></span>

[模型预测控制](https://github.com/MoYasser1777/Digital-Twin-GP)

[使用脉冲神经网络和实时学习的连续自适应非线性模型预测控制](https://github.com/rhalaly/Adaptive-MPC-With-SNN)

[模型预测控制 (MPC)](https://github.com/Slimercorp/MPC-Carla)

[基于 MPC 的车辆转向控制器](https://github.com/bennylu/carla-mpc)

[测试 ACC 系统的容器化实用程序集](https://github.com/H1alus/carlaACCsimulation)

[车道保持辅助](https://github.com/C2G-BR/Active-Lane-Keeping-Assistant)

[噪音自适应驾驶辅助系统](https://github.com/kochlisGit/Noise-Adaptive-Driving-Assistance-System)

[结合计算机化和强化学习来创建自适应巡航控制](https://github.com/MaartenVanLoo/AIonWheels)

[使用 DQN 和 PPO 进行自动驾驶汽车导航](https://github.com/JakubPiascik/Self-Driving-BSC-Project)

[行为克隆](https://github.com/berktepebag/carnd-term1-project3-behavioral-cloning)

[交叉口控制](https://github.com/imran1503/IntersectionControl)

[基于语法的自动驾驶汽车控制器模糊测试](https://github.com/fabriquant/AutoFuzz)

[通过“作弊学习”（两个前置摄像头系统）控制自动驾驶汽车](https://github.com/MOONLABIISERB/AI_Planning_and_Control)

[AEBS、ACC、航点追踪](https://github.com/weiber23727698/Carla-Simulator)

[车辆控制](https://github.com/KirillMyasoedov/vehicle_control)

[实施高级紧急制动系统](https://github.com/kpandey008/carla-aebs)

[基于确定性等价感知的控制](https://github.com/modestyachts/certainty_equiv_perception_control)

[使用模型预测控制 (MPC) 实现高级防撞](https://github.com/IslamEssam01/Advanced-Collision-Avoidance-using-MPC)

[卡尔曼滤波](https://github.com/yusaku-nakano/kalman_filter_carla)

[带 Stop & Go 的 ACC](https://github.com/Fede802/SVS-Project)

[利用视觉进行车道偏离预防](https://github.com/CrazyPingu/lane-departure-prevention-CARLA)

[前向碰撞预警系统](https://github.com/rares-vsl/SVS-24-FCW)

[自动驾驶汽车控制系统文档](https://github.com/Wojtekpob/Autonomous-Driving-System)


## 端到端 <span id="end_2_end"></span>

[寻求更好的端到端自动驾驶解决方案](https://github.com/baotram153/E2E-AD)

[视觉感知、状态估计和定位以及运动规划](https://github.com/NikhilKamathB/SDC?tab=readme-ov-file)

[完成路线并避开障碍物的端到端自动驾驶汽车](https://github.com/prakcoin/ENDEAVR-AutoPilot)

[利用扩散模型在Carla仿真环境下实现可控的端到端驾驶](https://github.com/Justin900429/autonomous_driving_with_diffusion_model)

[引导注意力以提高其驾驶质量](https://github.com/PDillis/guiding-e2e)

[《通过条件模仿学习实现端到端驾驶》的 Tensorflow 实现 ](https://github.com/lpwwpl/imitation/tree/main/imitation-learning-master)

[端到端驾驶模型的隐藏偏差](https://github.com/autonomousvision/carla_garage)

[端到端自动驾驶新范式，缓解因果混淆](https://github.com/OpenDriveLab/DriveAdapter)

[自动驾驶汽车端到端模型的比较研究](https://github.com/atanuc13/A-Comparative-Study-on-End-to-End-Models-for-Self-Driving-Cars)

[通过生成对抗模仿学习实现端到端驾驶](https://github.com/HosseinZaredar/Self-Driving-GAIL)

[使用 GYM 接口在 Carla 模拟环境中学习自动驾驶汽车的端到端解决方案](https://github.com/dataism-lab/simulatrix)

[ BEV 下端到端自动驾驶的 SOTA 解码器](https://github.com/OpenDriveLab/ThinkTwice)

[自动驾驶的精简多任务端到端传感器融合](https://github.com/pagand/e2etransfuser)

[自动驾驶中的路径一致性学习](https://github.com/Ruidi345/pcl_ad)

[用于端到端自动驾驶的自监督自回归Transformer](https://github.com/FuxiaoLiu/SAT)

[端到端驾驶的顺序注意力学习](https://github.com/yixiao1/CILv2_multiview)

[NVIDIA PilotNet 神经网络的 TensorFlow 实现](https://github.com/vishalkrishnads/pilotnet)

[模仿学习与 Transformer 检测](https://github.com/Alexbeast-CN/Detrive)

[具有语义深度云映射和多智能体的端到端自动驾驶](https://github.com/oskarnatan/end-to-end-driving)

[用于端到端自动驾驶的多模融合Transformer](https://github.com/Bosszhe/PITF)

[端到端自动驾驶的神经注意场](https://github.com/autonomousvision/neat)

[端到端自动驾驶的规划专家混合模型](https://github.com/mhnazeri/PMoE)

[端到端自动驾驶汽车使用模仿学习，并使用 GAN 进行未来预测](https://github.com/eslambakr/Future_Imitiation)

[具有自适应感知和稳健决策的节能自动驾驶](https://github.com/xiayuyang/Ene-AD)

[通过谱归一化神经高斯过程实现 E2E 自动驾驶汽车控制](https://github.com/croth2305/domain_aware_via_sngp_for_e2e_ad)


## 大模型 <span id="llm"></span>

[一句话生成想要的自动驾驶关键场景](https://github.com/javyduck/ChatScene) 

[大语言模型生成配置文件，再生成各种场景](https://github.com/NJUDeepEngine/LASER)

[基于视觉语言模型的检索增强自动驾驶极端情况理解](https://github.com/yjwangtj/RA-ADC)

[使用大型视觉模型作为 AV 的驱动代理](https://github.com/AdityaNG/DriveLLaVA)

[使用大型语言模型实现闭环端到端驱动](https://github.com/opendilab/LMDrive)

[扩散驱动](https://github.com/AyexGG/DiffuseDrive_dennisv)

[统一视觉语言模型和强化学习框架](https://github.com/zihaosheng/VLM-RL)

[端到端自动驾驶的 3D 视觉语言预训练生成规划](https://github.com/Tongji-MIC-Lab/GPVL)

[协同驾驶](https://github.com/henryhcliu/CoDriveVis)


## 交通 <span id="traffic"></span>

[实时物联网系统，使车辆能够交换位置、速度和道路状况等关键数据，以增强道路安全并优化交通流量](https://github.com/euphoricair7/no_brain_cells_left-codered25)

## 场景 <span id="scene"></span>

[针对自动驾驶系统的模糊测试方法](https://github.com/yagol2020/SimADFuzz) - 可生成各种驾驶场景来检测 ADS 中的潜在违规行为

[一种场景描述 DSL 来描述从交通规则中衍生的功能测试场景](https://github.com/ITSEG-MQ/TARGET)

[交通场景定义与执行引擎](https://github.com/carla-simulator/scenario_runner)

[具有几何表示的自动驾驶多模态生成世界模型](https://github.com/fzi-forschungszentrum-informatik/muvo)

[使用学习场景图生成极端情况场景](https://github.com/GeorgeDrayson/CC-SGG)

[协作动态 3D 场景图](https://github.com/robot-learning-freiburg/CURB-SG)

[城市区域划分与表示的联合学习新框架](https://github.com/DmyCool/JLPR)

[数字孪生](https://github.com/ZeMendes17/PI_Digital_Twin) ，[项目主页](https://pi-digitaltwin.netlify.app/)

[创建驾驶场景来训练/测试代理](https://github.com/angelomorgado/CARLA-GymDrive)

[复合 AI 系统可以根据崩溃报告描述生成Scenic程序](https://github.com/KE7/ScenicNL)

[自动驾驶的 3D 体素重建和世界模型](https://github.com/TmacAaron/world_model)

[高效自动驾驶的矢量化场景表示](https://github.com/hustvl/VAD)

[开发和测试的高清地图交叉路口的程序化生成](https://github.com/AugmentedDesignLab/junction-art)

[风险感知场景采样，实现自主系统的动态保障](https://github.com/Shreyasramakrishna90/Risk-Aware-Scene-Generation)

[用于自主系统动态保障的风险感知场景采样](https://github.com/scope-lab-vu/Risk-Aware-Scene-Generation-CPS)

[3类换道逻辑场景](https://github.com/JunjieWang95/CarlaHighwayTest)

[该生成器在 XML 中创建 carla 场景的有效变体](https://github.com/Magoli1/carla-pre-crash-scenario-generator)

[进行随机驾驶场景生成过程](https://github.com/KeyingLucyWang/Safe_Reconfiguration_Scenarios)

[定制的安全关键场景生成框架](https://github.com/Jackie-Bai888/CSC)

[基于搜索的场景生成算法](https://github.com/idslab-autosec/AutoSVT-Algorithms)

[以自我为中心的 3D 和 4D 重建模型](https://github.com/continental/seed4d)

[使用 SUMO 和 CARLA 模拟 AR 挡风玻璃 HUD 对驾驶性能的影响](https://github.com/Pascal-Jansen/HUD-SUMO)

### 车辆 <span id="vehicle"></span>

[半挂卡车队列框架](https://github.com/Gaochengzhi/Carla_Truck_Platoon/tree/main)

### 传感器 <span id="sensor"></span>

[LiDAR 点云仿真和传感器放置（雨天的雷达）](https://github.com/PJLab-ADG/PCSim)

[基于语义射线投射传感器构建的简单雷达模拟器](https://github.com/delite3/CRST)


## 行人 <span id="pedestrian"></span>

[chrono读取opensim接口](https://api.projectchrono.org/classchrono_1_1parsers_1_1_ch_parser_open_sim.html) - 额外的 [数据仓库](https://gitlab.buaanlsde.cn/carla/chrono/-/tree/7.0.2/data/opensim)

[HuMoR：激活导航大脑](https://github.com/GuoYipeng279/myProject) - 人体姿态估计

[优化多视角行人检测的摄像头配置](https://github.com/hou-yz/MVconfig)

[仿生机器人](https://github.com/hty0111/Biorobotics)

[通过车与行人 (V2P) 通信避免碰撞](https://github.com/masskro0/carla_connected_mobility) - 演示了鬼探头

[行人社会力模型](https://github.com/felixlutz/carla-social-force-model)

[使用 zwift 室内自行车配件在 Carla 中骑自行车](https://github.com/tensorturtle/cycarla)

[开源自行车模拟器](https://github.com/tensorturtle/metacycle)

[智能人群场景分析引擎](https://github.com/adrielkuek/ITSS_2022)

[GUI 工具可帮助人车交互研究人员使用 CARLA 设计和开展交通实验](https://github.com/UmichSIM/SIM_MDP_GUI) - [首页](https://umichsim.github.io/SIM_MDP_GUI/)

[人类与汽车行为的模拟算法](https://github.com/Dawlau/agents-motion-prediction)

[针对横穿马路者的实验](https://github.com/javtran/jaywalker)

[带肌肉骨骼模型的强化学习环境](https://github.com/stanfordnmbl/osim-rl) - 基于opensim


## 智能体 <span id="agent"></span>

[受人类认知过程启发持续学习、适应和改进自动驾驶新范式](https://github.com/PJLab-ADG/LeapAD)

[构建一辆能够使用 SOTA 技术进化的 AI 汽车](https://github.com/good-sijin/EvolveCar)

[基于动作的自动驾驶表征学习](https://github.com/yixiao1/Action-Based-Representation-Learning)

[出在 BEV 中学习以对象为中心的表示](https://github.com/Shamdan17/CarFormer)

[机器学习演示](https://github.com/kishkaru/ML-demos)

[智能体训练栈](https://github.com/pAplakidis/ADA_training_stack)

[探索和绘制月球表面地图](https://github.com/Sthirunavukkarasu7450/PennStateLACTeam?tab=readme-ov-file)

[自动驾驶汽车](https://github.com/mayank-vekariya/SelfDriving-Carla) - 对应的 [毕业论文](https://github.com/mayank-vekariya/Carla-Master) 


## 多智能体 <span id="mulgi_agent"></span>

[可定制多智能体训练营](https://github.com/TrustAISys/cmad-gym)

[基于LLM的交通流生成](https://github.com/CXYyp5SkNg/CXYyp5SkNg.github.io) ，效果[视频](https://github.com/CXYyp5SkNg/CXYyp5SkNg.github.io/blob/main/README.md) 

[情境感知交通规则例外情况的知情强化学习](https://github.com/fzi-forschungszentrum-informatik/informed_rl)

[模拟器与 ROTRA 公路法规工具相结合](https://github.com/Kcuga/carla-rotra) 

[强化学习代理设置速度限制，旨在减少网络上的总排放量](https://github.com/gustavomoers/OptimalEmission-SUMO-ReinforcementLearning)

[多智能体交通场景gym](https://github.com/AutonomousDrivingExaminer/mats-gym) - 基于场景的 CARLA 训练和评估框架。

[多智能体联网自动驾驶 (MACAD) 训练平台](https://github.com/Kinvy66/MAD-ARL-latest)

[交通堵塞检测](https://github.com/Eclipse-SDV-Hackathon-Accenture/FEVio_ClosedLoopDrivingChallenge)

[自动驾驶模拟学习对提高交通规则遵守率有何影响](https://github.com/VVeiCao/p_csg_plus)

[车辆与车辆通信在改善道路拥堵和降低事故数量方面的作用](https://github.com/markoBel3/v2v-simulation)

[交通感知自动驾驶](https://github.com/laurayuzheng/TrAAD)

[联邦学习自动驾驶](https://github.com/SIAT-INVS/CarlaFLCAV)


## 可解释 <span id="explainability"></span>

[使用深度生成模型的反事实解释方法](https://github.com/darshandodamani/A-Counterfactual-Explanation-Approach-Using-Deep-Generative-Models)

[反事实解释](https://github.com/darshandodamani/KontrafaktischeErkl-rungzurAD-Masterarbeit) ：确定输入中的哪些微小变化会改变模型的决策。

[通过对象级表示可解释的Transformers](https://github.com/autonomousvision/plant)

[提出了一种可解释的自动驾驶汽车纵向控制的深度学习方法](https://github.com/limeng-1234/Pos_Shap)

[与模型无关的可解释技术：预期积分离散梯度](https://github.com/limeng-1234/EIDG)

[通过推理管道实现可验证的稳健学习](https://github.com/javyduck/CARE) - [其他实现](https://github.com/javyduck/care_armony) 

[可解释的基于目标的预测和规划](https://github.com/fayeiii/IGP2_GraduationProject)

[符合交通规则的城市自动驾驶统一决策和控制框架](https://github.com/henryhcliu/udmc_carla) - 还没代码

[使用可解释的传感器融合变压器实现安全增强型自动驾驶](https://github.com/opendilab/InterFuser)

[通过对象级表示实现可解释的规划转换器](https://github.com/autonomousvision/plant)

[使用类激活映射技术测试 CNN ](https://github.com/RocaPiedra/carla-simulator-CAM)

[可解释的基于目标的自动驾驶预测和规划](https://github.com/uoe-agents/IGP2)

[AI 可解释性模块集合](https://github.com/willparker123/fat-face)

[端到端自动驾驶中整体一致的可解释性](https://github.com/Robot-K/Hint-AD)

## 安全 <span id="security"></span>

[联网和自动驾驶汽车的攻击模拟框架](https://github.com/tum-esi/simutack) - 包含系统代码和视频

[轨迹数据隐私保护](https://github.com/pisanovo/pcis-privacy-protection-of-trajectory-data)

[自动驾驶传感器的网络安全测试与评估](https://github.com/Kitty9701/AttackLa)

[通过对抗轨迹对多目标跟踪进行物理攻击](https://github.com/ch3ny1/AdvTraj_ID_Transfer)

[面向端到端自动驾驶的模块式自适应对抗训练](https://github.com/waterluy/MA2T)

[自动驾驶缺陷测试平台](https://github.com/yagol2020/ADSFuzzEval)

[平衡安全性和性能](https://github.com/BaitingLuo/Dynamic_Simplex)

[通过运动学梯度生成稳健模仿的安全关键驾驶场景](https://github.com/autonomousvision/king)

[对交通标志识别和车道检测的对抗性攻击](https://github.com/FriedrichZimmer/CARLA-TSR-adversarial-benchmark)

[自动驾驶异常检测的多模式基准](https://github.com/fzi-forschungszentrum-informatik/anovox)

[传感器故障和攻击下的机器人系统的容错神经控制屏障功能](https://github.com/HongchaoZhang-HZ/FTNCBF)

[为自动驾驶生成可转移的对抗模拟场景](https://github.com/yasasa/adv-scenarios)

[使用场景图指定和监控安全驾驶属性](https://github.com/less-lab-uva/SGSM)

[通过增强基于场景的测试来提高自动驾驶汽车 (AV) 的安全性和可靠性](https://github.com/ldegao/TMfuzz)

[使用可解释传感器融合Transformer实现安全增强型自动驾驶](https://github.com/No4x/InterFuser)

[安全导航](https://github.com/ghadinehme/Safe-Navigation-Training-Autonomous-Vehicles-using-Deep-Reinforcement-Learning-in-CARLA) - 使用 CARLA 中的深度强化学习训练自动驾驶汽车

[自动驾驶汽车的安全分析](https://github.com/ElisabethHalvorsen/Human_and_AI_Comparison)

[用于寻找可能故障事件的自动驾驶汽车奖励建模](https://github.com/T3AS/ReMAV)

[让用户模拟给定攻击性等级的攻击性驾驶员的行为](https://github.com/necst/driver-aggressiveness-simulator)

[基于本体的正向安全关键仿真测试场景生成](https://github.com/initiative416/CornerCaseOntology)

[系统地查找原始车辆特性设置的最小变化会影响部署在车辆上的 ADS 的安全性](https://github.com/simplexity-lab/SAFEVAR)

[CARLA 中的自动驾驶汽车对抗性测试框架](https://github.com/scope-lab-vu/ANTI-CARLA)

[事故模拟器](https://github.com/hankluo2/CARLA_Accident_Simulator)

[通过运动学梯度生成稳健模仿的安全关键驾驶场景](https://github.com/autonomousvision/king)

[罕见事件采样用于安全验证](https://github.com/craigiedon/CarlaStuff)

[对抗性深度强化学习用于提高多智能体自动驾驶策略的鲁棒性](https://github.com/T3AS/MAD-ARL)

[使用机器学习对图像分类器的安全监控器进行基准测试](https://github.com/raulsenaferreira/PRDC_2021_SUT_module)

[自主信息物理系统运行时风险评估框架](https://github.com/scope-lab-vu/Resonate)

## 测试 <span id="test"></span>

[评估端到端](https://github.com/YangyangFu/fsd-agents-carla)

[在场景级别对自动驾驶系统进行模糊测试的框架](https://github.com/AtongWang/ScenarioFuzz)

[多智能体对抗测试平台](https://github.com/alanshuo123/AMATest)

[可加速自动驾驶汽车的开发、测试和部署](https://github.com/1127705669/EDrive)

[McMaster EcoCAR 联网和自动驾驶汽车团队的软件在环 (SIL) 测试环境](https://github.com/Oliverzzzzz/CavDevChallenge2023) - 包含车辆工厂模型、环境和模拟真实车辆行为的模拟传感器

[路边激光雷达评估](https://github.com/ElizabethSF/RoadSide-Lidar-Evaluation)

[基于交叉口情境覆盖的场景测试代码环境](https://github.com/initiative416/SituationCoverageTest)

[模拟器中自动驾驶汽车的形式化和验证](https://github.com/dnzggg/Masters-Project)

[基于学习的自动驾驶汽车语法模糊测试](https://github.com/AIasd/AutoCop)

[在高保真模拟器中模糊测试自动驾驶系统的开源软件包](https://github.com/AIasd/ADFuzz)

[基于CARLA模拟器的自动驾驶汽车行为测试](https://github.com/hitabm/carla-vehicle-testing)

[稳健性引导测试：CARLA、RSS、参数探索](https://github.com/nellro/rgt)

[测试自主代理的框架](https://github.com/MasoudJTehrani/PCLA)


## 数据集 <span id="dataset"></span>

[车辆语义分割数据集](https://github.com/yjzhai-cs/RFL-Collector)

[为自动驾驶模型生成训练数据](https://github.com/leunark/easycarla)

[YOLO数据生成器](https://github.com/RyangDiaz/carla-yolo-dataset-generator)

[为汽车和制造业用例生成合成数据](https://github.com/aws-samples/synthSceneGen)

[Carla 环境收集数据](https://github.com/Merealtea/CarlaEnv)

[在 Carla 模拟环境中高效收集数据](https://github.com/command-z-z/CarlaDataCollector)

[为自动驾驶生成合成数据](https://github.com/neetmehta/carla_data_tools)

[具有主动数据采集算法的数据收集工具](https://github.com/Kazawaryu/CARLA_ADA)

[驾驶场景批量生成器](https://github.com/tudo-aqua/stars-export-carla)

[生成模拟和真实的数据](https://github.com/Tottowich/carla-synthetic-data)

[从 carla 收集训练数据](https://github.com/FrankGu0911/CarlaDataCollection)

[无需专家标签的稳健车辆导航 LiDAR 视图合成](https://github.com/jonathsch/lidar-synthesis)

[车端/路端仿真数据集自动构建](https://github.com/Philipcjh/Carla-Simulation-Dataset-Generator)

[使用 Carla 0.9.13 导出数据（KITTI 格式）](https://github.com/TroyeFun/carla-export-data)

[CARLA 0.9.13 的数据生成工具](https://github.com/kkowol/CARLA-Datagen)

[收集数据的轻量级实用程序](https://github.com/bluffish/carla_collector)

[以专家策略作为收集数据时的策略](https://github.com/Kin-Zhang/carla-expert)

[从 CARLA 收集数据并将其保存为 Webdataset 的脚本](https://github.com/HemuManju/carla-data-collector)

[基于动态情况的相关传感器数据选择](https://github.com/akshaynarla/DySi_Select)

[行车记录仪视频事故预测](https://github.com/Tranito/SOD-Cog-TAA)

[像 Kitti 一样合成](https://github.com/richardmarcus/synth-it-like-kitti)

## 工具 <span id="tools"></span>

[自动驾驶模拟及相关任务的综合框架](https://github.com/femw03/DAI_project) - 集成了车辆控制、计算机视觉、强化学习环境和通过 CARLA 进行模拟的模块。

[在各种环境中尝试训练和迁移深度强化模型](https://github.com/Guthax/Sim2Sim2Real)

[提供 API 以将基于 CommonRoad 的工具与CARLA一起使用](https://github.com/CommonRoad/commonroad-carla-interface) 

[自动驾驶算法测试平台](https://github.com/siupal/UniversalPlatformForAutodriving)

[包含 CARLA 排行榜 2.0 的第一个完整入门套件](https://github.com/akodama428/carla_garage)

[驾驶试验](https://github.com/VIPERFGCU/FGCUDrivingSimulator)

[自动驾驶简介](https://github.com/mubashir1osmani/self-driving-car)

[gym 环境](https://github.com/natelincyber/carla-gym-optim)

[控制 CARLA 模拟器的 Web 界面](https://github.com/mateus-aleixo/carla-webui)

[系统将有三个不同的用户：车主、商业用户和系统管理员](https://github.com/mkovelamudi/AVRentals-main?tab=readme-ov-file)

[关于自动驾驶的书](https://github.com/YangyangFu/autonomous-driving-book)

[循序渐进的教程](https://github.com/dora-rs/dora-drives) - 从头开始编写自己的自动驾驶汽车程序

[《智能网联汽车决策控制技术》代码](https://github.com/DRL-CASIA/ICV-book)

[可立即使用的训练和评估环境](https://github.com/alberto-mate/CARLA-SB3-RL-Training-Environment)

[使用特定领域建模语言指定的自动驾驶汽车测试场景转化为具体场景](https://github.com/ArenBabikian/concretize)

[包括 LiDAR 点云和相机姿态的数据可视化应用程序](https://github.com/zhumorui/CARLA2NMR)

[绘制线条并保存参考点](https://github.com/suayu/Carla_line_drawer)

[提取地图信息（可行驶区域、车道、人行横道等）的代码](https://github.com/rohanNkhaire/CARLA_BEV_maps)

[运行端到端自动驾驶模拟的 Docker 容器](https://github.com/ruddyscent/adlab-e2e)

[用于在 CARLA+SUMO 下原型设计全栈协同驾驶自动化应用程序的通用框架](https://github.com/ucla-mobility/OpenCDA)

[基于开放联合仿真的研究/工程框架](https://github.com/shivamkumarpanda/OpenCDA-CEE298HW)

[OpenCDA文档](https://github.com/xiaxin2000/OpenCDA-Documents)

[驾驶辅助系统](https://github.com/commaai/openpilot) 

[sunnypilot是openpilot 的一个分支](https://github.com/r66auto/Sunnypilot2) - [另一个分支](https://github.com/RalphHightower/openpilot)

[增加了一些舒适性和生活质量改进](https://github.com/opgm/openpilot)

[协作式自动驾驶：仿真平台和端到端系统](https://github.com/CollaborativePerception/V2Xverse)

[基于世界模型的 CARLA 自动驾驶平台](https://github.com/ucd-dare/CarDreamer)

[模块化的基于规则的代理系统](https://github.com/Daraan/LunaticAI-Driver-for-CARLA-Simulator?tab=readme-ov-file)

[FLCAV 仿真平台](https://github.com/Creepersheep/CarlaFLCAV) - 多模态数据集生成、训练和推理、各种 FL 框架、基于优化的模块。

[使用 6 个 RGB 摄像头对自动驾驶汽车进行 BEV 表示](https://github.com/Raghuram-Veeramallu/DiffTransBEV)

[各种以模块化方式构建的功能](https://github.com/angelomorgado/CARLA-Ultimate-Template)

[使用 PyGame 可视化自我车辆的传感器](https://github.com/angelomorgado/CARLA-Sensor-Visualization)

[FHWA CARMA 实用程序脚本](https://github.com/davidgayman/fhwa-carma-utility-scripts)

[车辆跟踪监控](https://github.com/serene4uto/car-following_monitor)

[用于将特定类型的日志文件转换为基本 OpenScenario 格式的工具](https://github.com/lpwwpl/imitation/tree/main/OpenScenario_Converter-master)

[CARLA 模拟的 GUI 管理器](https://github.com/angelomorgado/CARLA-GUI-Manager)

[OpenCDA + 云端分布式/异步边缘计算](https://github.com/tlandle/eCloudSim)


[ROAR Group 最新的 Python 接口可控制模拟和现实中的事物](https://github.com/augcog/ROAR_PY) - 支持远程训练和兼容gym

[Carla ROS2 Bridge](https://github.com/JesusAnaya/tfm-carla-ros2-bridge)

[遥控驾驶模拟器](https://github.com/connets/tod-carla)

[开发模拟测试环境](https://github.com/jodi106/AI_Testing_Simulator)

[可视化车道跟踪模型](https://github.com/jjy0709/model_tester)

[开源库CARLANeT的 carla 端](https://github.com/carlanet/pycarlanet) - 用于 CARLA 与 OMNeT++ 之间的联合仿真

[OpenCDA 是一个基于开放联合仿真的研究/工程框架，集成了原型协同驾驶自动化](https://github.com/lpwwpl/opencda_lpw?tab=readme-ov-file)

[将 ML 模型与 BDI 代理相结合](https://github.com/alshukairi/MLMAS-Framework-AAMAS23)

[用于捕捉 NeRF 实验的场景](https://github.com/aasewold/carlo)

[Carla Sandbox——自动驾驶汽车的试验台](https://github.com/avstack-lab/carla-sandbox)

[将 Ray RLlib 集成到 CARLA 自动驾驶模拟器](https://github.com/resuldagdanov/carla-rllib-integration)

[CARLA 与 RLlib 集成](https://github.com/carla-simulator/rllib-integration)

[使用 Zenoh 桥接 Autoware 和 Carla](https://github.com/evshary/zenoh_carla_bridge)

[带有数字镜子的 Carla 模拟器客户端](https://github.com/doddsy2018/carla_sim_digital_mirrors) - 带后视镜

[用于控制自动驾驶模拟中的非玩家角色 (NPC) 的 API](https://github.com/inverted-ai/invertedai) - 可以作为REST API或在其上构建的Python SDK

[BARK 的 Carla 接口](https://github.com/bark-simulator/carla-interface)

[ROAR RL 管道的 ROS2 基础设施](https://github.com/amansrf/ROAR_RL_ROS)

[ROAR 的 ROS 子模块](https://github.com/amansrf/ros_roar_streamer)

[适用于 ROAR 平台的 Transform Publisher 包](https://github.com/amansrf/roar_transforms)

[carla-autoware](https://github.com/kemjensak/carla-autoware)

[开源自动驾驶汽车软件](https://github.com/yoonhero/nova)

[数据集生成工具](https://github.com/KevinLADLee/carla_dataset_tools)

[基于 Carla Simulator 的虚拟汽车租赁 Web 应用程序](https://github.com/lhy2016/AVCloud-Carla)

[可扩展的 CARLA 模拟器实用程序](https://github.com/eneserciyes/carlatools)

[Carla 模拟器游乐场，用于训练 AI 代理](https://github.com/Matesxs/CarlaSimulator-Playground)

[用于托管运行 TRI Carla 相关挑战的脚本的存储库](https://github.com/exoticDFT/TRI-Carla-Challenges)

[用于深度强化学习的多智能体互联自动驾驶 (MACAD) Gym 环境](https://github.com/praveen-palanisamy/macad-gym)

[自动驾驶强化学习快速启动小工具](https://github.com/siupal/CarlaAutodriveQuickStart) - 生成点选择器、拍摄地图全景、驾驶场景示例

[简化Carla和Autoware的安装](https://github.com/pradhanshrijal/pha_carlaware)

## 杂项 <span id="misc"></span>

[智能车辆系统项目](https://github.com/davidedimarco00/SVS-Project)

[AITyrimas](https://github.com/mantekarys/AITyrimas)

[通过专家协作增强自动驾驶系统](https://github.com/Expert-AD/ExpertAD)

[2024-tfg-felix-martinez](https://github.com/RoboticsLabURJC/2024-tfg-felix-martinez)

[多种传感器配置、实时可视化和自动数据收集](https://github.com/ssh-keyz/weathering-the-storm)

[CARLA-vehicle-simulation](https://github.com/TO-autonomy/CARLA-vehicle-simulation/tree/main)

[油管博主示例](https://github.com/vadim7s/SelfDrive)

[由多种遮挡事件组成的驾驶模拟基准](https://github.com/opendilab/DOS)

[东风比赛](https://github.com/donymorph/Dongfeng_competition)

[智能地面车辆竞赛](https://github.com/westpoint-robotics/AY21_IGVC)

[CarlaContest2023](https://github.com/niwatNETH/CarlaContest2023)

[NYCU 智能驾驶系统专题 2024 春季](https://github.com/HCIS-Lab/IDS_s24)

[NTU CSIE 物联网班实验室](https://github.com/leon123858/IOT2022)

[自动驾驶汽车课程作业](https://github.com/baj31415/self-driving-cars)

[壳牌环保马拉松 2022 自主编程比赛 ROS 软件包](https://github.com/KON-Team/shell_simulation)

[车辆自主模拟应用](https://github.com/lucas-cosson/m2project)

[自动停车项目](https://github.com/Louis2099/autonomous_parking_project)

[停车模拟](https://github.com/rubenmartinezgonzalez94/UADY-carla-tests)

[自动泊车](https://github.com/irinamrdhtllh/autonomous-parking-rl)

[交通状况演示](https://github.com/fdominik98/TrafficSituationDemo)

[毕设：自动驾驶救生](https://github.com/Project3Team/LifeSaver-Autonomous-Drive-System-LADS-Graduation-Project)

[毕设测试](https://github.com/FrankGu0911/GP_test1)

[视觉里程计](https://github.com/pranav-ap/carla_driver)

[新型数据驱动专家系统，旨在解决 CARLA Leaderboard 2.0 问题](https://github.com/DanielCoelho112/priboot)

[感知周围环境并根据传感器数据做出驾驶决策](https://github.com/ABHI2410/SelfDriving)

[使用多种深度强化学习方法学习在赛道上驾驶](https://github.com/Recoan0/DrAIve)

[通过量化逃生路线的变化来表征和减轻风险](https://github.com/zihengjackchen/iPrism)

[CARLA_GYM](https://github.com/lzqw/CARLA_GYM)

[carla-diffuser-bc](https://github.com/MacielMachado/carla-diffuser-bc)

[使用 VQ-VAE 和可微分优化学习自动驾驶的采样分布和安全过滤器](https://github.com/cisimon7/VQOptMain)

[自动驾驶汽车课程作业解决方案](https://github.com/robofar/autonomous-vehicles)

[包含无人机的功能](https://github.com/gionji/carla_surveillance_cams)

[设置 CARLA 模拟器并编程](https://github.com/lzqw/CARLA)  - 包含leaderboard 2的数据下载链接

[改进 Carla 模拟器中的边界框](https://github.com/Mofeed-Chaar/Improving-bouning-box-in-Carla-simulator)

[逐步在 Carla 模拟器中为汽车创建自动驾驶仪](https://github.com/ropama3/Carla_Simulator)

[CARLA 模拟器的有用教程](https://github.com/3F999/CARLA) - 包括天气分类、控制器、车道检测、目标检测、语义分割。

[在 Docker 容器中运行 CARLA 模拟器](https://github.com/olegsinavski/carla_ml)

[基于触觉的 CARLA 模拟的 ROS 包](https://github.com/MohamedOmda92/haptic_carla_pkg)

[对参考 TCP 模型进行基准测试](https://github.com/s-suryakiran/DriveVLM)

[模拟汽车行驶到乘客所在地并将他们送达目的地的过程](https://github.com/Barry-Tan/249Project_The_Taxi)

[使用表观遗传算法和注意力机制测试自动驾驶汽车](https://github.com/Simula-COMPLEX/EpiTESTER)

[实施和测试的自动驾驶汽车的控制、规划和感知测试](https://github.com/gokulkchenchani/self_driving)

[将 Carla 中收集的传感器数据发布到 eCAL 主题，并订阅包含自动应用于相应 Carla 车辆的汽车指令的主题](https://github.com/eclipse-ecal/ecal-carla-bridge)

[车辆跟随](https://github.com/serene4uto/car_following_carla_sim)

[驾驶惯例](https://github.com/w-wojtak/driving-routines-carla)

[自动驾驶汽车课程](https://github.com/ajayragh97/self_driving_cars)

[Carla 模拟器的控制、规划和感知](https://github.com/prasanna1511/selfdriving-carlasimulator)

[在地图上计算路线并使用控制器来遵循该路线](https://github.com/daniel-bogdoll/carla_hello_world)

[使用 Openpilot 系统、OP-Deepdive 和 Carla 模拟器进行的一些实验](https://github.com/ulrik2204/carla_experiments)

[Udacity 自动驾驶](https://github.com/thomasian06/udacity-self-driving-car)

[TARGET](https://github.com/kdfkjlk/TARGET)

[自动驾驶](https://github.com/vadim7s/SelfDrive)

[自动驾驶汽车模拟](https://github.com/ccalvin97/Self-Driving-Car)

[使用 ML-MAS 框架提高自动驾驶性能](https://github.com/Abhijith14/ML-MAS-Framework)

[测试Carla](https://github.com/kntPro/testing_carla)

[pcl_torch](https://github.com/Ruidi345/pcl_torch)

[使用 Python 和 CARLA 进行测试](https://github.com/caseyeaster39/cityslam-python)

[在 COSMOS 地图上跟踪 Ego 车辆](https://github.com/sunnieeee/CARLA)

[以技能为基础的GCRL](https://github.com/magenta1223/skill-basedGCRL)

[ndetr](https://github.com/hrz2000/ndetr)

[CASE22_碰撞_VS](https://github.com/marleyshan21/CASE22_Collision_VS)

[GCPrior](https://github.com/magenta1223/GCPrior)

[rlcarla](https://github.com/CSI-Woo-Lab/rlcarla)

[Carla_gym 中的自动驾驶与 RL](https://github.com/AccRay/Autonomous-Driving-with-RL-in-Carla_gym)

[深度学习](https://github.com/addinedu-amr-2th/deeplearning-repo-1)

[XAI611 项目提案的基准代码](https://github.com/tlatjddnd101/xai611_project_proposal_2023)

[XAI-carla-v9.14-Windows](https://github.com/xAI-Self-Driving-Research/XAI-carla-v9.14-Windows)

[提高通用汽车（尤其是雪佛兰 Volt）的 OP 性能和便利性](https://github.com/wonkuk02/tw-pilot)

[AutonomousDriving-HWs](https://github.com/ajam74001/AutonomousDriving-HWs/tree/main)

[研究人类和自动驾驶汽车之间的共享控制](https://github.com/UmichSIM/Wizard)

[自动驾驶多智能体系统](https://github.com/Abhijith14/MLMAS-Carla)

[自动驾驶监督期间升级分心警告的 VR 实验](https://github.com/canmanie-swov/EscalatingWarningsVRStudy)

[可执行脚本的集合](https://github.com/Nikitich2033/CARLA_0.9.13_scripts)

[自动驾驶管道，利用 Zenoh Flow 实现组件之间的通信](https://github.com/ZettaScaleLabs/STUNT)

[F1TENTH Gym 环境](https://github.com/2longAGO/Projet_IA)

[CL2 自动因果](https://github.com/kunalchandan/CL2-AutoDetective)

[人工势场自动驾驶算法](https://github.com/URIS-AutoVehicle/APF-Autonomous-Driving)

[tpcl_mcts](https://github.com/Ruidi345/tpcl_mcts)

[carla-bc](https://github.com/gustavokcouto/carla-bc)

[自动驾驶汽车和人工智能课程 WS 22/23 的一部分](https://github.com/RUB-AVAI/allassignmens-35)

[汽车驾驶示例](https://github.com/NEWSLabNTU/carla-examples)

[NTU IoT 2022 课程期末项目](https://github.com/NEWSLabNTU/2022-ntu-iot-final-project)

[AD-Stacks](https://github.com/caseymeiz/AD-Stacks)

[Carsmos](https://github.com/littlerants/Carsmos)

[Carla-CTS02](https://github.com/dikshant2210/Carla-CTS02)

[驾驶仪显示](https://github.com/ajouatom/apilot)

[权重和偏差集成教程](https://github.com/neuralsorcerer/wandb-tutorial)

[实验性图像转换工具](https://github.com/raulsenaferreira/SiMOOD)

[连接模拟器的示例项目](https://github.com/Eclipse-SDV-Hackathon-BCX/Group1-TruckCarla-eCAL)

[判断自动驾驶汽车的良好/不良行为](https://github.com/lttnml1/ca_disengagement)

[deep_rl_with_carla](https://github.com/mhmohammadirad/deep_rl_with_carla)

[简化和自动化设置 openpilot 开发环境的过程](https://github.com/jeroenlammersma/openpilot-dev)

[比赛套件](https://github.com/thecountif/CarlaContest)

[在 Carla 模拟器中运行的自动驾驶汽车算法](https://github.com/popesculuca00/Self-driving-car)

[从 World on Rails 学习驾驶](https://github.com/MorningClub/master-thesis)

[基于模拟的自主系统测试](https://github.com/MustafaEmreTelli/CMPE486-Term-Project)

[用于教育目的的多容器 Carla 项目模板](https://github.com/bounverif/starter-carla-0913)

[实验](https://github.com/ojalmaps/carla-experiments)

[从轨迹世界学习驾驶](https://github.com/mael25/CBS2)

[网络物理系统保证案例开发的自动化模式选择](https://github.com/scope-lab-vu/AV-Assurance)

[使用基于极限学习机的控制屏障函数在线自适应补偿模型不确定性](https://github.com/EmanuelSamir/adaptive-learning-qpcbfclf-elm)

[realistic_agent](https://github.com/pjw1/realistic_agent)

[Carla 模拟器客户端](https://github.com/satyamjay-iitd/CarlaClient)

[carlaapi](https://github.com/urasakikeisuke/carlaapi)

[学习如何在环形交叉路口漂移](https://github.com/angloth/auto-drift)

[使用深度生成模型压缩传感器数据以实现自动驾驶汽车的远程协助](https://github.com/daniel-bogdoll/deep_generative_models)

[auto_drive](https://github.com/superclocks/auto_drive)

[BTP-Project](https://github.com/aditya3434/BTP-Project)

[FY2021](https://github.com/NETH-TDET-Programing-Contest/FY2021)

[Carla_PPO](https://github.com/Stephanehk/Carla_PPO)

[autonomous-driving-car-to-car](https://github.com/Rigo74/autonomous-driving-car-to-car)

[carla_course](https://github.com/raywongstudy/carla_course)

[MoCAD 实验材料](https://github.com/liuyandong1988/MoCAD-experiment)

[AutoPilot](https://github.com/supercatex/AutoPilot)

[自动驾驶Carla代理](https://github.com/Teo03/self-driving-agent)

[carla_project](https://github.com/bradyz/carla_project)

[各种自动驾驶任务代码](https://github.com/KrishGupta25/SelfDrivingSTEM)

