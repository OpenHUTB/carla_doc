# 悬挂模型

悬架子系统是轮式车辆的一个轴的模型。基类 [ChSuspension](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_suspension.html) 规定，任何派生的悬架类（悬架模板）都提供两个车轮主轴（左和右），每个车轮主轴通过旋转接头连接到该类型悬架的某个部分，以及两个主轴轴（[ChShaft](https://api.projectchrono.org/classchrono_1_1_ch_shaft.html) 类型的元素），如果该轴被驱动，则可连接到车辆传动系统。

派生悬架类型定义了特定类型悬架的主体、关节、力元素和拓扑结构。所有位置均假定相对于悬架参考系提供（派生悬架类型可以自由选择此参考系的位置，但不能选择其方向，假定方向与底盘 ISO 参考系平行）。

通过指定悬架组件参考系相对于底盘参考系的位置（参见 [ISO 参考系](https://api.projectchrono.org/vehicle_overview.html#vehicle_ISO_frame) 的定义），将悬架组件连接到车辆底盘上。


## 双叉臂
独立可操纵悬架，使用两个叉臂控制臂（也称为 A 臂）连接转向节和底盘。用作HMMWV车辆模型的前悬架和后悬架。

参见 [ChDoubleWishbone](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_double_wishbone.html) 和 [DoubleWishbone](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_double_wishbone.html) 。（wishbone 叉骨，如愿骨（吃家禽等时两人将颈与胸之间的 V 形骨拉开，得大块骨者可许愿））。

![](../img/chrono/DoubleWishbone_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/DoubleWishbone_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/DoubleWishbone_points.png)

## 双叉臂（简化）
这种简化的双叉臂悬架模板使用两个距离约束来模拟下控制臂和上控制臂。当控制臂的质量和惯性相对于系统中的其他物体较小，因此可以忽略不计时，这种悬架类型是合适的。

参见 [ChDoubleWishboneReduced](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_double_wishbone_reduced.html) 和 [DoubleWishboneReduced](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_double_wishbone_reduced.html) 。

![](../img/chrono/DoubleWishboneReduced_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/DoubleWishboneReduced_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/DoubleWishboneReduced_points.png)

## 麦弗逊支柱
可转向独立悬架系统，适用于前轮驱动、横向安装发动机/变速箱的中小型乘用车。

参见 [ChMacPhersonStrut](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_mac_pherson_strut.html) 和 [MacPhersonStrut](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_mac_pherson_strut.html) 。

![](../img/chrono/MacPhersonStrut_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/MacPhersonStrut_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/MacPhersonStrut_points.png)

## 多连杆
该悬架系统类似于双叉臂轴，拖曳臂可以承受较大的纵向力。

参见 [ChMultiLink](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_multi_link.html) 和 [MultiLink](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_multi_link.html) 。

![](../img/chrono/MultiLink_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/MultiLink_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/MultiLink_points.png)

## 半拖曳臂
小型乘用车采用简单的独立轴系统作为后悬架。

参见 [ChSemiTrailingArm](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_semi_trailing_arm.html) 和 [SemiTrailingArm](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_semi_trailing_arm.html) 。

![](../img/chrono/SemiTrailingArm_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SemiTrailingArm_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SemiTrailingArm_points.png)

## 实心轴
由四个连杆引导的实心轴系统。它通常使用螺旋弹簧或空气弹簧，常见于较旧的乘用车。

参见 [ChSolidAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_solid_axle.html) 和 [SolidAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_solid_axle.html) 。

![](../img/chrono/SolidAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SolidAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SolidAxle_points.png)

## 实心三连杆轴
用作 [MAN 5t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__5t.html) 、[MAN 7t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__7t.html) 和 [MAN 10t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__10t.html) 卡车型号的后悬架。这种悬架允许非常高的车轮行程，这是板簧无法实现的。它也用于带有空气弹簧的公路卡车。空气弹簧和螺旋弹簧需要由连杆引导的悬架。

参见 [ChSolidThreeLinkAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_solid_three_link_axle.html) 和 [SolidThreeLinkAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_solid_three_link_axle.html) 。

![](../img/chrono/SolidThreeLinkAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SolidThreeLinkAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SolidThreeLinkAxle_points.png)

## 带摇臂的实心三连杆轴
用作 [MAN 5t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__5t.html) 、[MAN 7t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__7t.html) 和 [MAN 10t](https://api.projectchrono.org/classchrono_1_1vehicle_1_1man_1_1_m_a_n__10t.html) 卡车型号的前悬架。

参见 [ChSolidBellcrankThreeLinkAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_solid_bellcrank_three_link_axle.html) 和 [SolidBellcrankThreeLinkAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_solid_bellcrank_three_link_axle.html) 。

![](../img/chrono/SolidBellcrankThreeLinkAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SolidBellcrankThreeLinkAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SolidBellcrankThreeLinkAxle_points.png)

## 板簧实心轴
用作 [UAZ](https://api.projectchrono.org/classchrono_1_1vehicle_1_1uaz_1_1_u_a_z_b_u_s.html) 车型的后悬架。板簧轴具有复杂的导向行为。这是一种模拟解决方案，其中板簧的导向效果通过轴管中心的特殊接头模拟。悬架效果由螺旋弹簧模拟。滚动行为接近真实的板簧轴。

参见 [ChLeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_leafspring_axle.html) 和 [LeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_leafspring_axle.html) 。

![](../img/chrono/LeafspringAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/LeafspringAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/LeafspringAxle_points.png)

## 带趾杆的板簧实心轴
用作 [UAZ](https://api.projectchrono.org/classchrono_1_1vehicle_1_1uaz_1_1_u_a_z_b_u_s.html) 车型的前悬架。

参见 [ChToeBarLeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_toe_bar_leafspring_axle.html) 和 [ToeBarLeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_toe_bar_leafspring_axle.html) 。

![](../img/chrono/ToeBarLeafspringAxle_bodies.png)


该悬挂模板的拓扑结构为：

![](../img/chrono/ToeBarLeafspringAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/ToeBarLeafspringAxle_points.png)

## SAE 板簧实心轴
SAE 弹簧设计手册介绍了一种在负载条件下具有真实变形行为的板簧建模方法。板簧的运动学可以用 5 个点来定义。这些点可用于定义由 6 个刚体（前板簧、前夹钳、后夹钳、后板簧和卸扣）组成的板簧。这些刚体通过关节连接。前板簧和后板簧以及前夹钳和后夹钳的旋转弹簧具有旋转刚度，用户可以设置该刚度以定义正确的行为。该悬架用作 [UAZ](https://api.projectchrono.org/classchrono_1_1vehicle_1_1uaz_1_1_u_a_z_b_u_s.html) 车辆 SAE 型模型的后悬架。该板簧模型可以正确模拟由于车轮行驶而引起的轴体运动以及由于纵向力而引起的束缚效应。

参见 [ChSAELeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_leafspring_axle.html) 和 [SAELeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_leafspring_axle.html) 。

![](../img/chrono/SAELeafspringAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SAELeafspringAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SAELeafspringAxle_points.png)

## 带趾杆的 SAE 板簧实心轴
用作 [UAZ](https://api.projectchrono.org/classchrono_1_1vehicle_1_1uaz_1_1_u_a_z_b_u_s.html) SAE 型车型的前悬架。板簧定义与 SAE 板簧轴相同。

参见 [ChSAEToeBarLeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_toe_bar_leafspring_axle.html) 和 [SAEToeBarLeafspringAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_s_a_e_toe_bar_leafspring_axle.html) 。

![](../img/chrono/SAEToeBarLeafspringAxle_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/SAEToeBarLeafspringAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/SAEToeBarLeafspringAxle_points.png)

## 三连杆独立后悬架
三连杆独立后悬架 (IRS)，如 Polaris RZR 车辆上所见。

参见 [ChThreeLinkIRS](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_three_link_i_r_s.html) 和 [ThreeLinkIRS](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_three_link_i_r_s.html) 。

![](../img/chrono/ThreeLinkIRS_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/ThreeLinkIRS_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/ThreeLinkIRS_points.png)


## 刚性悬挂
简单组装，主轴直接连接到轴管上，轴管固定在底盘上。这是农用拖拉机和联合收割机的典型装配。

参见 [ChRigidSuspension](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_rigid_suspension.html) 和 [RigidSuspension](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_rigid_suspension.html) 。

![](../img/chrono/RigidSuspension_bodies.png)

该悬挂模板的拓扑结构为：

![](../img/chrono/RigidSuspension_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/RigidSuspension_points.png)

## 刚性销轴
简单组件，主轴直接连接到轴管上，可以围绕枢轴点摆动，抵靠底盘。如果需要非悬挂轴系统，但又必须在起伏的地形上行驶以避免车轮抬起，则可以使用此组件。

参见 [ChRigidPinnedAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_rigid_pinned_axle.html) 和 [RigidPinnedAxle](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_rigid_pinned_axle.html) 。

![](../img/chrono/RigidPinnedAxle_bodies.png)

该悬挂模板的拓扑结构为：
![](../img/chrono/RigidPinnedAxle_topology.png)

硬点（仅为左侧定义并镜像构建右侧）是：

![](../img/chrono/RigidPinnedAxle_points.png)

## 通用轮式车辆悬架模板
此模板允许指定具有任意拓扑的用户定义的自定义悬架子系统。通过添加主体、关节、距离约束和 TSDA 来构建具体的悬架。每个建模组件都可以标记为“镜像”。用户负责定义所有非镜像组件和左侧镜像组件（相应的右侧组件会自动生成）。

参见 [ChGenericWheeledSuspension](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_generic_wheeled_suspension.html) 和 [GenericWheeledSuspension](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_generic_wheeled_suspension.html) 。


