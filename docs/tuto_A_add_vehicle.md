# [添加新车](https://carla.readthedocs.io/en/latest/tuto_A_add_vehicle/)

这个教程详细介绍了如何向 Carla 添加新的车辆。教程分为两个部分，一个用于四轮车辆，另一个用于两轮车辆。教程概述了建模车辆时必须满足的基本要求，以确保车辆在 Carla 中运行良好，并提供了在将车辆导入虚幻引擎后所需的配置说明。

*   [__添加 4 轮车辆__](#add-a-4-wheeled-vehicle)
  *   [绑定并建模车辆](#bind-and-model-the-vehicle)
  *   [导入并配置车辆](#import-and-configure-the-vehicle)
*   [__添加 2 轮车辆__](#add-a-2-wheeled-vehicle)

!!! 重要
    本教程仅适用于使用源代码构建并有权访问虚幻引擎编辑器的用户。
---
## 添加 4 轮车辆

添加到 Carla 的车辆需要使用可在 [__此处__]((https://carla-assets.s3.eu-west-3.amazonaws.com/fbx/VehicleSkeleton.rar)) 找到的通用基础骨架。此链接将下载一个名为 `VehicleSkeleton.rar` 的文件夹，其中包含两种不同格式的基本框架，一种为 ASCII，另一种为二进制。您使用的格式取决于您的 3D 建模软件要求。

__骨架骨骼的位置可以更改，但任何其他操作（例如旋转、添加新骨骼或更改当前层次结构）都会导致错误。__

---

### 绑定并建模车辆

本节详细介绍了车辆建模阶段的最低要求，以确保其可以在 Carla 中成功使用。该过程包括将骨架正确绑定到车辆的底座和车轮、创建物理资源和光线投射传感器网格，以及导出为正确的格式。

__1. 导入基础骨架。__

将基础骨架导入您首选的三维建模软件中。常见的编辑器包括 Maya 和 Blender。

__2. 绑定骨骼。__

根据下面的命名法将骨骼绑定到车辆网格的相应部分。确保轮子的骨骼在网格内居中。

*   __左前轮：__ `Wheel_Front_Left`
*   __右前轮：__ `Wheel_Front_Right`
*   __左后轮：__ `Wheel_Rear_Left`
*   __右后轮：__ `Wheel_Rear_Right`
*   __网格的其余部分：__ `VehicleBase`

!!! 警告
    不要对骨骼名称或层次结构进行任何更改，也不要添加任何新骨骼。

__3.  为您的车辆建模。__

车辆应拥有大约 50,000 - 100,000 个 tris。我们使用实际汽车的尺寸和比例来建模车辆。

我们建议您将车辆分为以下材质：

>1. __Bodywork__: 车辆的金属部分。该材质已更改为虚幻引擎材质。可以添加徽标和细节，但为了可见，必须使用虚幻引擎编辑器中的 Alpha 通道将它们绘制为不同的颜色。
- __Glass_Ext__: 一层玻璃，可以从车辆外部看到内部。
- __Glass_Int__: 一层玻璃，允许从车辆内部到外部的可见性。
- __Lights__: 头灯、指示灯等。
- __LightGlass_Ext__: 一层玻璃，允许从外部到内部的灯光可见性。
- __LightGlass_Int__: 一层玻璃，允许从内部到外部看到光。
- __LicensePlate__: 29x12 厘米的矩形平面。您可以使用 Carla 提供的 `.fbx` 以获得最佳结果，请在 [此处](https://carla-assets.s3.eu-west-3.amazonaws.com/fbx/LicensePlate.rar) 下载。纹理将在虚幻引擎中自动分配。
- __Interior__: 任何其他不适合上述部分的细节都可以进入 _内饰_.


材料（Material）应使用格式 `M_CarPart_CarName` 命名，例如 `M_Bodywork_Mustang`。

纹理（Textures）应使用格式 `T_CarPart_CarName`命名，例如 `T_Bodywork_Mustang`。纹理的大小应为 2048x2048。

虚幻引擎会自动创建(Level of Details, LOD)，但您也可以在 3D 编辑器中手动创建它们。Tri 计数如下：

- __LOD 0__: 100,000 tris
- __LOD 1__: 80,000 tris
- __LOD 2__: 60,000 tris
- __LOD 3__: 30,000 tris


__4. 创建物理资源网格。__

物理资源网格是一个附加网格，允许虚幻引擎计算车辆的物理特性。它应该尽可能简单，减少多边形数量，并且应该覆盖除车轮之外的整个车辆。请参阅下图的示例。

>>![physical asset mesh](./img/physical_asset_mesh.png)

物理资源网格应导出为单独的`.fbx`文件。最终文件应满足以下要求：

- 有一个基础网格。这应该是物理资源网格的副本。它应该与原始车辆具有相同的名称。
- 物理资源网格体必须使用格式  `UCX_<vehicle_name>_<number_of_mesh>`命名， __否则虚幻引擎将无法识别。__
- 网格不得超出原始模型的边界。
- 网格应与原始模型具有相同的位置。

>>![base mesh](./img/base_mesh.png)

将最终 `.fbx` 网格导出为名为 `SMC_<vehicle_name>.fbx`的文件。

__5. 为光线投射传感器创建网格。__

光线投射传感器网格设置将由光线投射传感器（RADAR、LiDAR 和 Semantic LiDAR）检测到的车辆形状。该网格应该具有比物理资产网格稍微更明确的几何形状，以提高传感器仿真的真实感，但出于性能原因，不如汽车网格那么详细。

创建光线投射传感器网格时请考虑以下几点：

- 网格应覆盖车辆的各个方面，包括车轮、后视镜和格栅。
- 轮子应为不超过 16 圈的圆柱体。
- 如果需要，可以将各种网格连接在一起。
- 网格不得超出原始模型的边界。
- 网格应该与原始网格具有相同的位置。

>>![collision mesh](./img/collision_mesh.png)

将最终网格导出名为 `SM_sc_<vehicle_name>.fbx` 的 `.fbx` 文件。

__5. 导出车辆网格。__

选择所有主要车辆网格和骨架底座并导出为`.fbx`。

---

### 导入并配置车辆

本节详细介绍了将车辆导入虚幻引擎以在 Carla 中使用的过程。在虚幻引擎编辑器中执行这些步骤。

__1. 创建车辆文件夹。__

在 `Content/Carla/Static/Vehicles/4Wheeled` 里创建一个名为 `<vehicle_name>` 的新文件夹。

__2. 导入 `.fbx`。__

在新的车辆文件夹中，通过在**_Content Browser_**中右键单击并选择 **_导入到 Game/Carla/Static/Vehicles/4Wheeled/<vehicle_name\>_** 来导入主车辆骨架`.fbx`。

在弹出的对话框中：

- 将导入内容类型 **_Import Content Type_** 设置为`Geometry and Skinning Weights`。
- 将 **_Normal Import Method_** 设置为 `Import Normals`。
- （可选）将**_Material Import Method_**设置为`Do not create materials`。取消选中**_Import Textures_**以避免虚幻引擎创建默认材质。 

骨架网格体将与两个新文件`<vehicle_name>_PhysicsAssets` and `<vehicle_name>_Skeleton`一起出现。

将其余`.fbx`文件与主车辆骨架`.fbx`文件分开导入。

__3. 设置物理资源网格。__

>1. 从内容浏览器**_Content Browser_** `<vehicle_name>_PhysicsAssets`。
- **_Skeleton Tree_**面板中右键单击 `Vehicle_Base` 的网格体，然后转到**_Copy Collision from StaticMesh_**。
- 搜索并选择您的`SMC_<vehicle_name>`文件。您应该会看到物理资源网格的轮廓出现在视口中。
- 从 `Vehicle_Base` 中删除默认的胶囊形状。
- 选择所有轮子：
    - 转到**_Tools_**面板并将“基元类型**_Primitive Type_**”更改为`Sphere`。 
    - 转到详细信息**_Details_**面板并将物理类型**_Physics Type_**更改为`Kinematic`。
    - 将线性阻尼**_Linear Damping_**设置为`0`。这将消除车轮上的任何额外摩擦。
- 为所有网格启用**_Simulation Generates Hit Event_**。
- 单击**_Re-generate Bodies_**。
- 将车轮球体调整至车轮尺寸。
- 保存并关闭窗口。

>![Collision mesh](./img/collision_mesh_vehicle.png)

__4. 创建动画蓝图。__

>1. 在内容浏览器**_Content Browser_**中，右键单击车辆文件夹并选择**_Animation -> Animation Blueprint_**。
- 在父类**_Parent Class_**中搜索并选择`VehicleAnimInstance`。
- 在目标骨架**_Target Skeleton_**中搜索并选择`<vehicle_name>_Skeleton`。 
- 按**_OK_**并将蓝图重命名为`AnimBP_<vehicle_name>`。 

__5.  配置动画蓝图。__

为了简化配置动画蓝图的过程，我们将从本地 Carla 车辆复制现有的动画蓝图：

>1. 转到`Content/Carla/Static/Vehicle`并选择任何 Carla 车辆文件夹。打开其动画蓝图。
- 在**_My Blueprint_**面板中，双击**_AnimGraph_**。您将看到图表出现在视口中。 
- 单击并拖动以选择**_Mesh Space Ref Pose_**、**_Wheel Handler_**和**_Component To Local_**组件。右键单击并选择复制。
- 返回您自己的车辆动画蓝图并将复制的内容粘贴到图形区域中。
- 单击**_Component To Local_**组件中的站立人物并将其拖动到**_Output Pose_**中的人物，以将组件连接在一起。
- 单击左上角的**_Compile_**。您现在应该看到一条脉动的线流过整个序列。
- 保存并关闭窗口。

>>![add_vehicle_step_04](img/add_vehicle_step_04.jpg)

__6. 准备车辆和车轮蓝图。__

>1. 在内容浏览器**_Content Browser_**中，转到`Content/Carla/Blueprints/Vehicles`并创建一个新文件夹`<vehicle_name>`。 
- 在该文件夹内，右键单击并转到**_Blueprint Class_**。在弹出窗口中打开**_All Classes_**部分。
- 搜索`BaseVehiclePawn`并按**_Select_**。
- 将文件重命名为`BP_<vehicle_name>`。
- 转到 `Carla/Blueprints/Vehicles` 中任何本地 Carla 车辆的文件夹。从内容浏览器**_Content Browser_**中，将四轮蓝图复制到您自己车辆的蓝图文件夹中。重命名文件以将旧车辆名称替换为您自己的车辆名称。

>>![Copy wheel blueprints](./img/copy_wheel_blueprint.png)

__7. 配置轮子蓝图。__

>1. 在车辆蓝图文件夹中，打开所有四个车轮蓝图。
- 在**_Class Defaults_**面板中，将 **_Collision Mesh_** 设置为 `Wheel_Shape`。__省略此步骤将导致车辆车轮陷入地面__。 
- 根据您的车辆规格调整车轮形状半径、宽度、质量和阻尼率的值。 
- 将轮胎配置**_Tire Config_**设置为`CommonTireConfig`。
- 在前轮上根据您的喜好设置转向角度 **_Steer Angle_**（默认为`70`）。取消选中**_Affected by Handbrake_** 。
- 在后轮上将转向角度 **_Steer Angle_** 设置为 `0`。检查 **_Affected by Handbrake_**。
- 设置悬架值时，您可以使用 [此处](tuto_D_customize_vehicle_suspension.md) 的值作为指导。
- 编译并保存。

>>![wheel shape](./img/wheel_shape.png)

__8. 配置车辆蓝图。__

>1. 从内容浏览器 **_Content Browser_** 中，打开您的`BP_<vehicle_name>`。 
- 在 **_Components_** 面板中，选择 **_Mesh (VehicleMesh) (Inherited)_** 。
- 在 **_Details_** 面板中，转到 **_Skeletal Mesh_** 并搜索并选择车辆的基础骨架文件（位于 `Carla/Static/Vehicles/4Wheeled/<vehicle_name>` 文件夹中）。 
- 转到 **_Details_** 面板中的 **_Anim Class_**。搜索并选择您的 `AnimBP_<vehicle_name>` 文件。
- 在 **_Components_** 面板中，选择 **_Custom Collision (Inherited)_** 。
- 在 **_Details_** 面板中选择 **_Static Mesh_** 并搜索您的 `SM_sc_<vehicle_name>` 光线投射传感器网格体。
- 在 **_Components_** 面板中，选择 **_VehicleMovement (MovementComp) (Inherited)_** 。 
- 在 **_Details_** 面板中，搜索 `wheel` 。您将找到每个轮子的设置。对于每一个，单击 **_Wheel Class_** 并搜索与正确车轮位置相对应的`BP_<vehicle_name>_<wheel_name>`文件。 

>>>>![wheel blueprint](./img/wheel_blueprint.png)

如果您的车辆有任何与基础网格分开的附加网格（门、灯等）：

>1. 将它们拖到**_Components_**面板中的 **_Mesh (VehicleMesh) (Inherited)_** 层次结构中。
- 选择层次结构中的额外网格并在**_Details_**面板中搜索 `Collision`。 
- 将碰撞预设 **_Collision Presets_** 设置为 `NoCollision`。
- 选择层次结构中的任何灯光网格。在 **_Details_** 面板中搜索 `Tag` 并添加标签 `emissive`。 

单击 **_Save_** 并 **_Compile_** 。

- 我们还可以[自定义车辆的车牌](custom_plate.md)。



__9. 将车辆添加到蓝图库中。__.

>1. 在 `Content/Carla/Blueprint/Vehicle` 中，打开 `VehicleFactory` 文件。
- 在 **_Generate Definitions_** 选项卡中，双击 **_Vehicles_** 。
- 在 **_Details_** 面板中，展开 **_Default Value_** 部分并向车辆数组添加一个新元素。 
- 填写您车辆的 **_Make_** 和 **_Model_** 。 
- 使用您的 `BP_<vehicle_name>` 文件填写 **_Class_** 值。
- （可选）为车辆提供一组推荐颜色。
- 编译并保存。

>![vehicle factory](./img/vehicle_factory.png)

__10. 测试车辆__。

启动 Carla，打开终端 `PythonAPI/examples` 并运行以下命令：

```sh
python3 manual_control.py --filter <model_name> # The make or model defined in step 9
```

!!! 笔记
    即使您在品牌和型号中使用了大写字符，它们在传递给过滤器时也需要转换为小写字符。
---

## 添加 N 轮车辆

添加 N 轮车辆遵循与上述 4 轮车辆相同的导入管道，但有几个步骤不同。

__5.__ __配置 N 轮车辆的动画蓝图__

搜索 `BaseVehiclePawnNW` 并按 **_Select_**。

![n_wheel_base](./img/base_nw.png)

__6.__ __准备车辆和车轮蓝图__

转到 Carla/Blueprints/Vehicles 中所有本地 Carla 车辆的文件夹。从内容浏览器中，将四轮蓝图复制到您自己车辆的蓝图文件夹中。重命名文件以将旧车辆名称替换为您自己的车辆名称。

复制四个轮子，然后再次复制其他轮子。如果是 6 轮车辆，您将需要 6 个不同的车轮：FLW、FRW、MLW、MRW、RLW、RRW。

![n_wheel_bps](./img/nwheels.png)

__7.__ __配置轮子蓝图__

对于 4 轮车辆，请遵循上述第 __7__ 节。 N 轮车辆的主要区别在于手刹和转向参数的影响。在某些车辆（例如长轴距卡车）中，前 2 对车轮将进行转向，其中一组可能比其他车轮转向更多。最后面的一对可能会受到手刹的影响，具体情况取决于您正在建模的车辆。

__8.__ __配置车辆蓝图__

在“详细信息”面板中，搜索`wheel`。您将找到每个轮子的设置。对于每一个，单击车轮类别并搜索 BP_<vehicle_name>_<wheel_name> 对应于正确车轮位置的文件。

这是正确的，但只是为了指定，在 N 轮车辆的情况下，您需要设置所有车轮。这是 6 轮车辆的示例：

![n_wheel_config](./img/nwheel_config.png)


最后，一个额外的考虑因素是设置差速器。对于 4 轮车辆，我们有不同的差速器预设（限滑、开放 4W 等），但对于 N 轮车辆，您需要选择要在哪些车轮上应用扭矩。本例中我们选择只有中后轮有扭矩，前轮没有扭矩，您可以指定其他配置。这些数字将与本文上方的图像相同（例如，0 将是左前轮，如上所述）。

![n_wheel_mech](./img/nwheel_mech_setup.png)

所有其他参数，如发动机、变速箱、转向曲线，均与四轮车辆相同。

---
## 添加 2 轮车辆

添加 2 轮车辆与添加 4 轮车辆类似，但由于动画的复杂性，您需要设置额外的骨骼来引导驾驶员的动画。[这](https://carla-assets.s3.eu-west-3.amazonaws.com/fbx/BikeSkeleton.rar) 是两轮车辆参考骨架的链接。

与 4 轮车辆一样，将模型朝向正“x”，每个骨骼轴朝向正 x，z 轴朝上。

```yaml
Bone Setup:
  - Bike_Rig:                   # 网格的原点。将其放置在场景的0点
    - BikeBody:                 # 模型身体的中心。
      - Pedals:                 # 如果车辆是自行车，将脚踏板绑在这块骨头上，就会随着自行车的加速度而旋转。
        - RightPedal:           # 如果车辆是自行车，则设置驾驶员的脚部位置并与脚踏板一起旋转。
        - LeftPedal:            # ^
      - RearWheel:              # 车辆后轮
      - Handler:                # 与车辆的前轮一起旋转，将车辆操纵器固定在其上。
        - HandlerMidBone:       # 定位在前轮骨架上，以确定操纵器与车轮的方向
        - HandlerRight:         # 设置驾驶员的手的位置，无需将其绑定到任何东西。
        - HandlerLeft:          # ^
      - Frontwheel:             # 车辆的前轮。
      - RightHelperRotator:     # 这四块额外的骨头是为一个过时的系统提供的，该系统通过使用传统的隐形车轮使自行车稳定 
        - RightHelprWheel:      # ^
      - LeftHelperRotator:      # ^
        - LeftHelperWheel:      # ^
      - Seat:                   # 设置驾驶员髋骨的位置。无需将其绑在任何东西上，只需小心放置即可。
```

__1.__ 将 fbx 作为 Skelletal Mesh 导入到其自己的 `Content/Carla/Static/Vehicles/2Wheeled` 文件夹中。导入时，选择“General2WheeledVehicleSkeleton”作为骨架，应自动创建并链接物理资源。

__2.__ 调整物理资源。删除自动创建的框并向 `BikeBody` 骨骼添加框，尝试尽可能匹配形状，确保启用生成命中事件。为每个轮子添加一个球体并将其“物理类型”设置为“运动学”。

__3.__ 创建 `Content/Blueprints/Vehicles/<vehicle-model>` 文件夹  

__4.__ 在该文件夹内创建两个派生自“VehicleWheel”类的蓝图类。调用 `<vehicle-model>_FrontWheel` 和 `<vehicle-model>_RearWheel`。将其 "Shape Radius" 设置为与网格轮半径完全匹配（小心，半径不是直径）。将其“轮胎配置”设置为“CommonTireConfig”。在前轮上取消选中“受手刹影响”，在后轮上将“转向角”设置为零。

__5.__ 在同一文件夹内创建一个派生 `Base2WheeledVehicle` 的蓝图类，并调用它 `<vehicle-model>` 。打开它进行编辑，选择组件“Mesh”，将“Skeletal Mesh”和“Anim Class”设置为相应的。然后选择 VehicleBounds 组件并设置大小以覆盖车辆区域（如上图所示）。

__6.__ 选择组件“VehicleMovement”，在“Vehicle Setup”下展开“Wheel Setups”，设置各个车轮。

*   __0:__ 轮类=`<vehicle-model>_FrontWheel`, 骨骼名称=`FrontWheel`  
*   __1:__ 轮类=`<vehicle-model>_FrontWheel`, 骨骼名称=`FrontWheel`  
*   __2:__ 轮类=`<vehicle-model>_RearWheel`, 骨骼名称=`RearWheel`  
*   __3:__ 轮类=`<vehicle-model>_RearWheel`, 骨骼名称=`RearWheel`  
  （您会注意到，我们基本上在每个骨骼中放置了两个轮子。unreal 提供的车辆类别不支持车轮编号不同于 4 的车辆，因此我们必须让它相信车辆具有4个轮子）

__7.__ 选择变量 "is bike" ，如果您的型号是自行车，则勾选它。这将激活踏板旋转。如果您要安装摩托车，请不要标记。 

__8.__ 找到变量 back Rotation 并将其设置为更合适的位置，更好地选择组件 SkeletalMesh（驱动程序）并将其沿 x 轴移动，直到其位于座椅位置。 

__9.__ 测试一下，进入CarlaGameMode蓝图并将“Default Pawn Class”更改为新创建的自行车蓝图。
