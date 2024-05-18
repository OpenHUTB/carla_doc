# NVIDIA Omniverse 的 SimReady 内容

NVIDIA 的 [__SimReady__](https://developer.nvidia.com/omniverse/simready-assets) 规范支持准备专 __为仿真而构建的三维内容__，有助于简化仿真三维虚拟环境的内容创建管道，以实现机器人和自动驾驶中的机器学习目的。通过现已集成到 Carla 中的 Omniverse 虚幻引擎插件，用户只需点击几下即可导入 SimReady 内容，例如已配置工作灯、门、车轮和道具的车辆，可立即用于装饰 Carla  地图。 Carla  的 Omniverse 集成能够显着加速您的环境构建流程，并为 [__Omniverse 生态系统__](https://www.nvidia.com/en-us/omniverse/ecosystem/) 中的整个应用程序世界打开大门。

!!! 笔记
    Omniverse 虚幻引擎连接器目前仅在 Windows 中可用。

请按照以下步骤开始使用 Omniverse 并在 Carla  中使用 SimReady 内容：

首先，您应该先 [安装 NVIDIA Omniverse](https://docs.omniverse.nvidia.com/install-guide/latest/index.html)

### 1. 安装虚幻引擎 Omniverse 连接器

1. 启动 NVIDIA Omniverse 启动器
2. 导航至 **Exchange** 选项卡
3. 找到 __Epic Games Unreal Engine 4.26 Omniverse 连接器__ 
4. 确保安装版本为 **Release 105.1.578**
5. 单击 *安装*
6. Omniverse 将无法找到 Unreal Engine，请单击 **OK**。
7. 它会提示你找到你的虚幻引擎安装，选择这个路径：{UE4_ROOT}\Engine\Plugins\Marketplace\NVIDIA
8. 按 *安装*。

### 2. 设置本地 Omniverse 服务器

1. 使用启动器打开 NVIDIA Omniverse
2. 导航至*Nucleus*选项卡
3. 点击*Create Local Server*
4. 创建管理员详细信息
5. 选择*Local Nucleus Service*旁边的文件夹图标，这应该在 Web 浏览器中打开本地主机服务器

### 3. 连接 Carla  仿真器

1. 从 Carla   根文件夹命令行使用 `make launch` 启动 CARLA
2. 如果现有服务器处于活动状态并且您想要重新启动设置，请单击*Clean Local Assets*（可选） 
3. 选择 *Omniverse* 图标并单击 *Add Server*
4. 为服务器命名并单击*Add to Content Browser*
5. 登录表单应在您的浏览器中启动，单击*Create Account*
6. 创建管理员详细信息
7. Omniverse 文件夹现在应该在虚幻引擎内容浏览器的 Content/Omniverse 文件夹中可见
8. 再次访问您的网络浏览器
9. 单击*Connect to a Server*
10. 使用您为 Carla  服务器设置的服务器名称进行身份验证
11. 使用您为 Carla  服务器设置的管理员详细信息
12. 服务器文件夹现在应该显示在您的浏览器中

### 4. 导入 SimReady 资产

1. 导航至浏览器导航器中的“Projects”文件夹
2. 右键单击，然后选择*Upload Folder*
3. 选择您的 SimReady 文件夹
4. 上传文件

### 5. 使用 Omniverse 连接器将车辆加载到 CARLA

1. 在虚幻引擎中打开 Carla  项目
2. 导航至 `CarlaTools/Content/USDImporter`
3. 右键单击 *UW_USDVehicleImporterEditorWidget*
4. 选择 *Run Editor Utility Widget*
5. 在 Omniverse 浏览器导航器中查找车辆
6. 复制路径（应类似于：omniverse://localhost/Projects/SimReadyUSD.../vehicle.usd)
7. 将其粘贴到小组件内的 Omiverse URL 选项卡中
8. 选择 *Import Asset*
9. 您应该会看到车辆显示在引擎中您在 *Import destination* 字段中指定的文件夹中
10. 打开另一个地图场景（它可能已打开包含导入数据的新场景），应出现*Save Content* 提示。取消选中 *Untitled* 场景，然后单击 *Save selected* 以保存新内容
11. 该车辆现已可在 Carla  内使用，并将通过 Python API 提供
