# 如何链接 Epic 的汽车材质

!!! 重要
    自 0.8.0 版本起，CARLA 默认不使用 Epic 的汽车材质。但是，如果您从源代码编译，仍然可以启用它们。


Epic Game 提供了一套免费使用的逼真的汽车材质。在本文档中，我们将解释如何下载这些材质并将其链接到我们的车辆，以获得更逼真的车漆。


### 从市场下载
Epic Games 的汽车材料包可以从虚幻引擎市场免费下载。

!!! 笔记
    不幸的是，虚幻引擎的市场仅在 Windows 上可用，因此您需要一台 Windows 机器来下载此软件包。
    
    
1. 从 [虚幻引擎网站](www.unrealengine.com) 安装 Epic Games Launcher 。
2. 以 0 美元的价格购买 [汽车材料包](https://www.unrealengine.com/marketplace/automotive-material-pack) 。
3. 创建一个新的虚拟项目并向其中添加汽车材料包。
4. 在新建项目的“Content”文件夹中，您可以找到“AutomotiveMaterials”文件夹。将此文件夹复制到 CARLA 项目
- <font color="#f8805a">`{NewProject}/Content/AutomotiveMaterials`</font>--> <font color="#f8805a">`{CARLA}/Unreal/CarlaUE4/Content/AutomotiveMaterials`</font>


## 手动链接材料
打开项目后，您应该链接刚刚下载的汽车材料。

在内容浏览器中，转到 `Content/Static/Vehicles/GeneralMaterials/MaterialRedirectors` 并打开 `RedirectorInstance` 。

![](./img/redirector_instance.png)

在“详细信息”窗格下，搜索“父”材料，并用“M_Carpaint”材料替换“DummyCar”材料。

![](./img/materials_screenshot_01.png)

现在保存“RedirectorInstance”，您就可以开始了。