# 虚幻引擎高级特性

## [控制台命令](https://blog.csdn.net/somnusand/article/details/115511383) 

启动模拟后，输入`~`键调用控制台命令。

`stat rhi`: Rendering Hardware Interface，显示RHI内存和性能统计数据。

`HighResShot 3840x2160` —— 指定分辨率截图（分辨率），图片存储位置：`\Saved\Screenshots\`。


## [渲染管线](https://zhuanlan.zhihu.com/p/373995698) 

1. 动画模拟
2. 光线传播
3. 几何表示
4. 光栅化

绘制调用（draw call）：在计算机图形学中，指向图形处理器发送指令以绘制图形的过程。

光栅化：找出三角形所覆盖到的像素；

## 内容

### 编译
[编译配置参考](https://docs.unrealengine.com/4.26/zh-CN/ProductionPipelines/DevelopmentSetup/BuildConfigurations/)

### 像素流插件
相关源码位于：`unreal\Engine\Plugins\Media\PixelStreaming\PixelStreaming.uplugin`

### 编辑器界面汉化
资源存储位置：`engine\Engine\Content\Localization\Editor\zh-Hans`，只需使用文本编辑器（例如 Notepad++）编辑`Editor.archive`。

### 问题
* 右键`.uproject`文件没有`Switch Unreal Engine version...`
解决：双击`Engine\Binaries\Win64\UnrealVersionSelector-Win64-Shipping.exe`，出现`Register this directory as an Unreal Engine installation?`后点击`是(Y)`。

* 增加`matlab`插件进行虚幻引擎编译，导致启动虚幻编辑器启动失败，原因不明。

* 如果启动虚幻编辑器时出现选择`renderdocui.exe`的选项，则从 [RenderDoc](https://renderdoc.org/) 中下载并安装软件，然后选择相应的exe文件即可启动。

## [Git 版本管理](https://zhuanlan.zhihu.com/p/104197715)

1. 源码控制->连接到源代码管理->Git；
2. Git路径选择Git安装后bin文件中的启动程序；选择`接收设置`



## [法律风险](https://www.3dcat.live/share/ue4-vsualization-software/) 

UE4虽然给了完整的源码，但并不意味着全部都能用于发布产品。

[条款](https://www.unrealengine.com/en-US/eula/publishing) 阐述了许可授予的相关问题，其中明确规定了发布的产品中不能包含引擎内容和未经过打包的付费内容。

引擎内容就是UE4编辑器相关的各种工具，小到移动旋转缩放轴，大到刷草刷树的地形编辑工具，这些代码都不能打进自己的项目里。

付费内容是从市场花钱购买的插件或工具，如果未经打包就发布到产品中，就相当于你买了一个模型以后直接发到网上，别人可以免费拷走，侵犯了作者的利益，当然也是不允许的。


## 其他 
- [__虚幻引擎中文文档__](https://bitbucket.org/openhutb/engine_doc/src/master/)

- [虚幻引擎官方中文文档](https://www.unrealengine.com/zh-CN/uses/simulation)

- [代码规范](https://dev.epicgames.com/documentation/zh-cn/unreal-engine/epic-cplusplus-coding-standard-for-unreal-engine)





