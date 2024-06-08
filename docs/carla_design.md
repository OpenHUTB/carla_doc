# CARLA 设计
*本文档尚在编写中，可能不完整。*

CARLA由以下模块组成

- 客户端
* Python 客户端 API：“PythonClient/carla”
- 服务器端
* CarlaUE4 虚幻引擎项目：“Unreal/CarlaUE4”
* 虚幻引擎的 Carla 插件：“Unreal/CarlaUE4/Plugins/Carla”
* Carla服务器：“Util/CarlaServer”

!!! 笔记
    通过在 CARLA 项目主文件夹中运行 [Doxygen](http://www.doxygen.org/) 可以生成 C++ 代码的文档 。
    
    
## Python 客户端 API
客户端 API 提供了一个用于与 CARLA 服务器通信的 Python 模块。在“PythonClient”文件夹中，我们提供了几个使用“carla”模块编写 CARLA 客户端脚本的示例。


## CarlaUE4虚幻引擎项目
Unreal 项目“CarlaUE4”包含生成 CARLA 二进制文件的所有资产和场景。它使用 Carla 插件提供的工具来组装场景中的城市和代理的行为。

## 虚幻引擎的 Carla 插件
Carla 插件包含 CARLA 的所有功能。我们尝试将此功能与资产分开，以便可以在任何虚幻引擎项目中尽可能多地使用此插件中的功能。

它使用“CarlaServer”库进行网络通信。

## CarlaServer
用于网络通信的外部库。

有关实施细节，请参阅“[CarlaServer](./carla_server.md) ” 。