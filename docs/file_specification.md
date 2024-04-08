

## Util/BuildTools/*

![](img/tuto_D_windows_debug/file_specification_build_tools.png)

* BuildUE4Plugins.bat : 构建虚幻引擎插件 StreetMap 
* Setup.bat：调用`Util/InstallersWin/*.bat`下载并生成 zlib、libpng、rpclib、Google Test、Recast & Detour、Fast-DDS (for ROS2)、Boost、Xercesc、Sqlite3、PROJ、Eigen、Chrono 库；设置资产下载URL；生成CMakeLists.txt.in；
* BuildLibCarla.bat：创建LibCarla的客户端和服务端；
* BuildOSM2ODR.bat：创建OSM转OpenDRIVE的库；




## 附加：构建步骤分析
### Update.bat
1. 环境变量初始化；
2. 获取最新的资产包；
3. 下载并解压资产内容；

    如果不存在7zip就用`powershell -Command "Expand-Archive '`命令进行解压。

#### [Makefile](https://seisman.github.io/how-to-write-makefile/index.html)
1. `make PythonAPI`

    执行`Util/BuildTools/BuildPythonAPI.bat`，真正执行的是`python setup.py bdist_egg bdist_wheel`，在`PythonAPI/carla/dist`目录下生成`*.egg`和`*.whl`文件；

2. `make launch`

    执行`Makefile`，调用`Util/BuildTools/Windows.mk`，真正执行的是`BuildLibCarla.bat`，构建LibCarla的服务端和客户端；

### Build
* `Xerces`是由Apache组织所推动的一项XML文档解析开源项目。
* `SQLite`是一款轻型的数据库，是遵守ACID的关系型数据库管理系统，它包含在一个相对小的C库中。
* `PROJ`是一种通用坐标转换软件，它将坐标从一个坐标参考系（CRS）转换为另一个坐标参考系。这包括地图投影和大地坐标变换。

