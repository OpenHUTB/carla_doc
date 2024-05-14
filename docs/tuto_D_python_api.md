

### 解读 `setup.py`

| 外部链接库                     | 含义                                                                                                       |
|---------------------------|----------------------------------------------------------------------------------------------------------|
| `shlwapi.lib` | 包含了大量的Windows字符串处理方法                                                                                     |
| `Advapi32.lib` | 包含的函数与对象的安全性，注册表的操控以及事件日志有关                                                                              |
| `ole32.lib` | 一般是需要生成一个唯一的uuid时用到                                                                                      |
|`shell32.lib` | 包含了一些与Shel相关的函数和数据结构的定义。这些函数和数据结构可以用于创建和管理windows资源管理器中的文件夹、文件、快捷方式等对象，以及实现一些与Shel相关的功能，如拖放、剪贴板操作、文件关联等。 |
| `ole32.lib` | 一般是需要生成一个唯一的uuid时用到                                                                                      |
| `libboost_python37-vc142-mt-x64-1_80.lib` | python调用c++的接口                                                                                           |
| `libboost_filesystem-vc142-mt-x64-1_80.lib` | boost的文件系统库                                                                                              |
| `rpc.lib` | 远程过程调用库，提供了客户端和服务器的实现。                                                                                   |
| `carla_client.lib` | LibCarla的客户端实现                                                                                           |
| `libpng.lib` | 图片处理库                                                                                                    |
| `zlib.lib` | 解压缩库                                                                                                     |
| `Recast.lib` | 要功能是将场景网格模型生成用于寻路的网格模型（Navigation Mesh，俗称**行走面**，标记哪些地方可行走的多边形网格数据结构）。                                   |
| `Detour.lib` | 主要功能是利用上一步所生成的Navigation Mesh进行**寻路**。                                                                   |
| `DetourCrowd.lib` | 提供了**群体**寻路行为的功能。                                                                                        |
| `xerces-c_3.lib` | XML文档解析开源项目。                                                                                             |
| `sqlite3.lib` | 嵌入式数据库。                                                                                                  |
| `proj.lib` | 通用坐标转换库。                                                                                                 |
| `osm2odr.lib` | OpenStreetMap转OpenDrive库。                                                                                |