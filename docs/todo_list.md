# 计划

* [车辆碰撞](https://github.com/OpenRadioss/OpenRadioss)

* 行人控制和自主导航

* [VS硬盘版制作](https://theoractice.github.io/)


## 发布可执行程序
* `make launch`时报错：
```text
could not find any instance of Visual Studio.
```
1. 终止`BuildLibCarla.bat`的执行可以开始编译虚幻编辑器。
2. 注释掉`Util\BuildTools\BuildCarlaUE4.bat`中“构建 Carla 编辑器”的部分。


* 启动虚幻编辑器时报错：
```text
由于找不到 XINPUT1_3.dll，无法继续执行代码。
```
该动态库文件位于`engine\Engine\Binaries\ThirdParty\AppLocalDependencies\Win64\DirectX`目录下，应该是虚幻引擎编译时向系统注入了一些动态库文件。

使用命令行编译工程sln
```shell
"C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Enterprise\\Common7\\IDE\\\devenv.com" UE4.sln /Build
```