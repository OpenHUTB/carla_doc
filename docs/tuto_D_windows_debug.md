# 在 Windows 上进行 Carla 的调试

1. 进入目录`carla/Unreal/CarlaUE4/`，右键文件`CarlaUE4.uproject`，选择运行`Generate Visual Studio project files`，在当前目录中将会生成VS的工程文件，双击打开`CarlaUE4.sln`。

![](img/tuto_D_windows_debug/generate_vs_project_files.png)

!!! 笔记
    如果右键菜单中未出现`Generate Visual Studio project files`选项，则到虚幻引擎的目录中运双击执行`engine\Engine\Binaries\Win64\UnrealVersionSelector.exe`，将虚幻引擎软件注册到系统中。

2. 在`解决方案`中展开`Games->CarlaUE4`，在想要查看的源代码行的最左侧单击增加断点（比如：`CarlaUE4->Plugins->CarlaTools->Source->CarlaTools->Private`的`OpenDriveToMap.cpp`的`GenerateTileStandalone()`），在菜单运行`调试(D)->开始调试(S)`，程序将在断点出暂停。通过`调式(D)->窗口(W)->监视(W)->监视 1`打开变量监视窗口，查看变量值是否异常。
3. 
![](img/tuto_D_windows_debug/debug_project.png)


## VS2019 打开 CarlaUE4 的 Cmake 工程
windows操作系统下通过vs2019打开并编译carla：

1. 开Carla的CMake项目：

从 VS 的菜单中选择 `File-->Open-->CMake`, 在对话框中找到 Carla 所在的本地文件夹（包含CMakeLists），选择CMakeLists.txt文件，打开，Visual studio 会自动加载此仓库，解析 `CMakeLists.txt` 文件，并提取其配置和变量信息。解析完成（需要等会儿或者重启）会从`解决方案资源管理器`中看到`.cpp`文件。

2. 修改配置：

点击`x64-Debug`下拉菜单中的`管理配置`，并在弹出的界面点击`编辑JSON`（该过程在工程根目录下生成`CMakeSettings.json`），将所需要构建的类型改为想编译的类型，比如`Client`。

3. 生成：

点击菜单栏`生成`-`全部生成`或`部分生成`即可。


## 导入崩溃问题

1. 例如，导入链接：
```text
https://overpass-api.de/api/map?bbox=2.08191,41.49611,2.08268,41.49671
小地图：
https://overpass-api.de/api/map?bbox=112.9088,28.2221,112.9172,28.2290
大地图：
https://overpass-api.de/api/map?bbox=112.8971,28.1976,112.9607,28.2501
```

2. `CarlaUE4->Plugins->CarlaTools->Source->CarlaTools->Private`的`OpenDriveToMap.cpp`463行`GenerateTileStandalone()` ，然后调用`ExecuteTileCommandlet()` : 

UCommandlet是 Unreal Engine 中用于实现命令行操作的基类。它允许开发者创建自定义的命令行命令，并在引擎启动时执行这些命令。这个类主要用于扩展引擎功能，提供自定义的命令行工具。

3. ExecuteTileCommandlet() 调用的是`carla\Unreal\CarlaUE4\Plugins\CarlaTools\Intermediate\Build\Win64\UE4Editor\Inc\CarlaTools\OpenDriveToMap.gen.cpp`（反射文件gen）194行的
```shell
static FName NAME_UOpenDriveToMap_ExecuteTileCommandlet = FName(TEXT("ExecuteTileCommandlet"));
void UOpenDriveToMap::ExecuteTileCommandlet()
{
	ProcessEvent(FindFunctionChecked(NAME_UOpenDriveToMap_ExecuteTileCommandlet),NULL);
}
```
* 找到名为“ExecuteTileCommandlet”的`UFunction`对象；
* 执行“ProcessEvent”函数，然后从全局函数映射表找到对应的函数指针执行（利用函数名调用UFUNCTION函数）。


[蓝图的字节码](https://neil3d.github.io/unreal/bp_in_depth.html) ：在`engine\Engine\Source\Runtime\CoreUObject\Public\UObject\Script.h`这个文件中有一个`enum EExprToken`，这个枚举就是蓝图的字节码定义。引擎中使用一个全局查找表，把上述字节码映射到函数指针。在运行时，从一个字节码数组中逐个取出字节码，并查找函数指针，进行调用，也就完成了所谓的“字节码解释执行”的过程。


改为
```shell
void UOpenDriveToMap::GenerateTileStandalone(){
  UE_LOG(LogCarlaToolsMapGenerator, Log, TEXT("UOpenDriveToMap::GenerateTileStandalone Function called"));

  // ExecuteTileCommandlet();
  GenerateTile();

  UEditorLoadingAndSavingUtils::SaveDirtyPackages(true, true);
  UEditorLevelLibrary::SaveCurrentLevel();

}
```
注意：大地图也只能生成一小片地图。


## [VS 调试 UE 项目](https://ue5wiki.com/wiki/14373/)

* 变量已被优化掉，因而不可用。	

VS菜单中选择`DebugGammingEditor`，然后开始调试，就可以添加要监视的项。


## 问题
崩溃于`engine\Engine\Source\Runtime\CoreUObject\Private\UObject\ScriptCore.cpp`1990行`Function->Invoke(this, NewStack, ReturnValueAddress);`
弹出报错信息框：
```text
Failed to open descriptor file ./../../../carla/Unreal/CarlaUE4/CarlaUE4.uproject
```

* [VS跳转代码太慢](http://www.piaoyi.org/c-sharp/Visual-Studio-2019-Intellisense.html) 


## 依赖

### boost-1.80.0
[Python封装C++库调试](https://blog.csdn.net/ASCE_Python/article/details/105595218) 。


## 发布
包含所有软件依赖，双击`launch.bat`启动软件。