# 在 Windows 上进行 Carla 的调试

1. 进入目录`carla/Unreal/CarlaUE4/`，右键文件`CarlaUE4.uproject`，选择运行`Generate Visual Studio project files`，在当前目录中将会生成VS的工程文件，双击打开`CarlaUE4.sln`。

![](img/tuto_D_windows_debug/generate_vs_project_files.png)

!!! 笔记
    如果右键菜单中未出现`Generate Visual Studio project files`选项，则到虚幻引擎的目录中运双击执行`engine\Engine\Binaries\Win64\UnrealVersionSelector.exe`，将虚幻引擎软件注册到系统中。

2. 在想要查看的源代码行的最左侧单击增加断点，在菜单运行`调试(D)->开始调试(S)`，程序将在断点出暂停。通过`调式(D)->窗口(W)->监视(W)->监视 1`打开变量监视窗口，查看变量值是否异常。
![](img/tuto_D_windows_debug/debug_project.png)


## 打开 Cmake 工程
windows操作系统下通过vs2019打开并编译carla：
1. 开Carla的CMake项目：

从 VS 的菜单中选择 `File-->Open-->CMake`, 在对话框中找到 Carla 所在的本地文件夹（包含CMakeLists），选择CMakeLists.txt文件，打开，Visual studio 会自动加载此仓库，解析 `CMakeLists.txt` 文件，并提取其配置和变量信息。解析完成（需要等会儿或者重启）会从`解决方案资源管理器`中看到`.cpp`文件。

2. 修改配置
点击`x64-Debug`下拉菜单中的`管理配置`，并在弹出的界面点击`编辑JSON`，

将所需要构建的类型改为想编译的类型，比如`Client`。

3. 生成：
点击菜单栏`生成`-`全部生成`或`部分生成`即可。

## 发布
包含所有软件依赖，双击`launch.bat`启动软件。