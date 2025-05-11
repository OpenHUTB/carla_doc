
# 分析场景打包过程

[Windows.mk](https://github.com/OpenHUTB/carla/blob/3a71d4782a31b747968216e10f7993d0a0d7f453/Util/BuildTools/Windows.mk#L30) ：依赖PythonAPI，执行`Package.bat --ue-version 4.26 --all`
-> Package.bat

1.参数解析

2.获取虚幻引擎根路径

3.设置打包的路径（包括生成压缩包的完整路径 [carla/Build/UE4Carla/CARLA_0.9.15-262-g8f7e40f3c-dirty.zip](https://github.com/OpenHUTB/carla/blob/3a71d4782a31b747968216e10f7993d0a0d7f453/Util/BuildTools/Package.bat#L112) ），它根据`git describe --tags --dirty --always`命令生成压缩包的名称，
```shell
--tags  匹配远程的tag
--dirty  表示如果源码如果被修改了，则会在版本后面加上 -dirty
--always 获取最近的 commitID
```

4.创建Carla包

4.1编辑`Unreal/CarlaUE4/CarlaUE4.uproject`文件来启用/禁用CarSim插件

4.2通过`Build.bat`基于`Development`、`%PACKAGE_CONFIG%`配置进行编译

4.3通过`RunUAT.bat`（Unreal Automation Tool，虚幻自动化工具）烘焙并打包


5.给包中添加额外的文件，包括PythonAPI等

6.压缩项目

7.移除临时的中间文件

8.烘焙其他包


## 运行报错
```text
"C:\workspace\UnrealEngine\Engine\Build\BatchFiles\RunUAT.bat"        BuildCookRun        -nocompileeditor        -TargetPlatform=Win64        -Platform=Win64        -installed        -nop4        -project="C:/workspace/carla/Unreal/CarlaUE4/CarlaUE4.uproject"        -cook        -stage        -build        -archive        -archivedirectory="C:/workspace/carla/Build/UE4Carla/1.0-5-g3a71d4782-dirty/"        -package        -clientconfig=Shipping
```

实际运行命令：
```text
Running: C:\workspace\UnrealEngine\Engine\Binaries\DotNET\UnrealBuildTool.exe UnrealPak Win64 Development -Project=C:\workspace\carla\Unreal\CarlaUE4\CarlaUE4.uproject  C:\workspace\carla\Unreal\CarlaUE4\CarlaUE4.uproject -NoUBTMakefiles  -Manifest=C:\workspace\UnrealEngine\Engine\Intermediate\Build\Manifest.xml -NoHotReload -log="C:\workspace\UnrealEngine\Engine\Programs\AutomationTool\Saved\Logs\UBT-UnrealPak-Win64-Development.txt"
```




## 参考

* 打包脚本 [Package.bat](https://github.com/OpenHUTB/carla/blob/OpenHUTB/Util/BuildTools/Package.bat)
* [UE4如何调试BuildCookRun](https://blog.csdn.net/sinat_23135151/article/details/140663928)
* [编辑器内正常运行但打包出错](https://blog.csdn.net/charon8778/article/details/141339988)

