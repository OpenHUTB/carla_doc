
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

4.3通过 [`RunUAT.bat`](https://github.com/OpenHUTB/UnrealEngine/blob/OpenHUTB/Engine/Build/BatchFiles/RunUAT.bat) （Unreal Automation Tool，虚幻自动化工具）烘焙并打包 （报错地方）


5.给包中添加额外的文件，包括PythonAPI等

6.压缩项目

7.移除临时的中间文件

8.烘焙其他包

## RunUAT.bat [打包命令解析](https://blog.csdn.net/ttod/article/details/135868749)
查看`构建烘焙运行`命令`BuildCookRun`的帮助信息：
```shell
%UE4_ROOT%\Engine\Build\BatchFiles\RunUAT.bat BuildCookRun -Help
```
所使用到的命令含义：

| 命令                                                        | 描述                  |
|-----------------------------------------------------------|---------------------|
| `-Platform=Win64`                                         | 目标平台为Windows        |
| `-nop4`                                                   | 不使用Perforce版本控制     |
| `-project="%ROOT_PATH%Unreal/CarlaUE4/CarlaUE4.uproject"` | 目标UE项目工程文件          |
| `-build -cook -stage -archive -package`                           | 表示需要 构建、烘焙、暂存、归档、打包 |
| `-archivedirectory="!BUILD_FOLDER!"`                      | 归档输出目录              |
| `-clientconfig=%PACKAGE_CONFIG%`                      | 客户端编译配置，如Debug/Development/Shipping等              |




## 自动化工具
自动化工具（Unreal Automation Tool，简称 UAT），自动化工具使用特定的命令 BuildCookRun

封装流程包含

| 命令                  | 描述               |
|---------------------|------------------|
| 构建（build）           | 该阶段将为所选择的平台编译可执行文件     |
| 烘培（cook）            | 该阶段通过在特殊模式下执行编辑器来转化内容     |
| 暂存（stage）           | 该阶段将可执行文件和内容复制到暂存区，它是开发目录以外的独立目录     |
| 打包（package、archive） | 该该阶段将项目打包成平台原生的分发格式     |
| 部署（deploy）          | 该阶段将构建版本部署到目标设备     |
| 运行（run）             | 该阶段在目标平台上启动已封装的项目     |




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

