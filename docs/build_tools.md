## Package.bat

`%UE4_ROOT%\Engine\Build\BatchFiles\Build.bat`传递的参数包括：
* 参数1：游戏名（CarlaUE4Editor、CarlaUE4）
* 参数2：平台名（Win64）
* 参数3：配置名（Development）
* `-WaitMutex` ：是否需要等待其他锁操作执行完（默认是False）
* `-FromMsBuild`：是否把 MSBuild 的 Log 也打印到 UBT 的 log 系统的输出中（默认是false）
* `%ROOT_PATH%Unreal/CarlaUE4/CarlaUE4.uproject`


[自动化工具](https://cloud.tencent.com/developer/article/2066015) `%UE4_ROOT%\Engine\Build\BatchFiles\RunUAT.bat`(Unreal Automation Tool) 传递的参数包括：
* BuildCookRun：构建（Build）：该阶段将为所选择的平台编译可执行文件。烘培（Cook）：该阶段通过在特殊模式下执行编辑器来转化内容。运行（Run）：该阶段在目标平台上启动已封装的项目。
* nocompileeditor：不包括编译好的编辑器？
* -TargetPlatform=Win64 ： 编译成的目标文件
* -Platform=Win64 ：所在地平台
* -installed
* -nop4：在此构建期间不想与 Perforce 交互（相反是：-P4）
* -project="%ROOT_PATH%Unreal/CarlaUE4/CarlaUE4.uproject"
* -cook：要运行烘培步骤
* -stage：将烘培结果保存在暂存目录中
* -build：要运行构建步骤
* -archive：希望将目录中的完整输出归档
* -archivedirectory="!BUILD_FOLDER!"：归档项目的路径
* -package：为目标平台创建一个包（例如Mac上的应用程序文件，Android上的apk或iPhone上的ipa）
* -clientconfig=%PACKAGE_CONFIG%



## 参考
[UnrealBuildTool功能流程解析](https://zhuanlan.zhihu.com/p/438562865) 


