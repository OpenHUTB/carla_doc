# Windows 下的启动包的构建

## 发布安装版本
参考[链接](https://github.com/chiefGui/ue-from-source?tab=readme-ov-file#step-by-step-1) 进行虚幻引擎的发布。

1. 使用 Visual Studio 打开 `UE4.shn` 。
2. 在右侧边栏，您应该会看到一个`解决方案资源管理器`面板。展开`Programs`文件夹并找到`AutomationTool`项目（`Engine\Source\Programs\AutomationTool`）：
![Image](Engine/Documentation/fig/AutomationTool.png)
3. 右键单击它并选择`生成(Build)`，应该很快。

### 运行安装软件的构建脚本
1. 安装 [Windows 10 SDK](https://developer.microsoft.com/en-us/windows/downloads/windows-10-sdk) ;

2. 运行根目录下的安装构建脚本`GenerateInstalledBuild.bat`。

如果一切顺利，您应该会看到`LocalBuilds`与该文件夹处于同一级别的`Engine`文件夹，并且控制台中没有错误。(还包括一个 InstalledDDC 文件夹：DerivedDataCache)。


### 问题
```text
ERROR: Visual Studio 2017 must be installed in order to build this target.
```
解决：下载`3d5_VisualStudio20171509.rar`进行安装。


```text
Unable to find installation of PDBCOPY.EXE
```
解决：参考 [链接](https://arenas0.com/2018/12/03/UE4_Learn_Build_Binary/) 从 [百度网盘](https://pan.baidu.com/s/1Y0PQeHCMQh7Ln12d_p_Rzw) 下载`X64 Debuggers And Tools-x64_en-us.msi`安装。

```text
无法启动此程序，因为计算机中丢失XINPUT1_3.dll。尝试重新安装该程
```
解决：参考 [链接](http://www.codefaq.cn/category/Windows/) 安装 DirectX Redist (June 2010)。



单独运行`Engine->Binaries->Win64`会出现错误提示：
```text
Plugin 'ExampleDeviceProfileSelector' failed to load because moduleExampleDeviceProfileSelector' could not be found. Please ensure the plugin isproperly installed, otherwise consider disabling the plugin for this project.
```
但是`make launch`不会出错。


