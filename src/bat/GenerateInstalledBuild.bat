:: 代码页改为 Unicode (UTF-8)，解决命令行运行时中文乱码的问题
chcp 65501

echo off

:: 将当前目录更改为自动化工具的 AutomationTool 文件夹 
cd Engine\Binaries\DotNET

:: AutomationTool.exe是实际的自动化工具可执行文件，它会使用自动化工具和 BuildGraph 生成已安装的构建。
:: BuildGraph 是生成已安装版本本身的命令
:: -target="Make Installed Build Win64" 指定 BuildGraph 将运行的目标。在这种情况下是 Win64，（例如还可能是 Mac）。
:: -script=Engine/Build/InstalledEngineBuild.xml是描述 BuildGraph 将用于生成已安装版本的所有默认参数的文件的路径。
AutomationTool.exe BuildGraph -target="Make Installed Build Win64" -script=Engine/Build/InstalledEngineBuild.xml -set:HostPlatformOnly=true

:: 用一个空行来分割输出
echo.

:: 只是友好地提醒用户流程已经完成。
echo. Install Build Generate Script complete

:: 下面的“pause”可防止在脚本运行完成后立即关闭窗口。
pause