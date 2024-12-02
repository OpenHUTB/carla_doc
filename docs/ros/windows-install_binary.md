[Windows 平台下的ROS二进制安装](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html) 

# 安装依赖
## 安装 Chocolatey

1.根据[链接](https://chocolatey.org/install?_gl=1*1t2vzhg*_ga*MjA0Njg1OTQ3NC4xNzI3MzQ2NDc2*_ga_0WDD29GGN2*MTcyNzM0NjQ3Ni4xLjEuMTcyNzM0NjUxMS4wLjAuMA..) 安装chocolate

使用 `PowerShell` 运行`Get-ExecutionPolicy`，如果返回`Restricted`，然后以管理员身份运行`Set-ExecutionPolicy AllSigned`。
```shell script
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

然后运行以下脚本安装 Chocolatey：
```shell
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

如果没有看到任何错误，输入`choco` 或者 `choco -?`检查安装 Chocolatey 是否成功。

## 安装Python
打开命令行并通过输入以下命令



