# 更新 Carla

*   [__更新命令摘要__](#update-commands-summary)  
*   [__获取最新的二进制版本__](#get-latest-binary-release)  
*   [__更新 Linux 和 Windows 构建__](#update-linux-and-windows-build)  
	*   [清除构建](#clean-the-build)  
	*   [从 origin 拉取](#pull-from-origin)  
	*   [下载资产](#download-the-assets)  
	*   [启动服务器](#launch-the-server)  
*   [__获取开发资产__](#get-development-assets)  

要发布意外问题、疑问或建议，请随时登录 Carla 论坛。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="Go to the latest CARLA release">
Carla 论坛</a>
</p>
</div>

---
## 更新命令摘要

<details>
<summary> 显示更新 Carla 的命令行 </summary>

```sh
# 更新 Carla 包的发行版。 
#   1. 删除当前的版本。 
#   2. 按照快速开始安装或一个一个想要的版本。 


# 更新 Linux 构建。 
git checkout master
make clean
git pull origin master
./Update.sh


# 更新 Windows 构建。 
git checkout master
make clean
git pull origin master
#   擦除 `Unreal\CarlaUE4\Content\Carla` 中的内容。
#   转至 `\Util\ContentVersions.txt`.
#   下载最新的资产内容。
#   在 `Unreal\CarlaUE4\Content\Carla` 中解压出新的资产内容。


# 获取开发版本的资产 
#   删除包含之前资产的 `/Carla` 文件夹。
#   转至主 Carla 目录。
git clone https://bitbucket.org/carla-simulator/carla-content Unreal/CarlaUE4/Content/Carla

```
</details>

---
## 获取最新的二进制版本

二进制版本已预先打包，因此与 Carla 的特定版本相关联。要获取最新版本，请删除之前的版本，然后按照 [快速启动安装](start_quickstart.md) 来获取所需的版本。 

版本列在 Carla 仓库的 __Development__ 中。还有一个高度实验性的 __Nightly build__ ，其中包含 Carla 的最新状态。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/blob/master/Docs/download.md" target="_blank" class="btn btn-neutral" title="Go to the list of CARLA releases">
<span class="icon icon-github"></span> Get releases</a>
</p>

<p>
<a href="http://carla-releases.s3.amazonaws.com/Linux/Dev/CARLA_Latest.tar.gz" target="_blank" class="btn btn-neutral" title="Go to the nightly CARLA build">
<span class="icon fa-cloud-download"></span> Get the nightly build</a>
</p>
</div>

---
## 更新 Linux 和 Windows 构建

更新前请确保位于本地 `master` 分支。然后，将更改 merge 或rebase 到其他分支并解决可能的冲突。

```sh 
git checkout master
```

### 清理构建

转到主 Carla 文件夹并删除先前构建生成的二进制文件和临时文件。
```sh 
make clean
```

### 从 origin 拉取

从 Carla 存储库中 `master` 分支获取当前版本。 
```sh
git pull origin master
```

### 下载资产

__Linux.__
```sh
./Update.sh
```

__Windows.__  

__1.__ 删除 `Unreal\CarlaUE4\Content\Carla` 中之前的内容。  
__2.__ 转至 `\Util\ContentVersions.txt`。  
__3.__ 下载最新的 `latest` 的内容。  
__4.__ 在 `Unreal\CarlaUE4\Content\Carla` 抽取新的内容。

!!! 笔记
    为了与 Carla 团队正在开发的项目合作，请 __获取下面的开发资产__ 。

### 启动服务器

在旁观者视图中运行服务器以确保一切正常。

```sh
make launch
```

---
## 获取开发版的资产

Carla 团队使用仍在开发中的资产。这些模型和地图有一个 [公共 git 存储库][contentrepolink]，Carla 团队定期在其中推送最新更新。资产尚未完成，仅建议开发人员使用它们。

为了处理这个存储库，建议安装 [git-lfs][gitlfslink]。存储库会定期修改，并且 git-lfs 处理大型二进制文件的速度更快。

要克隆存储库，请 __转到主 Carla 目录__ 并运行以下命令。

```sh
git clone https://bitbucket.org/carla-simulator/carla-content Unreal/CarlaUE4/Content/Carla
```

!!! 警告
    在克隆存储库之前删除包含资产的 `/Carla` 文件夹。否则，将显示错误。

[contentrepolink]: https://bitbucket.org/carla-simulator/carla-content
[gitlfslink]: https://github.com/git-lfs/git-lfs/wiki/Installation
