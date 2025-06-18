# <p align=center> 人车模拟  </p>

<!-- Badges (flat): https://github.com/pudding0503/github-badge-collection -->
<p align=center>
    <a href="https://github.com/OpenHUTB/doc/actions">
        <img src="https://raw.githubusercontent.com/OpenHUTB/doc/refs/heads/master/docs/img/badge.svg" alt="Continuous Integration Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc/releases">
        <img src="https://img.shields.io/github/v/release/OpenHUTB/doc" alt="Releases Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc/blob/master/LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc">
        <img src="https://img.shields.io/badge/platform-windows%20%7C%20macos%20%7C%20linux-lightgrey" alt="Supported Platforms Badge">
    </a>
    <a href="https://zenhub.com">
        <img src="https://img.shields.io/badge/Shipping%20faster%20with-ZenHub-blueviolet" alt="ZenHub Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc/graphs/contributors">
        <img src="https://img.shields.io/github/contributors/OpenHUTB/doc" alt="ZenHub Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc">
        <img src="https://camo.githubusercontent.com/a2746db9952e0d9515498da4e1fb68e9101e40d534de5c644e22afa1e997964a/68747470733a2f2f696d672e736869656c64732e696f2f62616467652f4d61726b646f776e2d3030303030303f7374796c653d666c6174266c6f676f3d4d61726b646f776e266c6f676f436f6c6f723d666666666666" alt="ZenHub Badge">
    </a>
    <a href="https://github.com/OpenHUTB/doc">
        <img src="https://img.shields.io/github/forks/OpenHUTB/doc" alt="ZenHub Badge">
    </a>
</p>

该项目是一款能让用户快速测试 [车辆](https://openhutb.github.io/doc/#_4) 和 [行人](https://openhutb.github.io/doc/#_5) 感知、规划、控制算法的模拟器文档。


<center class="half">
<img href="https://openhutb.github.io/doc/pedestrian/tuto_content_chrono_opensim/" src="docs/img/pedestrian/opensim_running.gif" width=190/>
<img href="https://openhutb.github.io/doc/tuto_G_chrono/" src="docs/img/chrono/vechile_turnover.gif" width=250/>
</center>


---

## 文档部署

1. 安装 python 3.11，使用`pip`安装`mkdocs`
```shell
pip install mkdocs -i http://mirrors.aliyun.com/pypi/simple --trusted-host mirrors.aliyun.com
pip install pymdown-extensions  # pymdownx
pip install -r requirements.txt
```
（可选）安装完成后使用`mkdocs --version`查看是否安装成功。

2. 在命令行中进入`doc`目录下，运行：
```shell
mkdocs build
mkdocs serve
```
然后使用浏览器打开 [http://127.0.0.1:8000](http://127.0.0.1:8000)，查看文档页面能否正常显示。

3. 部署到`github`（可选）：
```shell
mkdocs gh-deploy
```
该命令会自动将相应内容推送到项目的`gh-pages`分支上，然后在 `Github` 项目设置中选择好对应 `GitPage` 的分支，目录选择`/(root)`（注意不要是`/(docs)`，然后通过 [`https://openhutb.github.io/doc/`](https://openhutb.github.io/doc/) 访问即可。

使用虚拟环境会出现找不到`git`的错误，请添加`git`的环境变量：
```shell
set path=%path%;C:\Program Files\Git\cmd\
```

4. 删除页脚（可选）
* 克隆删除页脚的mkdocs仓库（删除`mkdocs/themes/readthedocs
/footer.html`文件中的页脚内容）：
```shell
git clone https://github.com/OpenHUTB/mkdocs.git
```
安装mkdocs包：
```shell
cd mkdocs
python -m pip install . -i http://mirrors.aliyun.com/pypi/simple --trusted-host mirrors.aliyun.com
```
然后使用新的mkdocs执行步骤1-3。


5. 文档页面显示支持 [Latex 公式](https://gist.github.com/josemazo/36af7bb9c58b92c684bbd431f6c68ce9) 、[视频播放](https://pypi.org/project/mkdocs-video/)  。


## 软件发布
将源代码、文档、软件等进行发布，具体步骤参考 [链接](publish.md) 。


## 撰写规范

### 命名规则

`adv_*.md (advise_*.md)` : 建议

`build_*.md` : 源代码构建

`tuto_A_*.md (tutorial_asset_*)` : 资产教程

`tuto_D_*.md (tutorial_development_*)` : 开发教程

`tuto_E_*.md (tutorial_example_*)` : 参考示例

`tuto_G_*.md (tutorial_guide_*)` : 指南教程

`tuto_M_*.md (tutorial_map_*)` : 地图教程

- 页面跳转
1. 定义一个锚(id)： `<span id="jump"></span>` 或 `<span id="jump">跳转到的地方</span>`
2. 使用markdown语法：`[点击跳转](#jump)`


### 颜色规范

灰色底的变量名 **<font color="#f8805a">variable</font>**：
```shell
**<font color="#f8805a">variable</font>**
```
函数名 **<font color="#7fb800">method</font>** ：
```shell
**<font color="#7fb800">method</font>**
```
函数参数名 <font color="#00a6ed">**self**</font>：
```shell
<font color="#00a6ed">**self**</font>
```

### 公式

注意：`markdown_extensions:`标签内需要加上`- mdx_math`（python环境需要安装依赖`python-markdown-math`）。

行内公式
```text
\(...\)
```


行间公式
```text
$$
a + b = c
$$
```

### 绘图
常见图的绘制请参考[绘图指南](docs/demo/figure.md) 。




## 常见问题
* 编译文档时报错：`ERROR - Config value: ‘plugins‘. Error: The “redirects“ plugin is not installed`

> [解决](https://blog.csdn.net/LostSpeed/article/details/127192365) ：
> ```shell
> pip install redirects
> ```



* 编译文档时报错：`Config value: 'markdown_extensions'. Error: Failed loading extension "mdx_gh_links".`
> 解决：[手动安装库](https://github.com/mkdocs/mkdocs/issues/1587) ：
> ```shell
> pip install mdx_gh_links
> ```

* 克隆仓库时报错：`fatal: fetch-pack: invalid index-pack output`
> 解决：
> ```shell
> # 设置下载缓存参数
> git config --global http.postBuffer 2G
> # 确认参数是否正确设置
> git config http.postBuffer
> ```
