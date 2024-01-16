# Carla 中文文档

[文档主页](https://openhutb.github.io/carla_doc/) 。

## 本地部署并测试
1. 使用`pip`安装`mkdocs`
```shell
pip install mkdocs
```
（可选）安装完成后使用`mkdocs --version`查看是否安装成功。

2. 在命令行中进入`carla_doc`目录下，运行：
```shell
mkdocs build
mkdocs serve
```
然后使用浏览器打开 [http://127.0.0.1:8000](http://127.0.0.1:8000)，查看文档页面能否正常显示。

3. 部署到`github`：
```shell
mkdocs gh-deploy
```
该命令会自动将相应内容推送到项目的`gh-pages`分支上，然后在 `Github` 项目设置中选择好对应 `GitPage` 的分支，目录选择`/(root)`（注意不要是`/(docs)`，然后通过 `https://openhutb.github.io/carla_doc/` 访问即可。

4. 删除页脚（可选）
* 克隆删除页脚的mkdocs仓库（删除`mkdocs/themes/readthedocs
/footer.html`文件中的页脚内容）：
```shell
git clone https://github.com/OpenHUTB/mkdocs.git
```
安装mkdocs包：
```shell
python -m pip install .
```
然后使用新的mkdocs执行步骤1-3。


## 问题
```text
ERROR - Config value: ‘plugins‘. Error: The “redirects“ plugin is not installed
```
[解决](https://blog.csdn.net/LostSpeed/article/details/127192365) ：
```shell
pip install redirects
```



编译出错解决：
```text
Config value: 'markdown_extensions'. Error: Failed loading extension "mdx_gh_links".
```
[手动安装库](https://github.com/mkdocs/mkdocs/issues/1587) ：
```shell
pip install mdx_gh_links
```