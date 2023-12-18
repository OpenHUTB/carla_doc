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
mkdocs serve
```
然后使用浏览器打开 [http://127.0.0.1:8000](http://127.0.0.1:8000)，查看文档页面能否正常显示。

3. 部署到`github`：
```shell
mkdocs gh-deploy
```
该命令会自动将相应内容推送到项目的`gh-pages`分支上，然后在 `Github` 项目设置中选择好对应 `GitPage` 的分支，然后通过 `https://<user-name>.github.io/<project-name>` 访问即可。