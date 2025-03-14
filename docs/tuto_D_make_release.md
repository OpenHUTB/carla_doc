# 如何发布

> _本文档适用于想要发布新版本的开发人员。_

!!! 笔记
    已发布的服务：[Carla C++ 文档](openhutb.github.io/carla_cpp/)、[神经网络原理](openhutb.github.io/neuro/) 、[车辆信息可视化内网访问](http://172.21.108.56:8080/) 、[场景像素流](http://172.21.108.56) 、[DeepSeek大模型内网访问](http://172.21.108.56:3000/) （或 [公网访问](software/llm.md) ） 、[Jenkins 8080](http://172.21.108.56:8080/) 。

1. **确保内容是最新的。**<br>
   请参阅 [升级内容](tuto_D_contribute_assets.md)。

2. **必要时增加 Carla 版本。**<br>
   增加以下文件中的版本： _DefaultGame.ini_, _Carla.uplugin_,
   _setup.py_, _ContentVersions.txt_ 。Grep 查看当前版本，以确保您不会错过任何参考。

3. **清除 CHANGELOG.md。**<br>
   确保变更日志是最新的，必要时重写并重新组织；考虑哪些项目对用户来说更重要。

4. **提交更改并添加新标签。**<br>
   提交所有更改后，添加一个新标签 `git tag -a X.X.X`
   （用最新版本替换 `X.X.X` ）。 添加此版本的变更日志作为标签消息。

5. **标签内容仓库。**<br>
   在确切的提交处将类似的标签添加到内容存储库，如 _ContentVersions.txt_ 中。

6. **推送更改。**<br>
   将所有更改推送到两个存储库，以推送您可能需要使用
   `git push --tags` 的标签。如有必要，创建拉取请求。

7. **编辑 GitHub 版本。**<br>
   转到 [GitHub 发布](https://github.com/carla-simulator/carla/releases) 并在新创建的标签之上创建一个新版本。等到 Jenkins 完成发布最新版本的构建，并将下载链接添加到新创建的版本。

## 发布像素流

* [像素流](ue/pixel_streaming.md)

## [发布可执行游戏](https://blog.csdn.net/zl834205311/article/details/125905705) 
* 安装`pyinstaller`：
```shell
pip install pyinstaller
```
或安装图形窗口打包工具：
```shell
pip install auto-py-to-exe
```
然后运行`auto-py-to-exe`。

## 设计
参考[BeamNG](https://baike.baidu.com/item/BeamNG%E8%B5%9B%E8%BD%A6/18880401)

## 发布到商城
[讨论是否可以发布修改代码的插件](https://www.reddit.com/r/unrealengine/comments/w7ndj9/are_plugins_able_to_append_tooverwrite_engine/)

Baidunet Disk Download Link: https://pan.baidu.com/s/1hFngmeAvyUulsiI-JlNHuA?pwd=hutb

A simulator and scene editor for autonomous driving.

## 问题
点击打包后的文件出现错误：
```text
TypeError: expected str, bytes or os.PathLike object, not _io.BytesIO
```
解决：
```python
pygame.font.Font(pygame.font.get_default_font(), 20)
```
获得的字体文件为`freesansbold.ttf`，最后将字体文件拷贝到和exe文件相同的路径，然后执行即可。

## 参考
* [内网穿透](https://natapp.cn/article/config_ini)
