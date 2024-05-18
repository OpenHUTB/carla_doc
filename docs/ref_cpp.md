# [C++ 参考](https://carla.readthedocs.io/en/latest/ref_cpp/) 
我们使用 Doxygen 生成 C++ 代码的文档：

[Libcarla/source](http://carla.org/Doxygen/html/dir_b9166249188ce33115fd7d5eed1849f2.html) <br>
[Unreal/CarlaUE4/Source](http://carla.org/Doxygen/html/dir_733e9da672a36443d0957f83d26e7dbf.html) <br>
[Unreal/CarlaUE4/Plugins](http://carla.org/Doxygen/html/dir_8fc34afb5f07a67966c78bf5319f94ae.html)

生成的文档可通过此 [链接](http://carla.org/Doxygen/html/index.html) 获取。

!!! 笔记
    文档更新由 GitHub 自动完成。

### 创建 doxygen 文档

!!! 重要
    需要 [Doxygen](http://www.doxygen.nl/index.html) 来生成文档，需要 [Graphviz](https://www.graphviz.org/) 作为图形绘制工具包。 

1- 使用以下命令安装 doxygen 和 graphviz：

```sh
# linux
sudo apt-get install doxygen graphviz
```

2- 安装后，转到 _Doxyfile_ 文件所在的项目根文件夹并运行以下命令：

```sh
doxygen
```

它将开始构建文档网页。
生成的网页可以在 Doxygen/html/ 中找到

3- 在浏览器中打开 _index.html_ 。现在您已经有了本地 cpp 文档！
