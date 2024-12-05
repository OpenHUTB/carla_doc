## Python 安装
1. 打开 [Python 的官方网站](https://www.python.org/downloads/) 。
2. 下载建议版本 **3.7**（或最高 3.10 版本）。请注意：
  - 安装 **3.11 或更高版本** 可能导致导入 CARLA 包时报错。
  - 避免安装过低版本，以确保兼容性。
3. 双击下载的安装程序，按照以下设置完成安装：
  - 勾选 **Add Python to PATH**（将 Python 添加到环境变量）。
  - 点击 **Customize Installation**（自定义安装），保留默认设置，完成安装。
4. 注意： 不推荐使用 Conda 安装 Python，可能导致生成失败或报错，如 `py3` 或 `py` 错误。
5. 安装完成后，打开命令提示符（cmd），执行以下命令安装必要依赖包：
```shell
pip install setuptools
pip install wheel
```
6. 完成安装后，在命令提示符（cmd）中输入 python，出现对应的版本号，代表安装成功。
![](../img/build/python_version.jpg)
