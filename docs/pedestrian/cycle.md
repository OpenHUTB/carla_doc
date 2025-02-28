# [自行车](https://github.com/tensorturtle/Metacycle)
这是一款开源自行车模拟器。它与智能室内自行车配件配合使用，可提供最高程度的游戏内控制和反馈。它是一款严肃的模拟器，而不是街机游戏，具有逼真的物理效果、精致的图形以及可深度修改的地图和环境。

## 要求
1.自行车

2.智能转向板

3.智能Trainer

4.运行 Windows 10+ 或 Ubuntu 22.04+ 的游戏电脑

目前，市场上唯一的智能转向板是 [Elite Sterzo](https://www.elite-it.com/en/products/home-trainers/ecosystem-accessories/sterzo-smart) 。任何支持蓝牙 FTMS 的 Elite、JetBlack、Wahoo、Tacx 等智能训练器均可兼容。

与任何制造商均无关联。

## 安装
1.启动Carla
```shell
CarlaUE4.exe
```

2.安装UV
```shell
# Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
# 验证安装：
uv --version
```
3.下载安装所需要的python版本和metacycle应用：
```shell
uv python install 3.10
uv tool install metacycle --python 3.10
# 启动metacycle
metacycle
# 可以通过传递参数 --map 来切换地图，比如：
metacycle --map Town10HD
```
4.查看已安装和可安装的环境（可选）：
```shell
uv python list
```

## 效果
通过点击界面当中的`Toggle Control Mode()`来切换使用键盘进行控制。
![](../img/pedestrian/cycle.gif)