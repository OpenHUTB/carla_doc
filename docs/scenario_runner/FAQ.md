# 经常问的问题

## 在代理导航中收到错误 "TypeError: 'instancemethod' object has no attribute '__getitem__'" 

此问题很可能是由 Python Networkx 包的过时版本引起的。请删除当前安装（例如 sudo apt-get remove python-networkx）并使用“pip install --user networkx==2.2”进行安装。

## 没有可见的场景，我收到消息 "No more scenarios .... Exiting"

如果您收到以下输出
```
Preparing scenario: FollowLeadingVehicle_1
ScenarioManager: Running scenario FollowVehicle
Resetting ego-vehicle!
Failure!
Resetting ego-vehicle!
ERROR: failed to destroy actor 527 : unable to destroy actor: not found
No more scenarios .... Exiting
```
如果您没有看到任何事情发生，很可能是因为您没有启动控制自我车辆的程序。运行例如manual_control.py，您现在应该看到发生了一些事情。 


## 使用 --debug 命令行参数时，Scenario Runner 退出并出现错误

如果您收到以下输出
```
UnicodeEncodeError: 'ascii' codec can't encode character '\u2713' in position 58: ordinal not in range(128)
```
请设置环境变量
```
PYTHONIOENCODING=utf-8
```

