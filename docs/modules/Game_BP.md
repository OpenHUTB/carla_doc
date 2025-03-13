# 游戏蓝图

## 观察者 [Spectator](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Game/Spectator.uasset)

* **事件Tick**：Tick 后每帧调用的事件，依次更新相机移动、相机旋转、速度、视场角（FOV）；
* **左Shift**：按下或松开Shift键时所对应的事件；
* **Z**：按下或松开Z键时所对应的事件 -> 按下或松开鼠标右键时所对应的事件，
* **鼠标中键**：按下或松开鼠标中键时所对应的事件；
* 
### **F12**

按下或松开F12键时所对应的事件（保存当前界面的截图到`Unreal\CarlaUE4\Saved\Screenshots\Windows`目录中）；

执行控制台命令，默认分辨率为1920×1080。

