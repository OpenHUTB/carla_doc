

## 步骤
在**内容浏览器**中，
将`Unreal\CarlaUE4\Content\Carla\Static\Car\4Wheeled\Tesla`复制到当前文件夹并重命名为`BYD`，
将`Unreal\CarlaUE4\Content\Carla\Blueprints\Vehicles\Tesla`复制到当前文件夹并重命名为`BYD`。

测试代码：
```shell
python manual_control.py --filter vehicle.BYD.seal
python manual_control.py --filter vehicle.tesla.model3
```

如果车辆配置不对，生成玩家的代码一致返回为空：
```python
self.player = self.world.try_spawn_actor(blueprint, spawn_point)
```

3.设置物理资源网格。


## 参考

[比亚迪海豹PK特斯拉Model 3](https://www.dongchedi.com/article/7119344427587813891)
