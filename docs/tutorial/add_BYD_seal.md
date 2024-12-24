

## 步骤
1.在**内容浏览器**中，
将`Unreal\CarlaUE4\Content\Carla\Static\Car\4Wheeled\Tesla`复制到当前文件夹并重命名为`BYD`，
将`Unreal\CarlaUE4\Content\Carla\Blueprints\Vehicles\Tesla`复制到当前文件夹并重命名为`BYD`。

2.添加车辆到蓝图库并测试。

测试代码：
```shell
python manual_control.py --filter vehicle.BYD.seal
python manual_control.py --filter vehicle.tesla.model3
```

如果车辆配置不对，生成玩家的代码一致返回为空：
```python
self.player = self.world.try_spawn_actor(blueprint, spawn_point)
```

CarlaUE4的调用过程：
```text
Server/CarlaServer.cpp
auto Result = Episode->SpawnActorWithInfo(Transform, std::move(Description));

Game/CarlaEpisode.cpp
auto result = ActorDispatcher->SpawnActor(LocalTransform, thisActorDescription, DesiredId);

Actor/ActorDispatcher.cpp
// 在哪调用生成函数导致生成失败
FActorSpawnResult Result = SpawnFunctions[Description.UId - 1](Transform, Description);

// 这里的ActorFactory.SpawnActor函数似乎是ue4里的
void UActorDispatcher::Bind(ACarlaActorFactory &ActorFactory)
{
  for (const auto &Definition : ActorFactory.GetDefinitions())
  {
    Bind(Definition, [&](const FTransform &Transform, const FActorDescription &Description) {
      return ActorFactory.SpawnActor(Transform, Description);
    });
  }
}
```


## 参考

[比亚迪海豹PK特斯拉Model 3](https://www.dongchedi.com/article/7119344427587813891)

[跟踪和解决spawn_actor指令引发的问题：UnknownError while spawning the actor](https://blog.csdn.net/csdncyn/article/details/136691547)
