

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

方法`try_spawn_actor()`是从`carla/PythonAPI/carla/source/libcarla/World.cpp` 的`.def("try_spawn_actor", SPAWN_ACTOR_WITHOUT_GIL(TrySpawnActor))` 传递给 C++ 端的。

LibCarla的调用过程：
```text
LibCarla\source\carla\client\World.cpp
SharedPtr<Actor> World::TrySpawnActor()
SharedPtr<Actor> World::SpawnActor()

LibCarla\source\carla\client\detail\Simulator.cpp
SharedPtr<Actor> Simulator::SpawnActor()
actor = _client.SpawnActor()

LibCarla\source\carla\client\detail\Client.cpp
rpc::Actor Client::SpawnActor()
_pimpl->CallAndWait<rpc::Actor>("spawn_actor", description, transform);
auto CallAndWait(const std::string &function, Args && ... args)
auto object = RawCall(function, std::forward<Args>(args) ...);
auto RawCall(const std::string &function, Args && ... args)
return rpc_client.call(function, std::forward<Args>(args) ...);

LibCarla\source\carla\rpc\Client.h
auto call(const std::string &function, Args &&... args)
// 远程调用（_client里保存了IP地址和端口号）
return _client.call(function, Metadata::MakeSync(), std::forward<Args>(args)...);

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
