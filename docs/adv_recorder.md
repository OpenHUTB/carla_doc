# Recorder

此功能允许记录和重新制定以前的模拟。所有发生的事件都记录在 [recorder file](ref_recorder_binary_file_format.md) 中。有一些高级查询可以跟踪和研究这些事件。

- [__Recording__](#recording)
- [__模拟播放__](#simulation-playback)
     - [设置时间因子](#setting-a-time-factor)
- [__录制文件__](#recorded-file)
- [__查询__](#查询)
    - [碰撞](#collisions)
    - [被困住的角色](#blocked-actors)
- [__示例 Python 脚本__](#sample-python-scripts)

---
## Recording

所有数据仅写入服务器端的二进制文件。但是，使用 [carla.Client](python_api.md#carla.Client) 管理记录器。

根据记录文件中包含的数据，每帧更新角色。当前模拟中出现在录制中的 Actor 将被移动或重新生成以模拟它。那些没有出现在录音中的将继续他们的方式，就好像什么都没发生一样。

！！！重要的
    播放结束时，车辆将设置为自动驾驶，但 __行人将停止__。

记录器文件包括有关许多不同元素的信息。

* __角色__ — 创建和销毁、边界和触发框。
* __交通灯__ — 状态变化和时间设置。
* __车辆__ — 位置和方向、线速度和角速度、光状态和物理控制。
* __行人__ — 位置和方向，以及线速度和角速度。
* __灯光__ — 来自建筑物、街道和车辆的灯光状态。

要开始录制，只需要一个文件名。在文件名中使用 `\`、`/` 或 `:` 字符会将其定义为绝对路径。如果没有详细的路径，文件将保存在 `CarlaUE4/Saved` 中。

```py
client.start_recorder("/home/carla/recording01.log")
```

默认情况下，记录器设置为仅存储回放模拟所需的信息。为了保存前面提到的所有信息，必须在开始录制时配置参数`additional_data`。

```py
client.start_recorder("/home/carla/recording01.log", True)
```

!!! 笔记
    其他数据包括：车辆和行人的线速度和角速度、红绿灯时间设置、执行时间、演员的触发器和边界框，以及车辆的物理控制。

要停止录音，调用也很简单。

```py
client.stop_recorder()
```

!!! 笔记
    据估计，50 个红绿灯和 100 辆车的 1 小时记录大约需要 200MB 大小。

---
##模拟播放

可以在模拟过程中的任何时候开始播放。除了日志文件的路径，这个方法还需要一些参数。

```py
client.replay_file("recording01.log", start, duration, camera)
```

| 参数     | 说明              | 笔记                                     |
|--------|-----------------|----------------------------------------|
| `开始`   | 以秒为单位记录开始模拟的时间。 | 如果是肯定的，时间将从记录开始计算。 <br> 如果是否定的，将从最后考虑。 |
| `持续时间` | 播放秒数。 0 是所有的录音。 | 播放结束时，车辆将设置为自动驾驶，行人将停止。                |
| `相机`   | 相机将聚焦的演员的 ID。   | 将其设置为“0”以让观众自由移动。                      |

<br>



### 设置时间因子

时间因素将决定播放速度。它可以随时更改而无需停止回放。

```py
client.set_replayer_time_factor(2.0)
```

|参数 |默认 |快动作 |慢动作 |
| ------------- | ------------- | ------------- | ------------- |
| `time_factor` | **1\.0** | **\>1.0** | ** <1.0 ** |

<br>



!!! 重要的
    如果 `time_factor>2.0`，actors 的位置插值被禁用并且只是更新。行人的动画不受时间因素的影响。

当时间因素在 __20x__ 左右时，很容易理解流量。

![流](img/RecorderFlow2.gif)

---
##录制文件

可以使用简单的 API 调用来检索记录的详细信息。默认情况下，它只检索注册事件的那些帧。设置参数 `show_all` 将返回每一帧的所有信息。关于如何存储数据的细节在 [recorder's reference](ref_recorder_binary_file_format.md) 中有详细说明。

```py
# 显示相关帧的信息
print (client.show_recorder_file_info ("recording01.log"))
```

* __开放信息.__ 记录模拟的地图、日期和时间。

* __Frame information.__ 任何可能发生的事件，例如角色生成或碰撞。它包含演员的 ID 和一些附加信息。

* __关闭信息.__ 记录的帧数和总时间。

```
Version: 1 
Map: Town05 
Date: 02/21/19 10:46:20 

Frame 1 at 0 seconds 
    Create 2190: spectator (0) at (-260, -200, 382.001) 
    Create 2191: traffic.traffic_light (3) at (4255, 10020, 0) 
    Create 2192: traffic.traffic_light (3) at (4025, 7860, 0) 
    ... 
    Create 2258: traffic.speed_limit.90 (0) at (21651.7, -1347.59, 15) 
    Create 2259: traffic.speed_limit.90 (0) at (5357, 21457.1, 15) 

Frame 2 at 0.0254253 seconds 
    Create 2276: vehicle.mini.cooperst (1) at (4347.63, -8409.51, 120) 
        number_of_wheels = 4 
        object_type = color = 255,241,0 
        role_name = autopilot 
        ... 
Frame 2350 at 60.2805 seconds 
    Destroy 2276 

Frame 2351 at 60.3057 seconds 
    Destroy 2277 
... 
Frames: 2354 
    Duration: 60.3753 seconds
```

---
## 查询

### 碰撞

车辆必须有一个 [碰撞检测器](ref_sensors.md#collision-detector) 以记录碰撞。这些可以被查询，使用参数来过滤碰撞中涉及的参与者的类型。例如，`h`标识`role_name = hero`的角色，通常分配给用户管理的车辆。有一组特定的参与者类型可用于查询。

* __h__ = Hero
* __v__ = Vehicle
* __w__ = Walker
* __t__ = Traffic light
* __o__ = Other
* __a__ = Any

!!! 笔记
    `manual_control.py` 脚本为 自我车辆分配 `role_name = hero`。

碰撞查询需要两个标志来过滤碰撞。以下示例将显示车辆与任何其他对象之间的碰撞。

```py
print(client.show_recorder_collisions("recording01.log","v", "a"))
```

输出总结了碰撞的时间，以及所涉及的参与者的类型、ID 和描述。

```
Version: 1
Map: Town05
Date: 02/19/19 15:36:08

    Time  Types     Id Actor 1                                 Id Actor 2
      16   v v     122 vehicle.yamaha.yzf                     118 vehicle.dodge_charger.police
      27   v o     122 vehicle.yamaha.yzf                       0

Frames: 790
Duration: 46 seconds
```

!!! 重要的
    因为记录碰撞的是`Hero`或`Ego`车辆，所以这将始终是`Actor 1`。


可以通过使用记录器并在事件发生前几秒设置它来重新制定碰撞。

```py
client.replay_file("col2.log", 13, 0, 122)
```
在这种情况下，回放显示了这一点。

![碰撞](img/collision1.gif)

### 被卡住的角色

检测在录制过程中卡住的车辆。如果角色在特定时间内没有移动最小距离，则认为它被阻塞。该定义由用户在查询期间进行。

```py
print(client.show_recorder_actors_blocked("recording01.log", min_time, min_distance))
```

|参数 |说明 |默认 |
| -------------------------------------------------- ------ | -------------------------------------------------- ------ | -------------------------------------------------- ------ |
| `min_time` |移动的最小秒数\`min\_distance\`。 | 30 秒。 |
| `min_distance` |移动的最小厘米数不会被视为阻塞。 | 10厘米。 |



---

!!! 笔记
    有时车辆在红绿灯处停留的时间比预期的要长。

以下示例考虑车辆在 60 秒内移动不到 1 米时被卡住。

```py
client.show_recorder_actors_blocked("col3.log", 60, 100)
```

输出已排序
由 __duration__ 表示，它说明了停止被“阻塞”并移动 `min_distance` 需要多长时间。

```
Version: 1
Map: Town05
Date: 02/19/19 15:45:01

    Time     Id Actor                                 Duration
      36    173 vehicle.nissan.patrol                      336
      75    214 vehicle.chevrolet.impala                   295
     302    143 vehicle.bmw.grandtourer                     67

Frames: 6985
Duration: 374 seconds
```

车辆`173`在`36`秒时停止`336`秒。在第二个`36`之前几秒钟重新模拟以检查它。

```py
client.replay_file("col3.log", 34, 0, 173)
```

![事故](img/accident.gif)

---
## 示例 python 脚本

`PythonAPI/examples` 中提供的一些脚本有助于记录器的使用。



* __start_recording.py__ 开始录制。可以设置录制的持续时间，并且可以在开始时生成演员。

|参数 |说明 |
| -------------------------------------------------- ------------ | -------------------------------------------------- ------------ |
| `-f` |文件名。 |
| `-n`<small>（可选）</small> |产生的车辆。默认值为 10。
| `-t`<small>（可选）</small> |录制的持续时间。 |



* __start_replaying.py__ 开始播放录音。可以设置开始时间、持续时间和要跟随的演员。


|参数 |说明 |
| ------------------------------ | ------------------------------ |
| `-f` |文件名。 |
| `-s`<small>（可选）</small> |起始时间。默认值为 10。
| `-d`<small> (可选)</small> |期间。默认为全部。 |
| `-c`<small>（可选）</small> |要关注的演员的 ID。 |




* __show_recorder_file_info.py__ 显示录制文件中的所有信息。默认情况下，它仅显示记录事件的帧。但是，所有这些都可以显示。


|参数 |说明 |
| -------------------------------------------------- ------------ | -------------------------------------------------- ------------ |
| `-f` |文件名。 |
| `-s`<small>（可选）</small> |标记以显示所有详细信息。 |



* __show_recorder_collisions.py__ 显示类型为 __A__ 和 __B__ 的演员的两个标志之间记录的碰撞。 `-t = vv` 将显示车辆之间的所有碰撞。


| 参数   | 说明                                                                                         |
|------|--------------------------------------------------------------------------------------------|
| `-f` | 文件名。                                                                                       |
| `-t` | 相关角色的标志。 `h` = hero  `v` = vehicle  `w` = walker  `t` = traffic light `o` = other`a` = any |



* __show_recorder_actors_blocked.py__ 列出被认为被阻止的车辆。如果在特定时间内没有移动最小距离，则认为 Actors 被阻挡。


|参数 |说明 |
| -------------------------------------------------- | -------------------------------------------------- |
| `-f` |文件名。 |
| `-t`<small>（可选）</small> |是时候在被认为被阻塞之前移动`-d`了。 |
| `-d`<small> (可选)</small> |移动距离不会被视为阻塞。 |





---
现在是试验一段时间的时候了。使用记录器回放模拟、追溯事件、进行更改以查看新结果。在 CARLA 论坛上就此事发表意见。

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/discussions/" target="_blank" class="btn btn-neutral" title="前往 CARLA 论坛">
CARLA论坛</a>
</p>
</div>