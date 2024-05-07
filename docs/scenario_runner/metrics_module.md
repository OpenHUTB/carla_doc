# 指标模块

指标模块依靠 [CARLA 记录器](https://carla.readthedocs.io/en/latest/adv_recorder/) 来方便计算和监控任何类型的参数。场景完成后，记录器存储仿真信息。然后，用户可以定义自己的指标，并检查相应的结果，而无需一次又一次地进行仿真。

本节介绍了构成该模块的不同元素的说明、如何创建新指标的分步教程以及查询记录和定义指标的函数的参考。

* [__模块结构__](#structure-of-the-module)  
* [__如何使用指标模块__](#how-to-use-the-metrics-module)  
	* [1. 记录一个场景](#1.-record-a-scenario)  
	* [2. 定义指标](#2.-run-the-metrics-manager)  
	* [3. 运行指标管理器](#3.-run-the-metrics-manager)  
* [__记录查询参考__](#recording-queries-reference)  
	* [通用参与者数据](#generic-actor-data)  
	* [通用仿真数据](#generic-simulation-data)  
	* [参与者加速度](#actor-accelerations)  
	* [参与者角速度](#actor-angular-velocities)  
	* [参与者控制](#actor-controls)  
	* [参与者变换](#actor-transforms)  
	* [参与者速度](#actor-velocities)  
	* [场景灯](#scene-lights)  
	* [红绿灯](#traffic-lights)  
	* [车灯](#vehicle-lights)  

---
## 模块结构

与主 ScenarioRunner 模块类似，Metrics 模块使用主脚本 运行，`metrics_manager.py`其余信息包含在文件夹结构内。这里简单介绍一下主要脚本。

*   __`metrics_manager.py`__ — 模块的主脚本。运行此命令以显示一组指标的结果。该脚本具有常用的`host`和`port`参数，以及更多用于设置要使用的指标和记录的内容。
	*   `host` *(string)* – CARLA 仿真运行时的 IP 地址。默认为`(127.0.0.1)`。
	*   `port` *(int)* – CARLA 仿真运行的 TCP 端口。默认为`2000`和`2001`。 
	*   `metrics` — 要使用的指标的路径。 
	*   `log` — 包含记录的`.log`文件的路径（相对于环境变量`SCENARIO_RUNNER_ROOT`）。
	*   `criteria` *(可选)* — 具有场景条件的 JSON 文件的路径。

塑造模块的其余元素可以在该`srunner/metrics`文件夹中找到。这些文件夹已分为三个子文件夹。

* __`srunner/metrics/data`__ — 存储有关场景的信息。默认情况下，它有六个文件，它们是示例指标的一部分。

* __`srunner/metrics/examples`__ — 包含一些示例指标，以及包含用于创建新指标的基类`BaseMetric`的指标。  
	*   `basic_metric.py` – 包含基类`BaseMetric`。所有的指标都是从它继承的。 
	*   `criteria_filter.py` – 返回包含标准最重要属性的 JSON。
	*   `distance_between_vehicles.py` – 返回两辆车之间的距离。有助于展示如何查询记录。
	*   `distance_to_lane_center.py` – 计算车辆位置与车道中心之间的距离。有助于展示如何访问地图 API 信息。

* __`srunner/metrics/tools`__ — 包含两个允许查询记录的关键脚本。
	*   `metrics_parser.py` – 将记录提供的字符串转换为字典。
	*   `metrics_log.py` – 提供多种功能来查询使用`metrics_parser.py`创建的字典。这些函数是访问场景信息的最简单方法。它们列在本页最后一段的 [参考文献](#recording-queries-reference) 中。  

---
## 如何使用指标模块

### 1. 记录一个场景

指标模块需要记录仿真才能工作。否则，它没有数据来进行指标计算。

使用`record` 参数。添加应该保存信息的路径。该路径必须相对于环境变量`SCENARIO_RUNNER_ROOT`。

```sh
python scenario_runner.py --scenario <scenario_name> --record <path/to/save/the/recorder/file>
```

通过记录场景，将在所需路径中创建两个文件。这些`log`文件`criteria`稍后将用作`metrics_manager.py`的参数。
__1. CARLA 记录__ *(`.log`)* — 包含每帧的仿真数据。要了解更多信息，请阅读 [记录器文档](https://carla.readthedocs.io/en/latest/adv_recorder/) 。 
__2. 指标文件__ *(.json)* — 场景标准解析为字典，并存储在 JSON 文件中。字典的键是条件的名称。这些值是每个条件的属性。

!!! 注意
    仅解析 JSON 可序列化属性，其余属性将被忽略。

 默认情况下，这两个文件均以运行的场景命名。如果场景被命名为`example.py`，文件将为`example.log`和`example.json`。 


### 2. 定义指标

现在是创建将用于场景的所需指标的时候了。可能性是无限的，但为了本教程的目的，将使用`srunner/metrics/examples/distance_between_vehicles.py`中描述的指标作为示例。让我们深入研究一下代码本身。

    class DistanceBetweenVehicles(BasicMetric):
    """
    指标类 DistanceBetweenVehicles
    """

    def _create_metric(self, town_map, log, criteria):
        """
        度量的实现。这是一个示例，展示了如何使用通过日志访问的记录器。
        """

        # 获取两辆车的ID
        ego_id = log.get_ego_vehicle_id()
        adv_id = log.get_actor_ids_with_role_name("scenario")[0]  # Could have also used its type_id

        dist_list = []
        frames_list = []

        # 获得帧的参与者都活着
        start_ego, end_ego = log.get_actor_alive_frames(ego_id)
        start_adv, end_adv = log.get_actor_alive_frames(adv_id)
        start = max(start_ego, start_adv)
        end = min(end_ego, end_adv)

        # 获得两者之间的距离
        for i in range(start, end):

            # 获得变换
            ego_location = log.get_actor_transform(ego_id, i).location
            adv_location = log.get_actor_transform(adv_id, i).location

            # 过滤一些点以获得更好的图形
            if adv_location.z < -10:
                continue

            dist_v = ego_location - adv_location
            dist = math.sqrt(dist_v.x * dist_v.x + dist_v.y * dist_v.y + dist_v.z * dist_v.z)

            dist_list.append(dist)
            frames_list.append(i)

        # 使用 matplotlib 展示结果
        plt.plot(frames_list, dist_list)
        plt.ylabel('Distance [m]')
        plt.xlabel('Frame number')
        plt.title('Distance between the ego vehicle and the adversary over time')
        plt.show()

__2.1. 度量的名称__. 首先，正如前面提到的，所有指标都是该`BasicMetric`类的子级。 

```py
    class DistanceBetweenVehicles(BasicMetric):
```

__2.2. 主方法的名称__. 指标只需要一种方法即可运行，`_create_metric()`方法具有三个参数。

*   __`town_map`__ — 场景发生的[carla.Map()](https://carla.readthedocs.io/en/latest/python_api/#carlamap)实例。
*   __`log`__ — `MetricsLog`类的实例，具有访问记录器字典所需的所有函数。 
*   __`criteria`__ — 带有标准字典的 JSOn。记录场景时提供的文件，稍后提供给`metrics_manager.py`。

```py
def _create_metric(self, town_map, log, criteria):
```

__2.3. 实现指标__。 代码将根据指标本身而有所不同。

示例指标`DistanceBetweenVehicles`计算本车与其跟随的汽车（对手）之间的距离。为此，需要`id`两辆车的配合。这些可以使用`get_ego_vehicle_id()`和`get_actor_ids_with_role_name("scenario")[0]`函数从日志中检索。

```py
# 获取两个车辆的 ID 
ego_id = log.get_ego_vehicle_id()
adv_id = log.get_actor_ids_with_role_name("scenario")[0]  # Could have also used its type_id
```

它们`id`用`get_actor_alive_frames(actor_id)`检索车辆处于活动状态的帧。
```py
# 获取活跃的两个参与者的帧
start_ego, end_ego = log.get_actor_alive_frames(ego_id)
start_adv, end_adv = log.get_actor_alive_frames(adv_id)
start = max(start_ego, start_adv)
end = min(end_ego, end_adv)
```

现在一切准备就绪，可以循环这些帧，获取它们的变换并计算距离。使用`get_actor_transform(actor_id, frame)` 获得变换，

```py
dist_list = []
frames_list = []

...

# 获得两者之间的距离
for i in range(start, end):

    # 获取变换
    ego_location = log.get_actor_transform(ego_id, i).location
    adv_location = log.get_actor_transform(adv_id, i).location

    # 为了获得更好的图片过滤一些点
    if adv_location.z < -10:
        continue

    dist_v = ego_location - adv_location
    dist = math.sqrt(dist_v.x * dist_v.x + dist_v.y * dist_v.y + dist_v.z * dist_v.z)

    dist_list.append(dist)
```
!!! 注意
    对手的垂直情况仅考虑对手正常行驶时的情况。

最后，使用 [matplotlib](https://matplotlib.org/) 定义运行 `metrics_manager.py` 时指标的输出。 

```py
# 使用 matplotlib 展示结果
plt.plot(frames_list, dist_list)
plt.ylabel('Distance [m]')
plt.xlabel('Frame number')
plt.title('Distance between the ego vehicle and the adversary over time')
plt.show()
```

### 3. 运行指标管理器

最后，是时候使用从场景中检索的信息以及已定义的指标来返回一些指标信息。让我们使用迄今为止使用的示例指标以及以其命名的示例日志来运行该模块。 

```sh
python metrics_manager.py --metric srunner/metrics/examples/distance_between_vehicles.py --log srunner/metrics/data/DistanceBetweenVehicles.log
```

!!! 警告
    仿真必须正在运行。否则，模块将无法访问地图API。

这将创建一个新窗口，其中绘制了结果。在输出窗口关闭之前，脚本不会完成。

![metrics_plot](img/metrics_example.jpg)

---
## 记录查询参考

定义指标时，有关场景的所有信息都通过类`MetricsLog`（`_create_metric()`函数中的`log`参数）访问。该类位于`srunner/metrics/tools/metrics_log.py`，该引用是其中包含的函数的后续部分。

### 通用参与者数据

- <a name="get_ego_vehicle_id"></a>__<font color="#7fb800">get_ego_vehicle_id</font>__(<font color="#00a6ed">__self__</font>)  
返回自我车辆的 `id`。
    - __Return —__ int

- <a name="get_actor_ids_with_role_name"></a>__<font color="#7fb800">get_actor_ids_with_role_name</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__role_name__</font>)  
返回与给定`role_name`匹配的参与者`id`列表。
    - __Return —__ list
    - __Parameters__
        - `role_name` (_str_) — 参与者的`role_name`。

- <a name="get_actor_ids_with_type_id"></a>__<font color="#7fb800">get_actor_ids_with_type_id</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__type_id__</font>)  
根据 fnmatch 标准，返回与给定匹配的参与者 `id` 列表。
    - __Return —__ list
    - __Parameters__
        - `type_id` (_str_) — 参与者`type_id`。

- <a name="get_actor_attributes"></a>__<font color="#7fb800">get_actor_attributes</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>)  
返回包含参与者所有属性的字典。
    - __Return —__ dict
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。

- <a name="get_actor_bounding_box"></a>__<font color="#7fb800">get_actor_bounding_box</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>)  
返回指定参与者的边界框。
    - __Return —__ [carla.BoundingBox](https://carla.readthedocs.io/en/latest/python_api/#carlaboundingbox)
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。

- <a name="get_traffic_light_trigger_volume"></a>__<font color="#7fb800">get_traffic_light_trigger_volume</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__traffic_light_id__</font>)  
返回指定交通灯的 trigger volume。
    - __Return —__ [carla.BoundingBox](https://carla.readthedocs.io/en/latest/python_api/#carlaboundingbox)
    - __Parameters__
        - `traffic_light_id` (_int_) — 红绿灯的`id`。

- <a name="get_actor_alive_frames"></a>__<font color="#7fb800">get_actor_alive_frames</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>)  
返回一个元组，其中包含参与者活跃的第一帧和最后一帧。请注意，帧从 1 开始，而不是从 0 开始。
    - __Return —__ tuple
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。


### 通用仿真数据

- <a name="get_collisions"></a>__<font color="#7fb800">get_collisions</font>__(<font color="#00a6ed">__self__</font>,<font color="#00a6ed">__actor_id__</font>)  
返回具有两个键的字典列表。`frame`是碰撞的帧编号， 是`other_id` 参与者在该帧发生碰撞的 id 列表。
    - __Return —__ list
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。

- <a name="get_total_frame_count"></a>__<font color="#7fb800">get_total_frame_count</font>__(<font color="#00a6ed">__self__</font>)  
返回一个 int 值，其中包含仿真持续的帧总数。
    - __Return —__ int

- <a name="get_elapsed_time"></a>__<font color="#7fb800">get_elapsed_time</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>)  
返回一个包含特定帧的已用时间的浮点数。
    - __Return —__ float
    - __Parameters__
        - `frame` (_int_) — 帧号。

- <a name="get_delta_time"></a>__<font color="#7fb800">get_delta_time</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>)  
返回具有特定帧的增量时间的浮点数。R
    - __Return —__ float
    - __Parameters__
        - `frame` (_int_) — 帧号。

- <a name="get_platform_time"></a>__<font color="#7fb800">get_platform_time</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>)  
返回具有特定帧的平台时间的浮点数。
    - __Return —__ float
    - __Parameters__
        - `frame` (_int_) — 帧号。

### 参与者加速度

- <a name="get_actor_acceleration"></a>__<font color="#7fb800">get_actor_acceleration</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处参与者的加速度。如果参与者`id`不存在、参与者没有加速度或者参与者在该帧中不存在，则返回<b>None</b> 。
    - __Return —__ [carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)
    - __Parameters__
        - `actor_id` (_int_) — 参与者`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_all_actor_accelerations"></a>__<font color="#7fb800">get_all_actor_accelerations</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__first_frame__=None</font>, <font color="#00a6ed">__last_frame__=None</font>)  
返回包含参与者在帧间隔内的所有加速度的列表。默认情况下，帧间隔包含所有记录。
    - __Return —__ list(carla.Vector3D)
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `first_frame` (_int_) — 间隔的初始帧。默认情况下，仿真开始。
        - `last_frame` (_int_) — 间隔的最后一帧。默认情况下，仿真结束。

- <a name="get_actor_accelerations_at_frame"></a>__<font color="#7fb800">get_actor_accelerations_at_frame</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>, <font color="#00a6ed">__actor_list__=None</font>)  
返回一个字典，其中键是帧编号，值是给定帧处参与者的 carla.Vector3D。默认情况下，会考虑所有参与者，但如果传递了*actor_list*，则仅检查列表中的参与者。
    - __Return —__ list(carla.Vector3D)
    - __Parameters__
        - `frame` (_int_) — 帧号。
        - `actor_list` (_int_) — 参与者 `id`的列表。

### 参与者角速度

- <a name="get_actor_angular_velocity"></a>__<font color="#7fb800">get_actor_angular_velocity</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处参与者的角速度。如果参与者 id 不存在、参与者没有角速度或者参与者在该帧中不存在，则返回<b>None</b> 。
    - __Return —__ carla.Vector3D
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `frame` (_int_) — 帧的数目。

- <a name="get_all_actor_angular_velocities"></a>__<font color="#7fb800">get_all_actor_angular_velocities</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__first_frame__=None</font>, <font color="#00a6ed">__last_frame__=None</font>)  
返回包含参与者在帧间隔内的所有角速度的列表。默认情况下，帧间隔包含所有记录。
    - __Return —__ list(carla.Vector3D)
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `first_frame` (_int_) — 间隔的初始帧。默认情况下，仿真开始。
        - `last_frame` (_int_) — 间隔的最后一帧。默认情况下，仿真结束。

- <a name="get_actor_angular_velocities_at_frame"></a>__<font color="#7fb800">get_actor_angular_velocities_at_frame</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>, <font color="#00a6ed">__actor_list__=None</font>)  
返回一个字典，其中键是帧编号，值是给定帧处参与者的[carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d) 。默认情况下，会考虑所有参与者，但如果传递`actor_list`，则仅检查列表中的参与者。 
    - __Return —__ list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))
    - __Parameters__
        - `frame` (_int_) — 帧号。
        - `actor_list` (_int_) — 参与者的 ids 列表。

### 参与者控制

- <a name="get_vehicle_control"></a>__<font color="#7fb800">get_vehicle_control</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__vehicle_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处车辆的控制。该 `manual_gear_shift` 属性将始终为 False。
    - __Return —__ [carla.VehicleControl](https://carla.readthedocs.io/en/latest/python_api/#carlavehiclecontrol)
    - __Parameters__
        - `vehicle_id` (_int_) — 车辆的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_vehicle_physics_control"></a>__<font color="#7fb800">get_vehicle_physics_control</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__vehicle_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处车辆的物理控制。
    - __Return —__ [carla.VehiclePhysicsControl](https://carla.readthedocs.io/en/latest/python_api/#carlavehiclephysicscontrol)
    - __Parameters__
        - `vehicle_id` (_int_) — 车辆的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_walker_speed"></a>__<font color="#7fb800">get_walker_speed</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__walker_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处行人的速度。
    - __Return —__ [carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)
    - __Parameters__
        - `walker_id` (_int_) — 行人的`id`。
        - `frame` (_int_) — 帧号。


### 参与者变换

- <a name="get_actor_transform"></a>__<font color="#7fb800">get_actor_transform</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处参与者的变换。如果参与者 id 不存在、参与者没有变换或者参与者在该帧中不存在，则返回<b>None</b> 。
    - __Return —__ [carla.Transform](https://carla.readthedocs.io/en/latest/python_api/#carlatransform)
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_all_actor_transforms"></a>__<font color="#7fb800">get_all_actor_transforms</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__first_frame__=None</font>, <font color="#00a6ed">__last_frame__=None</font>)  
返回一个列表，其中包含参与者在帧间隔内的所有变换。默认情况下，帧间隔包含所有记录。 
    - __Return —__ list([carla.Transform](https://carla.readthedocs.io/en/latest/python_api/#carlatransform))
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `first_frame` (_int_) — 间隔的初始帧。默认情况下，仿真开始。
        - `last_frame` (_int_) — 间隔的最后一帧。默认情况下，仿真结束。

- <a name="get_actor_transforms_at_frame"></a>__<font color="#7fb800">get_actor_transforms_at_frame</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>, <font color="#00a6ed">__actor_list__=None</font>)  
返回一个字典，其中键是帧编号，值是给定帧处参与者的 [carla.Transform](https://carla.readthedocs.io/en/latest/python_api/#carlatransform)。默认情况下，会考虑所有参与者，但如果 `actor_list` 通过，则仅检查列表中的参与者。
    - __Return —__ list([carla.Transform](https://carla.readthedocs.io/en/latest/python_api/#carlatransform))
    - __Parameters__
        - `frame` (_int_) — 帧号。
        - `actor_list` (_int_) — 参与者 `id` 列表。 

### 参与者速度

- <a name="get_actor_velocity"></a>__<font color="#7fb800">get_actor_velocity</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处参与者的速度。如果参与者 id 不存在、参与者没有速度或者参与者在该帧中不存在，则返回<b>None</b> 。 
    - __Return —__ [carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_all_actor_velocities"></a>__<font color="#7fb800">get_all_actor_velocities</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__actor_id__</font>, <font color="#00a6ed">__first_frame__=None</font>, <font color="#00a6ed">__last_frame__=None</font>)  
返回包含参与者在帧间隔内的所有速度的列表。默认情况下，帧间隔包含所有记录。 
    - __Return —__ list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d))
    - __Parameters__
        - `actor_id` (_int_) — 参与者的`id`。
        - `first_frame` (_int_) — 间隔的初始帧。默认情况下，仿真开始。
        - `last_frame` (_int_) — 间隔的最后一帧。默认情况下，仿真结束。

- <a name="get_actor_velocities_at_frame"></a>__<font color="#7fb800">get_actor_velocities_at_frame</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__frame__</font>, <font color="#00a6ed">__actor_list__=None</font>)  
返回一个字典，其中键是帧编号，值是给定帧处参与者的 carla.Vector3D。默认情况下，会考虑所有参与者，但如果传递了*actor_list*，则仅检查列表中的参与者。 
    - __Return —__ list([carla.Vector3D](https://carla.readthedocs.io/en/latest/python_api/#carlavector3d)
    - __Parameters__
        - `frame` (_int_) — 帧号。
        - `actor_list` (_int_) — 参与者`id`列表。 

### 场景灯

- <a name="get_scene_light_state"></a>__<font color="#7fb800">get_scene_light_state</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__light__</font>, <font color="#00a6ed">__vehicle_id__</font>, <font color="#00a6ed">__frame__</font>)  
Returns the state of a scene light for a given frame. The light state group will always be [carla.LightGroup.None](https://carla.readthedocs.io/en/latest/python_api/#carlalightgroup).
    - __Return —__ carla.LightState
    - __Parameters__
        - `light_id` (_int_) — 场景灯的`id`。
        - `frame` (_int_) — 帧号。


### 红绿灯

- <a name="get_traffic_light_state"></a>__<font color="#7fb800">get_traffic_light_state</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__traffic_light_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧的红绿灯状态。
    - __Return —__ [carla.TrafficLightState](https://carla.readthedocs.io/en/latest/python_api/#carlatrafficlightstate).
    - __Parameters__
        - `traffic_light_id` (_int_) — 红绿灯的`id`。
        - `frame` (_int_) — 帧号。

- <a name="is_traffic_light_frozen"></a>__<font color="#7fb800">is_traffic_light_frozen</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__traffic_light_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回交通信号灯是否冻结在给定帧。
    - __Return —__ bool
    - __Parameters__
        - `traffic_light_id` (_int_) — 交通灯的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_traffic_light_elapsed_time"></a>__<font color="#7fb800">get_traffic_light_elapsed_time</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__traffic_light_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处交通灯的经过时间。
    - __Return —__ float
    - __Parameters__
        - `traffic_light_id` (_int_) — 交通灯的`id`。
        - `frame` (_int_) — 帧号。

- <a name="get_traffic_light_state_time"></a>__<font color="#7fb800">get_traffic_light_state_time</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__traffic_light_id__</font>, <font color="#00a6ed">__state__</font>, <font color="#00a6ed">__frame__</font>)  
返回给定帧处交通灯特定状态的最长时间。
    - __Return —__ float
    - __Parameters__
        - `traffic_light_id` (_int_) — `id` of the traffic light.
        - `state` ([carla.TrafficLightState](https://carla.readthedocs.io/en/latest/python_api/#carlatrafficlightstate)) — 参与者的`id`。
        - `frame` (_int_) — 帧号。

### 车灯

- <a name="get_vehicle_lights"></a>__<font color="#7fb800">get_vehicle_lights</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__vehicle_id__</font>, <font color="#00a6ed">__frame__</font>)  
返回一个列表，其中包含给定帧处特定车辆的活动灯。
    - __Return —__ list([carla.VehicleLightState](https://carla.readthedocs.io/en/latest/python_api/#carlavehiclelightstate))
    - __Parameters__
        - `vehicle_id` (_int_) — 车辆的`id`。
        - `frame` (_int_) — 帧号。

- <a name="is_vehicle_light_active"></a>__<font color="#7fb800">is_vehicle_light_active</font>__(<font color="#00a6ed">__self__</font>, <font color="#00a6ed">__light__</font>, <font color="#00a6ed">__vehicle_id__</font>, <font color="#00a6ed">__frame__</font>)  
检查给定车辆灯在特定帧是否处于活动状态。
    - __Return —__ bool
    - __Parameters__
        - `light` ([carla.VehicleLightState](https://carla.readthedocs.io/en/latest/python_api/#carlavehiclelightstate)) — 与车辆进行比较的灯光状态。
        - `vehicle_id` (_int_) — 车辆的`id`。
        - `frame` (_int_) — 帧号。
