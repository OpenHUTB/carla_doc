#Python API 参考
此参考包含 Python API 的所有详细信息。要查阅特定 Carla 版本的先前参考，请使用右下角的面板更改文档版本。<br>这会将整个文档更改为之前的状态。请记住， <i>最新</i> 版本是 `dev` 分支，可能会显示任何 Carla 打包版本中不可用的功能。<hr>  

## carla.AckermannControllerSettings<a name="carla.AckermannControllerSettings"></a>
管理 Ackermann PID 控制器的设置。

### 实例变量
- <a name="carla.AckermannControllerSettings.speed_kp"></a>**<font color="#f8805a">speed_kp</font>** (_float_)  
速度 PID 控制器的比例项。  
- <a name="carla.AckermannControllerSettings.speed_ki"></a>**<font color="#f8805a">speed_ki</font>** (_float_)  
速度 PID 控制器的积分项。  
- <a name="carla.AckermannControllerSettings.speed_kd"></a>**<font color="#f8805a">speed_kd</font>** (_float_)  
速度 PID 控制器的微分项。  
- <a name="carla.AckermannControllerSettings.accel_kp"></a>**<font color="#f8805a">accel_kp</font>** (_float_)  
加速度 PID 控制器的比例项。  
- <a name="carla.AckermannControllerSettings.accel_ki"></a>**<font color="#f8805a">accel_ki</font>** (_float_)  
加速度 PID 控制器的积分项。  
- <a name="carla.AckermannControllerSettings.accel_kd"></a>**<font color="#f8805a">accel_kd</font>** (_float_)  
加速度 PID 控制器的微分项。  

### 方法
- <a name="carla.AckermannControllerSettings.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**speed_kp**=0.15</font>, <font color="#00a6ed">**speed_ki**=0.0</font>, <font color="#00a6ed">**speed_kd**=0.25</font>, <font color="#00a6ed">**accel_kp**=0.01</font>, <font color="#00a6ed">**accel_ki**=0.0</font>, <font color="#00a6ed">**accel_kd**=0.01</font>)  
    - **Parameters:**
        - `speed_kp` (_float_)  
        - `speed_ki` (_float_)  
        - `speed_kd` (_float_)  
        - `accel_kp` (_float_)  
        - `accel_ki` (_float_)  
        - `accel_kd` (_float_)  

##### Dunder methods
- <a name="carla.AckermannControllerSettings.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.AckermannControllerSettings](#carla.AckermannControllerSettings)</font>)  
- <a name="carla.AckermannControllerSettings.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.AckermannControllerSettings](#carla.AckermannControllerSettings)</font>)  
- <a name="carla.AckermannControllerSettings.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Actor<a name="carla.Actor"></a>
Carla 将参与者定义为在仿真中发挥作用或可以移动的任何物体。其中包括：行人、车辆、传感器和交通标志（将交通灯视为其中的一部分）。参与者在仿真中由 [carla.World](#carla.World) 生成，并且需要创建 [carla.ActorBlueprint](#carla.ActorBlueprint) 。这些蓝图属于 Carla 提供的库，请在 [此处](bp_library.md) 找到有关它们的更多信息。

### 实例变量
- <a name="carla.Actor.attributes"></a>**<font color="#f8805a">attributes</font>** (_dict_)  
包含该参与者所基于蓝图属性的字典。 
- <a name="carla.Actor.id"></a>**<font color="#f8805a">id</font>** (_int_)  
该参与者的标识符。在给定的情节中是唯一的。  
- <a name="carla.Actor.type_id"></a>**<font color="#f8805a">type_id</font>** (_str_)  
该参与者所基于的蓝图的标识符，例如 `vehicle.ford.mustang`。  
- <a name="carla.Actor.is_alive"></a>**<font color="#f8805a">is_alive</font>** (_bool_)  
返回此对象是否是使用此参与者句柄销毁的。  
- <a name="carla.Actor.is_active"></a>**<font color="#f8805a">is_active</font>** (_bool_)  
返回此参与者是否处于活动状态 (True) 或非活动状态 (False)。  
- <a name="carla.Actor.is_dormant"></a>**<font color="#f8805a">is_dormant</font>** (_bool_)  
返回此参与者是否处于休眠状态 (True) 或非休眠状态 (False) - 与 is_active 相反。  
- <a name="carla.Actor.parent"></a>**<font color="#f8805a">parent</font>** (_[carla.Actor](#carla.Actor)_)  
参与者可以附加到他们将跟随的父参与者。这就是所说的参与者。  
- <a name="carla.Actor.semantic_tags"></a>**<font color="#f8805a">semantic_tags</font>** (_list(int)_)  
蓝图列表组件为此参与者提供的语义标签列表。例如，交通灯可以用 `Pole` 和 `TrafficLight`. 。这些标签由语义分割传感器使用。在[此处](ref_sensors.md#semantic-segmentation-camera) 查找有关此传感器和其他传感器的更多信息。  
- <a name="carla.Actor.actor_state"></a>**<font color="#f8805a">actor_state</font>** (_[carla.ActorState](#carla.ActorState)_)  
返回 [carla.ActorState](#carla.ActorState)，它可以识别参与者是否处于活动、休眠或无效状态。  
- <a name="carla.Actor.bounding_box"></a>**<font color="#f8805a">bounding_box</font>** (_[carla.BoundingBox](#carla.BoundingBox)_)  
包含参与者几何形状的边界框。它的位置和旋转是相对于它所附着的参与者的。  

### 方法
- <a name="carla.Actor.add_angular_impulse"></a>**<font color="#7fb800">add_angular_impulse</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**angular_impulse**</font>)  
在角色的质心处应用角冲量。此方法适用于瞬时扭矩，通常应用一次。使用 __<font color="#7fb800">add_torque()</font>__ 在一段时间内施加旋转力。  
    - **参数:**
        - `angular_impulse` (_[carla.Vector3D](#carla.Vector3D)<small> - degrees*s</small>_) - 全局坐标中的角脉冲矢量。  
- <a name="carla.Actor.add_force"></a>**<font color="#7fb800">add_force</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**force**</font>)  
在参与者的质心处施加力。此方法适用于在一定时间内施加的力。使用 __<font color="#7fb800">add_impulse()</font>__ 施加仅持续瞬间的脉冲。  
    - **参数:**
        - `force` (_[carla.Vector3D](#carla.Vector3D)<small> - N</small>_) - 全局坐标中的力矢量。  
- <a name="carla.Actor.add_impulse"></a>**<font color="#7fb800">add_impulse</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**impulse**</font>)  
在参与者的质心处施加脉冲。此方法适用于瞬时力，通常应用一次。使用 __<font color="#7fb800">add_force()</font>__ 在一段时间内施加力。  
    - **参数:**
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> - N*s</small>_) - 全局坐标中的脉冲矢量。 
- <a name="carla.Actor.add_torque"></a>**<font color="#7fb800">add_torque</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**torque**</font>)  
在参与者的质心处应用扭矩。此方法适用于在一定时间内施加的扭矩。使用 __<font color="#7fb800">add_angular_impulse()</font>__ 施加仅持续瞬间的扭矩。  
    - **参数:**
        - `torque` (_[carla.Vector3D](#carla.Vector3D)<small> - degrees</small>_) - 全局坐标中的扭矩矢量。  
- <a name="carla.Actor.destroy"></a>**<font color="#7fb800">destroy</font>**(<font color="#00a6ed">**self**</font>)  
告诉仿真器销毁这个参与者，如果成功则 <b>True</b> 。如果已经被破坏则没有任何效果。
    - **返回:** _bool_  
    - **警告:** <font color="#ED2F2F">_T此方法会阻止脚本，直到仿真器完成销毁为止。
_</font>  
- <a name="carla.Actor.disable_constant_velocity"></a>**<font color="#7fb800">disable_constant_velocity</font>**(<font color="#00a6ed">**self**</font>)  
禁用之前为 [carla.Vehicle](#carla.Vehicle) 参与者设置的任何恒定速度。  
- <a name="carla.Actor.enable_constant_velocity"></a>**<font color="#7fb800">enable_constant_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**velocity**</font>)  
将车辆的速度矢量设置为随时间变化的恒定值。所得到的速度将近似于 `velocity` 所设置的速度，与 __<font color="#7fb800">set_target_velocity()</font>__ 一样。  
    - **参数:**
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)<small> - m/s</small>_) - Velocity vector in local space.  
    - **注意:** <font color="#8E8E8E">_只有 [carla.Vehicle](#carla.Vehicle) 参与者可以使用此方法。  
_</font>  
    - **警告:** <font color="#ED2F2F">_为 交通管理器管理的车辆启用恒定速度可能会导致冲突。此方法会覆盖交通管理器对速度的任何更改。  
_</font>  

##### 获取器
- <a name="carla.Actor.get_acceleration"></a>**<font color="#7fb800">get_acceleration</font>**(<font color="#00a6ed">**self**</font>)  
返回客户端在最后一个tick期间收到的角色的三维加速度向量。该方法不调用仿真器。  
    - **返回:** _[carla.Vector3D](#carla.Vector3D)<small> - m/s<sup>2</sup></small>_  
- <a name="carla.Actor.get_angular_velocity"></a>**<font color="#7fb800">get_angular_velocity</font>**(<font color="#00a6ed">**self**</font>)  
返回客户端在最后一个tick期间接收到的角色的角速度向量。该方法不调用仿真器。  
    - **返回：** _[carla.Vector3D](#carla.Vector3D)<small> - deg/s</small>_  
- <a name="carla.Actor.get_location"></a>**<font color="#7fb800">get_location</font>**(<font color="#00a6ed">**self**</font>)  
返回客户端在上一次tick期间接收到的参与者的位置。该方法不调用仿真器。  
    - **返回：** _[carla.Location](#carla.Location)<small> - meters</small>_  
    - **设置器:** _[carla.Actor.set_location](#carla.Actor.set_location)_  
- <a name="carla.Actor.get_transform"></a>**<font color="#7fb800">get_transform</font>**(<font color="#00a6ed">**self**</font>)  
返回客户端在最后一个tick期间接收到的参与者的变换（位置和旋转）。该方法不调用仿真器。  
    - **返回:** _[carla.Transform](#carla.Transform)_  
    - **设置器:** _[carla.Actor.set_transform](#carla.Actor.set_transform)_  
- <a name="carla.Actor.get_velocity"></a>**<font color="#7fb800">get_velocity</font>**(<font color="#00a6ed">**self**</font>)  
返回客户端在最后一个tick期间接收到的角色的速度向量。该方法不调用仿真器。  
    - **返回：** _[carla.Vector3D](#carla.Vector3D)<small> - m/s</small>_  
- <a name="carla.Actor.get_world"></a>**<font color="#7fb800">get_world</font>**(<font color="#00a6ed">**self**</font>)  
返回该参与者所属的世界。  
    - **返回:** _[carla.World](#carla.World)_  

##### 设置器
- <a name="carla.Actor.set_enable_gravity"></a>**<font color="#7fb800">set_enable_gravity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**</font>)  
启用或禁用演员的重力。 __默认__ 为 True。  
    - **参数：**
        - `enabled` (_bool_)  
- <a name="carla.Actor.set_location"></a>**<font color="#7fb800">set_location</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
将角色传送到给定位置。  
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)<small> - meters</small>_)  
    - **获取器：** _[carla.Actor.get_location](#carla.Actor.get_location)_  
- <a name="carla.Actor.set_simulate_physics"></a>**<font color="#7fb800">set_simulate_physics</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=True</font>)  
启用或禁用此参与者上的物理仿真。  
    - **参与者：**
        - `enabled` (_bool_)  
- <a name="carla.Actor.set_target_angular_velocity"></a>**<font color="#7fb800">set_target_angular_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**angular_velocity**</font>)  
设置角色的角速度矢量。这是在物理步骤之前应用的，因此最终的角速度将受到摩擦等外力的影响。  
    - **参数：**
        - `angular_velocity` (_[carla.Vector3D](#carla.Vector3D)<small> - deg/s</small>_)  
- <a name="carla.Actor.set_target_velocity"></a>**<font color="#7fb800">set_target_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**velocity**</font>)  
设置参与者的速度向量。这是在物理步骤之前应用的，因此最终的角速度将受到摩擦等外力的影响。  
    - **参数：**
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)_)  
- <a name="carla.Actor.set_transform"></a>**<font color="#7fb800">set_transform</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**transform**</font>)  
将角色传送到给定的变换（位置和旋转）。 
    - **参数：**
        - `transform` (_[carla.Transform](#carla.Transform)_)  
    - **获取器：** _[carla.Actor.get_transform](#carla.Actor.get_transform)_  

##### Dunder methods
- <a name="carla.Actor.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.ActorAttribute<a name="carla.ActorAttribute"></a>
Carla 为参与者提供了一个蓝图库，可以通过 [carla.BlueprintLibrary](#carla.BlueprintLibrary) 进行访问。每个蓝图都有一系列内部定义的属性。其中一些是可以修改的，另一些则是不可修改的。为可设置的值提供了推荐值列表。  

### 实例变量
- <a name="carla.ActorAttribute.id"></a>**<font color="#f8805a">id</font>** (_str_)  
库中属性的名称和标识符。 
- <a name="carla.ActorAttribute.is_modifiable"></a>**<font color="#f8805a">is_modifiable</font>** (_bool_)  
如果属性的值可以修改，则为 <b>True</b> 。
- <a name="carla.ActorAttribute.recommended_values"></a>**<font color="#f8805a">recommended_values</font>** (_list(str)_)  
设计蓝图的人员建议的值列表。  
- <a name="carla.ActorAttribute.type"></a>**<font color="#f8805a">type</font>** (_[carla.ActorAttributeType](#carla.ActorAttributeType)_)  
属性的参数类型。  

### 方法
- <a name="carla.ActorAttribute.as_bool"></a>**<font color="#7fb800">as_bool</font>**(<font color="#00a6ed">**self**</font>)  
将属性读取为布尔值。  
- <a name="carla.ActorAttribute.as_color"></a>**<font color="#7fb800">as_color</font>**(<font color="#00a6ed">**self**</font>)  
将属性读取为 [carla.Color](#carla.Color)。 
- <a name="carla.ActorAttribute.as_float"></a>**<font color="#7fb800">as_float</font>**(<font color="#00a6ed">**self**</font>)  
将属性读取为浮点型。  
- <a name="carla.ActorAttribute.as_int"></a>**<font color="#7fb800">as_int</font>**(<font color="#00a6ed">**self**</font>)  
将属性读取为 int。  
- <a name="carla.ActorAttribute.as_str"></a>**<font color="#7fb800">as_str</font>**(<font color="#00a6ed">**self**</font>)  
将属性读取为字符串。  

##### Dunder methods
- <a name="carla.ActorAttribute.__bool__"></a>**<font color="#7fb800">\__bool__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.ActorAttribute.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=bool / int / float / str / [carla.Color](#carla.Color) / [carla.ActorAttribute](#carla.ActorAttribute)</font>)  
如果此参与者的属性 和`other` 相同，则返回 true。  
    - **返回：** _bool_  
- <a name="carla.ActorAttribute.__float__"></a>**<font color="#7fb800">\__float__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.ActorAttribute.__int__"></a>**<font color="#7fb800">\__int__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.ActorAttribute.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=bool / int / float / str / [carla.Color](#carla.Color) / [carla.ActorAttribute](#carla.ActorAttribute)</font>)  
如果此 actor 的属性和 `other` 不同，则返回 true。  
    - **返回：** _bool_  
- <a name="carla.ActorAttribute.__nonzero__"></a>**<font color="#7fb800">\__nonzero__</font>**(<font color="#00a6ed">**self**</font>)  
如果此参与者的属性不为零或 null，则返回 true。  
    - **Return:** _bool_  
- <a name="carla.ActorAttribute.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.ActorAttributeType<a name="carla.ActorAttributeType"></a>
Carla 在 [carla.BlueprintLibrary](#carla.BlueprintLibrary) 中为参与者提供了一个蓝图库，每个蓝图具有不同的属性。此类将 [carla.ActorAttribute](#carla.ActorAttribute) 中的类型定义为一系列枚举。所有这些信息均在内部管理，并在此处列出，以便更好地理解 Carla 的工作原理。 

### 实例变量
- <a name="carla.ActorAttributeType.Bool"></a>**<font color="#f8805a">Bool</font>**  
- <a name="carla.ActorAttributeType.Int"></a>**<font color="#f8805a">Int</font>**  
- <a name="carla.ActorAttributeType.Float"></a>**<font color="#f8805a">Float</font>**  
- <a name="carla.ActorAttributeType.String"></a>**<font color="#f8805a">String</font>**  
- <a name="carla.ActorAttributeType.RGBColor"></a>**<font color="#f8805a">RGBColor</font>**  

---

## carla.ActorBlueprint<a name="carla.ActorBlueprint"></a>
Carla 为 Actor 提供了一个蓝图库，可以通过 [carla.BlueprintLibrary](#carla.BlueprintLibrary)进行查阅。其中每一个都包含蓝图的标识符和一系列可以修改或不可修改的属性。该类是库和参与者创建之间的中间步骤。参与者需要一个参与者蓝图才能产生。这些将所述蓝图的信息及其属性和一些标签存储在对象中以对其进行分类。然后，用户可以自定义一些属性并最终通过 [carla.World](#carla.World)生成参与者。  

### 实例变量
- <a name="carla.ActorBlueprint.id"></a>**<font color="#f8805a">id</font>** (_str_)  
库内所述蓝图的标识符。例如 `walker.pedestrian.0001`。  
- <a name="carla.ActorBlueprint.tags"></a>**<font color="#f8805a">tags</font>** (_list(str)_)  
每个蓝图具有的有助于描述它们的标签列表。例如 `['0001', 'pedestrian', 'walker']`。  

### 方法
- <a name="carla.ActorBlueprint.has_attribute"></a>**<font color="#7fb800">has_attribute</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**id**</font>)  
如果蓝图包含该 `id` 属性，则返回 <b>True</b>。  
    - **参数：**
        - `id` (_str_) - 例如 `gender` 将为行人的蓝图返回 **True** 。  
    - **返回：** _bool_  
- <a name="carla.ActorBlueprint.has_tag"></a>**<font color="#7fb800">has_tag</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**tag**</font>)  
如果蓝图已列出指定`tag`的内容，则返回 <b>True</b> 。  
    - **参数：**
        - `tag` (_str_) - 例如： 'walker'。  
    - **返回：** _bool_  
- <a name="carla.ActorBlueprint.match_tags"></a>**<font color="#7fb800">match_tags</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**wildcard_pattern**</font>)  
如果为此蓝图列出的任何标记匹配 `wildcard_pattern`，则返回 <b>True</b> 。匹配遵循 [fnmatch](https://docs.python.org/2/library/fnmatch.html) 标准。
    - **参数：**
        - `wildcard_pattern` (_str_)  
    - **返回：** _bool_  

##### 获取器
- <a name="carla.ActorBlueprint.get_attribute"></a>**<font color="#7fb800">get_attribute</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**id**</font>)  
返回参与者的属性以及 `id` 标识符（如果存在）。  
    - **参与者：**
        - `id` (_str_)  
    - **返回：** _[carla.ActorAttribute](#carla.ActorAttribute)_  
    - **设置器：** _[carla.ActorBlueprint.set_attribute](#carla.ActorBlueprint.set_attribute)_  

##### 设置器
- <a name="carla.ActorBlueprint.set_attribute"></a>**<font color="#7fb800">set_attribute</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**id**</font>, <font color="#00a6ed">**value**</font>)<button class="SnipetButton" id="carla.ActorBlueprint.set_attribute-snipet_button">snippet &rarr;</button>  
如果 `id` 属性是可修改的，将其值改为 `value`。  
    - **参数：**
        - `id` (_str_) - 要更改属性的标识符。 
        - `value` (_str_) - 所述属性的新值。 
    - **获取器：** _[carla.ActorBlueprint.get_attribute](#carla.ActorBlueprint.get_attribute)_  

##### Dunder 方法
- <a name="carla.ActorBlueprint.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
在蓝图有的 [carla.ActorAttribute](#carla.ActorAttribute) 之上进行迭代。
- <a name="carla.ActorBlueprint.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
返回此蓝图的属性数量。  
- <a name="carla.ActorBlueprint.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.ActorList<a name="carla.ActorList"></a>
一个类，它包含现场出现的每个演员并提供对他们的访问。该列表由服务器自动创建和更新，可以使用 [carla.World](#carla.World) 返回。

### 方法
- <a name="carla.ActorList.filter"></a>**<font color="#7fb800">filter</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**wildcard_pattern**</font>)  
筛选匹配`wildcard_pattern`的参与者列表，以对抗变量 __<font color="#f8805a">type_id</font>__ (它标识用于生成它们的蓝图)。匹配遵循 [fnmatch](https://docs.python.org/2/library/fnmatch.html) 标准。 
    - **参数：**
        - `wildcard_pattern` (_str_)  
    - **返回：** _list_  
- <a name="carla.ActorList.find"></a>**<font color="#7fb800">find</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_id**</font>)  
使用其标识符查找参与者并返回它，如果不存在则返回<b>None</b>。 
    - **参数：**
        - `actor_id` (_int_)  
    - **返回：** _[carla.Actor](#carla.Actor)_  

##### Dunder 方法
- <a name="carla.ActorList.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
返回与列表中`pos`位置相对应的参与者。
    - **返回：** _[carla.Actor](#carla.Actor)_  
- <a name="carla.ActorList.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
在包含 [carla.Actor](#carla.Actor) 的列表上进行迭代。
- <a name="carla.ActorList.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
返回列出的参与者数量。  
    - **返回：** _int_  
- <a name="carla.ActorList.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
在列出的每个参与者上解析ID。
    - **返回：** _str_  

---

## carla.ActorSnapshot<a name="carla.ActorSnapshot"></a>
包含参与者在特定时刻的所有信息的类。这些对象包含在 [carla.WorldSnapshot](#carla.WorldSnapshot) 中，并在每个时钟周期发送到客户端一次。 

### 实例变量
- <a name="carla.ActorSnapshot.id"></a>**<font color="#f8805a">id</font>** (_int_)  
快照本身的标识符。  

### 方法

##### 获取器
- <a name="carla.ActorSnapshot.get_acceleration"></a>**<font color="#7fb800">get_acceleration</font>**(<font color="#00a6ed">**self**</font>)  
返回在该tick中为参与者注册的加速度向量。  
    - **返回：** _[carla.Vector3D](#carla.Vector3D)<small> - m/s<sup>2</sup></small>_  
- <a name="carla.ActorSnapshot.get_angular_velocity"></a>**<font color="#7fb800">get_angular_velocity</font>**(<font color="#00a6ed">**self**</font>)  
返回为该tick中的参与者注册的角速度向量。
    - **返回：** _[carla.Vector3D](#carla.Vector3D)<small> - rad/s</small>_  
- <a name="carla.ActorSnapshot.get_transform"></a>**<font color="#7fb800">get_transform</font>**(<font color="#00a6ed">**self**</font>)  
返回该 tick 中参与者的参与者变换（位置和旋转）。 
    - **返回：** _[carla.Transform](#carla.Transform)_  
- <a name="carla.ActorSnapshot.get_velocity"></a>**<font color="#7fb800">get_velocity</font>**(<font color="#00a6ed">**self**</font>)  
返回在该tick中为参与者注册的速度向量。  
    - **返回：** _[carla.Vector3D](#carla.Vector3D)<small> - m/s</small>_  

---

## carla.ActorState<a name="carla.ActorState"></a>
定义参与者状态的类。  

### 实例变量
- <a name="carla.ActorState.Invalid"></a>**<font color="#f8805a">Invalid</font>**  
如果出现问题，参与者就是无效的。  
- <a name="carla.ActorState.Active"></a>**<font color="#f8805a">Active</font>**  
当一个参与者可视化并且可以影响其他参与者时，它就是活跃的。 
- <a name="carla.ActorState.Dormant"></a>**<font color="#f8805a">Dormant</font>**  
当参与者不被可视化并且不会通过物理影响其他参与者时，参与者就处于休眠状态。例如，如果参与者位于大地图中已卸载的图块上，则参与者处于休眠状态。 

---

## carla.AttachmentType<a name="carla.AttachmentType"></a>
定义参与者与其父级之间的附件选项的类。当生成参与者时，它们可以附加到另一个参与者上，这样它们的位置就会相应改变。这对于传感器特别有用。 [carla.World.spawn_actor](#carla.World.spawn_actor) 中的片段显示了一些传感器在生成时连接到汽车上。请注意，附件类型在类中声明为枚举。 

### 实例变量
- <a name="carla.AttachmentType.Rigid"></a>**<font color="#f8805a">Rigid</font>**  
通过此固定附件，对象严格遵循其父位置。这是推荐的附件，用于从仿真中检索精确数据。  
- <a name="carla.AttachmentType.SpringArm"></a>**<font color="#f8805a">SpringArm</font>**  
一种附件，可根据其父级扩展或缩回参与者的位置。仅建议在需要平滑运动的仿真中录制视频时使用此附件。SpringArms 是一个虚幻引擎组件，因此 [check the UE 文档](https://docs.unrealengine.com/en-US/Gameplay/HowTo/UsingCameras/SpringArmComponents/index.html) 以了解有关它们的更多信息。 <br><b style="color:red;">警告：</b> 当参与者在Z轴上产生相对平移时，<b>SpringArm</b> 附件呈现出奇怪的行为。（比如：<code>child_location = Location(0,0,2)</code>）。 
- <a name="carla.AttachmentType.SpringArmGhost"></a>**<font color="#f8805a">SpringArmGhost</font>**  
一个与前一个类似的附件，但它不会进行碰撞测试，这意味着它不会扩展或收缩参与者的位置。**ghost** 一词是因为这样相机就可以穿过墙壁和其他几何形状。此附件仅建议用于录制需要平滑移动的仿真视频。SpringArms 是一个虚幻引擎组件，请 [查看虚幻引擎文档](https://docs.unrealengine.com/en-US/Gameplay/HowTo/UsingCameras/SpringArmComponents/index.html) 以了解更多信息。<br><b style="color:red;">警告：</b>当参与者在Z轴上产生相对平移时，<b>SpringArm</b>附件会出现奇怪的行为（比如<code>child_location = Location(0,0,2)</code>）。 

---

## carla.BlueprintLibrary<a name="carla.BlueprintLibrary"></a>
包含为参与者生成提供的蓝图的类。它的主要应用是返回生成参与者所需的 [carla.ActorBlueprint](#carla.ActorBlueprint) 对象。每个蓝图都有一个标识符和属性，这些属性可能是可修改的，也可能是不可修改的。该库由服务器自动创建，可以通过 [carla.World](#carla.World) 访问。

  [这](bp_library.md) 是包含每个可用蓝图及其细节的参考。  

### 方法
- <a name="carla.BlueprintLibrary.filter"></a>**<font color="#7fb800">filter</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**wildcard_pattern**</font>)  
根据此库中包含的每个蓝图的id和标记过滤与通配符模式 `wildcard_pattern` 匹配的蓝图列表，并将结果作为新的结果返回。匹配遵循 [fnmatch](https://docs.python.org/2/library/fnmatch.html) 标准。
    - **参数：**
        - `wildcard_pattern` (_str_)  
    - **返回：** _[carla.BlueprintLibrary](#carla.BlueprintLibrary)_  
- <a name="carla.BlueprintLibrary.filter_by_attribute"></a>**<font color="#7fb800">filter_by_attribute</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**name**</font>, <font color="#00a6ed">**value**</font>)  
根据此库中包含的每个蓝图，筛选具有与值 `value` 匹配的给定属性的蓝图列表，并将结果作为新的结果返回。匹配遵循 [fnmatch](https://docs.python.org/2/library/fnmatch.html) 标准。
    - **参数：**
        - `name` (_str_)  
        - `value` (_str_)  
    - **返回：** _[carla.BlueprintLibrary](#carla.BlueprintLibrary)_  
- <a name="carla.BlueprintLibrary.find"></a>**<font color="#7fb800">find</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**id**</font>)  
返回与该标识符对应的蓝图。
    - **参数：**
        - `id` (_str_)  
    - **返回：** _[carla.ActorBlueprint](#carla.ActorBlueprint)_  

##### Dunder 方法
- <a name="carla.BlueprintLibrary.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
返回存储在包含蓝图的数据结构内`pos`位置的蓝图。 
    - **Return:** _[carla.ActorBlueprint](#carla.ActorBlueprint)_  
- <a name="carla.BlueprintLibrary.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
在存储库 [carla.ActorBlueprint](#carla.ActorBlueprint) 上进行迭代。
- <a name="carla.BlueprintLibrary.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
返回构成库的蓝图数量。 
    - **返回：** _int_  
- <a name="carla.BlueprintLibrary.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
解析要字符串化的每个蓝图的标识符。 
    - **返回：** _string_  

---

## carla.BoundingBox<a name="carla.BoundingBox"></a>
边界框包含场景中参与者或元素的几何体。它们可以被[carla.DebugHelper](#carla.DebugHelper) 或 [carla.Client](#carla.Client) 用于绘制它们的形状以进行调试。看看 [carla.DebugHelper.draw_box](#carla.DebugHelper.draw_box)，其中使用世界快照绘制红绿灯的边界框。


### 实例变量
- <a name="carla.BoundingBox.extent"></a>**<font color="#f8805a">extent</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - meters</small>_)  
从长方体中心到一个顶点的矢量。每个轴中的值等于该轴框大小的一半。`extent.x * 2` 将返回框在x轴上的大小。 
- <a name="carla.BoundingBox.location"></a>**<font color="#f8805a">location</font>** (_[carla.Location](#carla.Location)<small> - meters</small>_)  
边界框的中心。  
- <a name="carla.BoundingBox.rotation"></a>**<font color="#f8805a">rotation</font>** (_[carla.Rotation](#carla.Rotation)_)  
边界框的朝向。 

### 方法
- <a name="carla.BoundingBox.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**extent**</font>)  
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)_) - 相对于其父框的中心。 
        - `extent` (_[carla.Vector3D](#carla.Vector3D)<small> - meters</small>_) - 包含每个轴的长方体大小的一半的矢量。 
- <a name="carla.BoundingBox.contains"></a>**<font color="#7fb800">contains</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**world_point**</font>, <font color="#00a6ed">**transform**</font>)  
如果在世界空间中传递的点在此边界框内，则返回**True**。
    - **Parameters:**
        - `world_point` (_[carla.Location](#carla.Location)<small> - meters</small>_) - 要检查的世界空间中的点。 
        - `transform` (_[carla.Transform](#carla.Transform)_) - 包含将此对象的局部空间转换为世界空间所需的位置和旋转。 
    - **返回：** _bool_  

##### 获取器
- <a name="carla.BoundingBox.get_local_vertices"></a>**<font color="#7fb800">get_local_vertices</font>**(<font color="#00a6ed">**self**</font>)  
返回一个列表，其中包含该对象的顶点在局部空间中的位置。 
    - **返回：** _list([carla.Location](#carla.Location))_  
- <a name="carla.BoundingBox.get_world_vertices"></a>**<font color="#7fb800">get_world_vertices</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**transform**</font>)  
返回一个列表，其中包含该对象的顶点在世界空间中的位置。  
    - **参数：**
        - `transform` (_[carla.Transform](#carla.Transform)_) - 包含将此对象的本地空间转换为世界空间所需的位置和旋转。  
    - **返回：** _list([carla.Location](#carla.Location))_  

##### Dunder 方法
- <a name="carla.BoundingBox.__eq__"></a>**<font color="#7fb800">\__eq__ </font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.BoundingBox](#carla.BoundingBox)</font>)  
如果此和另一个`other`的位置和范围相等，则返回true。
    - **返回：** _bool_  
- <a name="carla.BoundingBox.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.BoundingBox](#carla.BoundingBox)</font>)  
如果此和其他`other`的位置或范围不同，则返回true。
    - **返回：** _bool_  
- <a name="carla.BoundingBox.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
将边界框的位置和范围解析为字符串。  
    - **返回：** _str_  

---

## carla.CityObjectLabel<a name="carla.CityObjectLabel"></a>
包含可用于过滤 [carla.World.get_level_bbs](#carla.World.get_level_bbs)() 返回的边界框的不同标签的枚举声明。这些值对应于场景中元素所具有的 [语义标签](ref_sensors.md#semantic-segmentation-camera) 。  

### 实例变量
- <a name="carla.CityObjectLabel.None"></a>**<font color="#f8805a">None</font>**  
- <a name="carla.CityObjectLabel.Buildings"></a>**<font color="#f8805a">Buildings</font>**  
- <a name="carla.CityObjectLabel.Fences"></a>**<font color="#f8805a">Fences</font>**  
- <a name="carla.CityObjectLabel.Other"></a>**<font color="#f8805a">Other</font>**  
- <a name="carla.CityObjectLabel.Pedestrians"></a>**<font color="#f8805a">Pedestrians</font>**  
- <a name="carla.CityObjectLabel.Poles"></a>**<font color="#f8805a">Poles</font>**  
- <a name="carla.CityObjectLabel.RoadLines"></a>**<font color="#f8805a">RoadLines</font>**  
- <a name="carla.CityObjectLabel.Roads"></a>**<font color="#f8805a">Roads</font>**  
- <a name="carla.CityObjectLabel.Sidewalks"></a>**<font color="#f8805a">Sidewalks</font>**  
- <a name="carla.CityObjectLabel.TrafficSigns"></a>**<font color="#f8805a">TrafficSigns</font>**  
- <a name="carla.CityObjectLabel.Vegetation"></a>**<font color="#f8805a">Vegetation</font>**  
- <a name="carla.CityObjectLabel.Vehicles"></a>**<font color="#f8805a">Vehicles</font>**  
- <a name="carla.CityObjectLabel.Walls"></a>**<font color="#f8805a">Walls</font>**  
- <a name="carla.CityObjectLabel.Sky"></a>**<font color="#f8805a">Sky</font>**  
- <a name="carla.CityObjectLabel.Ground"></a>**<font color="#f8805a">Ground</font>**  
- <a name="carla.CityObjectLabel.Bridge"></a>**<font color="#f8805a">Bridge</font>**  
- <a name="carla.CityObjectLabel.RailTrack"></a>**<font color="#f8805a">RailTrack</font>**  
- <a name="carla.CityObjectLabel.GuardRail"></a>**<font color="#f8805a">GuardRail</font>**  
- <a name="carla.CityObjectLabel.TrafficLight"></a>**<font color="#f8805a">TrafficLight</font>**  
- <a name="carla.CityObjectLabel.Static"></a>**<font color="#f8805a">Static</font>**  
- <a name="carla.CityObjectLabel.Dynamic"></a>**<font color="#f8805a">Dynamic</font>**  
- <a name="carla.CityObjectLabel.Water"></a>**<font color="#f8805a">Water</font>**  
- <a name="carla.CityObjectLabel.Terrain"></a>**<font color="#f8805a">Terrain</font>**  
- <a name="carla.CityObjectLabel.Any"></a>**<font color="#f8805a">Any</font>**  

---

## carla.Client<a name="carla.Client"></a>
客户端将 Carla 连接到运行仿真的服务器。服务器和客户端都包含 Carla 库 (libcarla)，但存在一些差异，允许它们之间进行通信。可以创建许多客户端，每个客户端都会连接到仿真内的 RPC 服务器以发送命令。仿真在服务器端运行。一旦建立连接，客户端将仅接收从仿真检索的数据。步行者是例外。客户端负责管理行人，因此，如果您与多个客户端一起运行仿真，可能会出现一些问题。例如，如果您通过不同的客户端生成步行者，则可能会发生冲突，因为每个客户端只知道它负责的客户端。

客户端还具有记录功能，可以在运行仿真时保存仿真的所有信息。这使得服务器可以随意重放以获取信息并进行实验。[以下](adv_recorder.md) 是有关如何使用此录音机的一些信息。 

### 方法
- <a name="carla.Client.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**host**=127.0.0.1</font>, <font color="#00a6ed">**port**=2000</font>, <font color="#00a6ed">**worker_threads**=0</font>)<button class="SnipetButton" id="carla.Client.__init__-snipet_button">snippet &rarr;</button>  
客户端构造器。  
    - **参数：**
        - `host` (_str_) - 运行Carla 仿真器实例的IP地址。默认值为localhost（127.0.0.1）。  
        - `port` (_int_) - 运行Carla 仿真器实例的TCP端口。默认为 2000 和随后的2001 。 
        - `worker_threads` (_int_) - 用于后台更新的工作线程数。如果为 0，则使用所有可用的并发性。
- <a name="carla.Client.apply_batch"></a>**<font color="#7fb800">apply_batch</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**commands**</font>)  
在单个仿真步骤上执行命令列表，不检索任何信息。如果需要有关每个命令的响应的信息，请使用 __<font color="#7fb800">apply_batch_sync()</font>__ 方法。[下面](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/generate_traffic.py) 是一个关于如何删除卡拉中出现的演员的示例。一次列出所有参与者 [carla.ActorList](#carla.ActorList) 。
    - **参数：**
        - `commands` (_list_) - 要批量执行的命令列表。每个命令都不同，并且有自己的参数。它们显示在此页面底部列出。
- <a name="carla.Client.apply_batch_sync"></a>**<font color="#7fb800">apply_batch_sync</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**commands**</font>, <font color="#00a6ed">**due_tick_cue**=False</font>)<button class="SnipetButton" id="carla.Client.apply_batch_sync-snipet_button">snippet &rarr;</button>  
在单个仿真步骤上执行命令列表，阻止直到命令链接，并返回命令列表。可用于确定单个命令是否成功的响应。[下面](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/generate_traffic.py) 是一个用来生成参与者的例子。
    - **参数：**
        - `commands` (_list_) - 要批量执行的命令列表。可用的命令列在方法 **<font color="#7fb800">apply_batch()</font>** 的正上方。 
        - `due_tick_cue` (_bool_) - 一个布尔参数，用于指定是否执行 Carla。在 __同步模式__ 下应用批处理后进行 [carla.World.tick](#carla.World.tick)。默认情况下为 __False__ 。
    - **返回：** _list(command.Response)_  
- <a name="carla.Client.generate_opendrive_world"></a>**<font color="#7fb800">generate_opendrive_world</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**opendrive**</font>, <font color="#00a6ed">**parameters**=(2.0, 50.0, 1.0, 0.6, true, true)</font>, <font color="#00a6ed">**reset_settings**=True</font>)  
使用从OpenDRIVE文件的内容生成的基本三维拓扑加载新世界。此内容作为`string`参数传递。它类似于`client.load_world(map_name)`，但允许在服务器端自定义OpenDRIVE映射。汽车可以在地图上行驶，但除了道路和人行道之外，没有任何图形。
    - **参数：**
        - `opendrive` (_str_) - OpenDRIVE文件的内容为字符串`string`，__而不是`.xodr`的路径__。  
        - `parameters` (_[carla.OpendriveGenerationParameters](#carla.OpendriveGenerationParameters)_) - 网格生成的其他设置。如果没有提供，将使用默认值。 
        - `reset_settings` (_bool_) - 选项可将剧集设置重置为默认值，设置为false 可保留当前设置。这对于在更改映射时保持同步模式和保持确定性场景非常有用。
- <a name="carla.Client.load_world"></a>**<font color="#7fb800">load_world</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**map_name**</font>, <font color="#00a6ed">**reset_settings**=True</font>, <font color="#00a6ed">**map_layers**=[carla.MapLayer.All](#carla.MapLayer.All)</font>)  
Creates a new world with default settings using `map_name` map. All actors in the current world will be destroyed.  
    - **参数：**
        - `map_name` (_str_) - 要在这个世界上使用的地图的名称。接受完整路径和地图名称，例如“/Game/Calla/Maps/Town01”或“Town01”。请记住，这些路径是动态的。
        - `reset_settings` (_bool_) - 选项可将情节设置重置为默认值，设置为false可保留当前设置。这对于在更改映射时保持同步模式和保持确定性场景非常有用。
        - `map_layers` (_[carla.MapLayer](#carla.MapLayer)_) - 将加载的地图的图层。默认情况下，将加载所有图层。此参数的作用类似于标志掩码。
    - **警告：** <font color="#ED2F2F">_`map_layers` 仅对"Opt"地图可用。
_</font>  
- <a name="carla.Client.reload_world"></a>**<font color="#7fb800">reload_world</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**reset_settings**=True</font>)  
重新加载当前世界，请注意，将使用同一地图使用默认设置创建一个新世界。世界上所有参与者都将被摧毁，__但__ 交通管理器实例将继续存在。
    - **参数：**
        - `reset_settings` (_bool_) - 选项可将情节设置重置为默认值，设置为false可保留当前设置。这对于在更改映射时保持同步模式和保持确定性场景非常有用。
    - **Raises:** 对应的运行时错误 RuntimeError。   
- <a name="carla.Client.replay_file"></a>**<font color="#7fb800">replay_file</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**name**</font>, <font color="#00a6ed">**start**</font>, <font color="#00a6ed">**duration**</font>, <font color="#00a6ed">**follow_id**</font>, <font color="#00a6ed">**replay_sensors**</font>)  
使用`map_name`加载具有默认设置的新世界。当前世界中的所有参与者都将被摧毁，__但__ 交通管理器实例将继续存在。
    - **参数：**
        - `name` (_str_) - 包含仿真信息的文件的名称。
        - `start` (_float<small> - seconds</small>_) - 开始播放仿真的时间。负数表示从结尾开始读取，在录制结束前10秒为-10。
        - `duration` (_float<small> - seconds</small>_) - 将使用信息 `name` 文件重新执行的时间。如果到达终点，仿真将继续。
        - `follow_id` (_int_) - 要跟随的参与者的 ID。如果此值为0，则禁用相机。
        - `replay_sensors` (_bool_) - 用于在播放期间启用或禁用传感器生成的标志。
- <a name="carla.Client.request_file"></a>**<font color="#7fb800">request_file</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**name**</font>)  
由 [carla.Client.get_required_files](#carla.Client.get_required_files) 返回需要文件的一个请求。  
    - **参数：**
        - `name` (_str_) - 您请求的文件的名称。
- <a name="carla.Client.show_recorder_actors_blocked"></a>**<font color="#7fb800">show_recorder_actors_blocked</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**min_time**</font>, <font color="#00a6ed">**min_distance**</font>)  
终端将显示被视为被屏蔽的参与者的注册信息。当参与者在一段时间内没有移动最小距离时，即`min_distance`和`min_time`，则视为被阻挡。  
    - **参数：**
        - `filename` (_str_) - 要加载的录制文件的名称。 
        - `min_time` (_float<small> - seconds</small>_) - 参与者在被认为被阻挡之前必须移动最短距离的最短时间。默认值为60秒。  
        - `min_distance` (_float<small> - centimeters</small>_) - 参与者必须移动的最小距离才能不被视为被阻挡。默认值为100厘米。
    - **返回：** _string_  
- <a name="carla.Client.show_recorder_collisions"></a>**<font color="#7fb800">show_recorder_collisions</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**category1**</font>, <font color="#00a6ed">**category2**</font>)  
终端将显示记录器记录的碰撞。可以通过指定所涉及的参与者的类型来过滤这些内容。类别将在`category1`和`category1`中指定如下：“h”=英雄，一种可以手动控制或由用户管理的车辆。”v'=车辆'w'=步行者't'=红绿灯'o'=其他'a'=任何如果您只想看到车辆和步行者之间的碰撞，请将`category1`用作'v'，将`category2`用作'w'，反之亦然。如果要查看所有碰撞（过滤掉），可以对两个参数都使用“a”。 
    - **参数：**
        - `filename` (_str_) - 记录的文件的名称或绝对路径，具体取决于您之前的选择。 
        - `category1` (_single char_) - 指定冲突中涉及的第一类参与者的字符变量。
        - `category2` (_single char_) - 指定冲突中涉及的第二类参与者的字符变量。
    - **返回：** _string_  
- <a name="carla.Client.show_recorder_file_info"></a>**<font color="#7fb800">show_recorder_file_info</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**show_all**</font>)  
记录器保存的信息将被解析并以文本形式显示在您的终端中（帧、时间、事件、状态、位置…）。可以使用`show_all`参数指定显示的信息。[以下](ref_recorder_binary_file_format.md) 是有关如何读取记录器文件的更多信息。
    - **参数：**
        - `filename` (_str_) - 记录的文件的名称或绝对路径，具体取决于您之前的选择。
        - `show_all` (_bool_) - 如果为 __True__ ，则返回为每帧存储的所有信息（红绿灯状态、所有参与者的位置、方向和动画数据…）。如果为 __False__ ，则返回关键事件和帧的摘要。 
    - **返回：** _string_  
- <a name="carla.Client.start_recorder"></a>**<font color="#7fb800">start_recorder</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**filename**</font>, <font color="#00a6ed">**additional_data**=False</font>)  
启用录制功能，该功能将开始保存服务器重播仿真所需的所有信息。
    - **参数：**
        - `filename` (_str_) - 用于写入记录数据的文件的名称。一个简单的名称会将录制保存在“CarlaUE4/Saved/recording.log”中。否则，如果名称中出现某个文件夹，则会将其视为绝对路径。
        - `additional_data` (_bool_) - 启用或禁用记录用于再现仿真的非必要数据（边界框位置、物理控制参数等）。
- <a name="carla.Client.stop_recorder"></a>**<font color="#7fb800">stop_recorder</font>**(<font color="#00a6ed">**self**</font>)  
停止正在进行的录制。如果您在文件名中指定了一个路径，则录制文件将在那里。如果没有，请查看`CarlaUE4/Saved/`内部。
- <a name="carla.Client.stop_replayer"></a>**<font color="#7fb800">stop_replayer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**keep_actors**</font>)  
停止当前重播。 
    - **参数：**
        - `keep_actors` (_bool_) - 如果你想自动删除重播中的所有参与者，则为 True，如果你想保留他们，则为 False。

##### 获取器
- <a name="carla.Client.get_available_maps"></a>**<font color="#7fb800">get_available_maps</font>**(<font color="#00a6ed">**self**</font>)  
返回一个字符串列表，其中包含服务器上可用映射的路径。这些路径是动态的，它们将在仿真过程中创建，因此在文件中查找时找不到它们。此方法可能的返回结果之一是：
  ['/Game/Carla/Maps/Town01',
  '/Game/Carla/Maps/Town02',
  '/Game/Carla/Maps/Town03',
  '/Game/Carla/Maps/Town04',
  '/Game/Carla/Maps/Town05',
  '/Game/Carla/Maps/Town06',
  '/Game/Carla/Maps/Town07'].  
    - **返回：** _list(str)_  
- <a name="carla.Client.get_client_version"></a>**<font color="#7fb800">get_client_version</font>**(<font color="#00a6ed">**self**</font>)  
通过在“version.h”文件中查阅客户端libcarla版本，返回该版本。客户端和服务器都可以使用不同的libcarla版本，但可能会出现一些与意外不兼容有关的问题。 
    - **返回：** _str_  
- <a name="carla.Client.get_required_files"></a>**<font color="#7fb800">get_required_files</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**folder**</font>, <font color="#00a6ed">**download**=True</font>)  
询问服务器客户端需要哪些文件才能使用当前地图。选项，如果文件不在缓存中，则自动下载文件。
    - **参数：**
        - `folder` (_str_) - 将客户端所需文件下载到的文件夹。
        - `download` (_bool_) - 如果为True，则下载尚未在缓存中的文件。 
- <a name="carla.Client.get_server_version"></a>**<font color="#7fb800">get_server_version</font>**(<font color="#00a6ed">**self**</font>)  
通过在“version.h”文件中查阅服务器libcarla版本，返回该版本。客户端和服务器都应该使用相同的libcarla版本。  
    - **返回：** _str_  
- <a name="carla.Client.get_trafficmanager"></a>**<font color="#7fb800">get_trafficmanager</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**client_connection**=8000</font>)  
返回与指定端口相关的交通管理器实例。如果该实例不存在，则会创建该实例。 
    - **参数：**
        - `client_connection` (_int_) - 将由交通管理器使用的端口。默认值为`8000`。 
    - **返回：** _[carla.TrafficManager](#carla.TrafficManager)_  
- <a name="carla.Client.get_world"></a>**<font color="#7fb800">get_world</font>**(<font color="#00a6ed">**self**</font>)  
返回仿真中当前处于活动状态的世界对象。该世界稍后将用于例如加载地图。
    - **返回：** _[carla.World](#carla.World)_  

##### 设置器
- <a name="carla.Client.set_files_base_folder"></a>**<font color="#7fb800">set_files_base_folder</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>)  
    - **参数：**
        - `path` (_str_) - 返回仿真中当前处于活动状态的世界对象。该世界稍后将用于例如加载地图。 
- <a name="carla.Client.set_replayer_ignore_hero"></a>**<font color="#7fb800">set_replayer_ignore_hero</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**ignore_hero**</font>)  
    - **参数：**
        - `ignore_hero` (_bool_) - 在播放录制的仿真过程中启用或禁用英雄车辆的播放。 
- <a name="carla.Client.set_replayer_ignore_spectator"></a>**<font color="#7fb800">set_replayer_ignore_spectator</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**ignore_spectator**</font>)  
    - **参数：**
        - `ignore_spectator` (_bool_) - 确定回放程序是否会复制记录的参与者动作。
- <a name="carla.Client.set_replayer_time_factor"></a>**<font color="#7fb800">set_replayer_time_factor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**time_factor**=1.0</font>)  
使用时，会随意修改重新仿真的时间速度。当播放时，它可以使用多次。
    - **参数：**
        - `time_factor` (_float_) - 1.0表示正常时间速度。大于1.0表示快速运动（2.0表示双倍速度），小于1.0表示慢速运动（0.5表示一半速度）。  
- <a name="carla.Client.set_timeout"></a>**<font color="#7fb800">set_timeout</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seconds**</font>)  
设置在阻止网络调用并引发超时超时错误之前允许网络调用的最长时间。
    - **参数：**
        - `seconds` (_float<small> - seconds</small>_) - 新的超时值。默认值为5秒。

---

## carla.CollisionEvent<a name="carla.CollisionEvent"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
为<b>sensor.other.collision</b>定义碰撞数据的类。传感器为检测到的每个碰撞创建一个碰撞数据。每个碰撞传感器每帧每个碰撞产生一个碰撞事件。通过与多个其他参与者的碰撞，可以在单个帧中产生多个碰撞事件。点击 [此处](ref_sensors.md#collision-detector) 了解更多信息。  

### 实例变量
- <a name="carla.CollisionEvent.actor"></a>**<font color="#f8805a">actor</font>** (_[carla.Actor](#carla.Actor)_)  
传感器连接到的参与者，也就是测量碰撞的参与者。
- <a name="carla.CollisionEvent.other_actor"></a>**<font color="#f8805a">other_actor</font>** (_[carla.Actor](#carla.Actor)_)  
第二个参与碰撞的参与者。
- <a name="carla.CollisionEvent.normal_impulse"></a>**<font color="#f8805a">normal_impulse</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - N*s</small>_)  
碰撞产生的正常脉冲。

---

## carla.Color<a name="carla.Color"></a>
定义32位RGBA颜色的类。  

### 实例变量
- <a name="carla.Color.r"></a>**<font color="#f8805a">r</font>** (_int_)  
红色 (0-255)。 
- <a name="carla.Color.g"></a>**<font color="#f8805a">g</font>** (_int_)  
绿色 (0-255)。 
- <a name="carla.Color.b"></a>**<font color="#f8805a">b</font>** (_int_)  
蓝色 (0-255)。
- <a name="carla.Color.a"></a>**<font color="#f8805a">a</font>** (_int_)  
Alpha 通道 (0-255).。  

### 方法
- <a name="carla.Color.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**r**=0</font>, <font color="#00a6ed">**g**=0</font>, <font color="#00a6ed">**b**=0</font>, <font color="#00a6ed">**a**=255</font>)  
初始化颜色，默认为黑色。  
    - **参数：**
        - `r` (_int_)  
        - `g` (_int_)  
        - `b` (_int_)  
        - `a` (_int_)  

##### Dunder 方法
- <a name="carla.Color.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Color](#carla.Color)</font>)  
- <a name="carla.Color.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Color](#carla.Color)</font>)  
- <a name="carla.Color.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.ColorConverter<a name="carla.ColorConverter"></a>
定义可应用于 [carla.Image](#carla.Image) 的转换模式的类，以显示 [carla.Sensor](#carla.Sensor) 提供的信息。深度转换会导致精度损失，因为传感器将深度检测为浮点数，然后将其转换为 0 到 255 之间的灰度值。查看 [carla.Sensor.listen](#carla.Sensor.listen) 中的片段，了解如何创建和保存图像的示例<b>sensor.camera.semantic_segmentation</b>的数据。

### 实例变量
- <a name="carla.ColorConverter.CityScapesPalette"></a>**<font color="#f8805a">CityScapesPalette</font>**  
使用蓝图库提供的标签将图像转换为分段地图。由 [语义分割相机](ref_sensors.md#semantic-segmentation-camera) 使用。
- <a name="carla.ColorConverter.Depth"></a>**<font color="#f8805a">Depth</font>**  
将图像转换为线性深度图。由 [深度相机](ref_sensors.md#depth-camera) 使用。
- <a name="carla.ColorConverter.LogarithmicDepth"></a>**<font color="#f8805a">LogarithmicDepth</font>**  
使用对数刻度将图像转换为深度图，从而在小距离上获得更好的精度，但在距离较远时会丢失精度。
- <a name="carla.ColorConverter.Raw"></a>**<font color="#f8805a">Raw</font>**  
未对图像应用任何更改。由 [RGB 相机](ref_sensors.md#rgb-camera) 使用。

---

## carla.DVSEvent<a name="carla.DVSEvent"></a>
定义 DVS 事件的类。事件是一个四元组，因此是一个由 4 个元素组成的元组，其中`x`、像素`y`坐标位置、时间戳`t`和事件极性`pol`。在 [这里](ref_sensors.md) 了解更多关于它们的信息。

### 实例变量
- <a name="carla.DVSEvent.x"></a>**<font color="#f8805a">x</font>** (_int_)  
X 像素坐标。
- <a name="carla.DVSEvent.y"></a>**<font color="#f8805a">y</font>** (_int_)  
Y 像素坐标。  
- <a name="carla.DVSEvent.t"></a>**<font color="#f8805a">t</font>** (_int_)  
事件发生时刻的时间戳。
- <a name="carla.DVSEvent.pol"></a>**<font color="#f8805a">pol</font>** (_bool_)  
事件的极性。__True__ 表示正数， __False__ 表示负数。

### 方法

##### Dunder 方法
- <a name="carla.DVSEvent.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.DVSEventArray<a name="carla.DVSEventArray"></a>
在 [carla.DVSEvent](#carla.DVSEvent) 中定义事件流的类。此类流是任意大小的数组，具体取决于事件的数量。为了方便起见，该类还存储视野、图像的高度和宽度以及时间戳。在 [这里](ref_sensors.md) 了解更多关于它们的信息。

### 实例变量
- <a name="carla.DVSEventArray.fov"></a>**<font color="#f8805a">fov</font>** (_float<small> - 度</small>_)  
图像的水平视野。 
- <a name="carla.DVSEventArray.height"></a>**<font color="#f8805a">height</font>** (_int_)  
图像高度（以像素为单位）。
- <a name="carla.DVSEventArray.width"></a>**<font color="#f8805a">width</font>** (_int_)  
图像宽度（以像素为单位）。
- <a name="carla.DVSEventArray.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_字节_)  

### 方法
- <a name="carla.DVSEventArray.to_array"></a>**<font color="#7fb800">to_array</font>**(<font color="#00a6ed">**self**</font>)  
按以下顺序将事件流转换为 int 值数组<code>[x, y, t, pol]</code>。
- <a name="carla.DVSEventArray.to_array_pol"></a>**<font color="#7fb800">to_array_pol</font>**(<font color="#00a6ed">**self**</font>)  
返回一个数组，该数组具有流中所有事件的极性。 
- <a name="carla.DVSEventArray.to_array_t"></a>**<font color="#7fb800">to_array_t</font>**(<font color="#00a6ed">**self**</font>)  
返回一个数组，其中包含流中所有事件的时间戳。
- <a name="carla.DVSEventArray.to_array_x"></a>**<font color="#7fb800">to_array_x</font>**(<font color="#00a6ed">**self**</font>)  
返回流中所有事件的X像素坐标的数组。
- <a name="carla.DVSEventArray.to_array_y"></a>**<font color="#7fb800">to_array_y</font>**(<font color="#00a6ed">**self**</font>)  
返回流中所有事件的Y像素坐标数组。
- <a name="carla.DVSEventArray.to_image"></a>**<font color="#7fb800">to_image</font>**(<font color="#00a6ed">**self**</font>)  
按照以下模式转换图像：蓝色表示正事件，红色表示负事件。

##### 魔术方法
- <a name="carla.DVSEventArray.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.DVSEventArray.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
在 [carla.DVSEvent](#carla.DVSEvent) 作为检索的数据上迭代。
- <a name="carla.DVSEventArray.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.DVSEventArray.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**color**=[carla.Color](#carla.Color)</font>)  
- <a name="carla.DVSEventArray.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.DebugHelper<a name="carla.DebugHelper"></a>
[carla.World](#carla.World) 的辅助类部分，定义创建调试形状的方法。默认情况下，形状持续一秒。它们可以是永久性的，但要考虑到这样做所需的资源。查看此类的可用片段，了解如何在 Carla 中轻松调试。

### 方法
- <a name="carla.DebugHelper.draw_arrow"></a>**<font color="#7fb800">draw_arrow</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**begin**</font>, <font color="#00a6ed">**end**</font>, <font color="#00a6ed">**thickness**=0.1</font>, <font color="#00a6ed">**arrow_size**=0.1</font>, <font color="#00a6ed">**color**=(255,0,0)</font>, <font color="#00a6ed">**life_time**=-1.0</font>)  
从开始`begin`到结束`end`绘制一个指向该方向的箭头。 
    - **参数：**
        - `begin` (_[carla.Location](#carla.Location)<small> - meters</small>_) - 坐标系中箭头开始的点。
        - `end` (_[carla.Location](#carla.Location)<small> - meters</small>_) -  坐标系中箭头结束并指向的点。
        - `thickness` (_float<small> - meters</small>_) - 线的密度。  
        - `arrow_size` (_float<small> - meters</small>_) - 箭头尖端的大小。
        - `color` (_[carla.Color](#carla.Color)_) - 用于为对象着色的 RGB 代码。默认为红色。
        - `life_time` (_float<small> - seconds</small>_) - 形状的生命周期。默认情况下它只持续一帧。将其设置<code>0</code>为永久形状。 
- <a name="carla.DebugHelper.draw_box"></a>**<font color="#7fb800">draw_box</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**box**</font>, <font color="#00a6ed">**rotation**</font>, <font color="#00a6ed">**thickness**=0.1</font>, <font color="#00a6ed">**color**=(255,0,0)</font>, <font color="#00a6ed">**life_time**=-1.0</font>)<button class="SnipetButton" id="carla.DebugHelper.draw_box-snipet_button">snippet &rarr;</button>  
Draws a box, ussually to act for object colliders.  
    - **Parameters:**
        - `box` (_[carla.BoundingBox](#carla.BoundingBox)_) - 包含每个轴的位置和框长度的对象。
        - `rotation` (_[carla.Rotation](#carla.Rotation)<small> - 度 (pitch,yaw,roll)</small>_) - 根据虚幻引擎轴系的边界框方向。
        - `thickness` (_float<small> - meters</small>_) - 定义边界框的线的密度。
        - `color` (_[carla.Color](#carla.Color)_) - 用于为对象着色的 RGB 代码。默认为红色。
        - `life_time` (_float<small> - seconds</small>_) - 形状的生命周期。默认情况下它只持续一帧。将其设置<code>0</code>为永久形状。
- <a name="carla.DebugHelper.draw_line"></a>**<font color="#7fb800">draw_line</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**begin**</font>, <font color="#00a6ed">**end**</font>, <font color="#00a6ed">**thickness**=0.1</font>, <font color="#00a6ed">**color**=(255,0,0)</font>, <font color="#00a6ed">**life_time**=-1.0</font>)  
在开始`begin` 和结束 `end`之间绘制一条线。  
    - **参数：**
        - `begin` (_[carla.Location](#carla.Location)<small> - meters</small>_) -  坐标系中直线起点。
        - `end` (_[carla.Location](#carla.Location)<small> - meters</small>_) - 坐标系中直线终点。 
        - `thickness` (_float<small> - 米</small>_) - 线的密度。  
        - `color` (_[carla.Color](#carla.Color)_) - 用于为对象着色的 RGB 代码。默认为红色。
        - `life_time` (_float<small> - seconds</small>_) - 形状的生命周期。默认情况下它只持续一帧。将其设置<code>0</code>为永久形状。
- <a name="carla.DebugHelper.draw_point"></a>**<font color="#7fb800">draw_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**size**=0.1</font>, <font color="#00a6ed">**color**=(255,0,0)</font>, <font color="#00a6ed">**life_time**=-1.0</font>)  
绘制一个点 `location`。
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)<small> - meters</small>_) - 在坐标系中点以将对象居中。
        - `size` (_float<small> - meters</small>_) - 点的密度。
        - `color` (_[carla.Color](#carla.Color)_) - 用于为对象着色的 RGB 代码。默认为红色。
        - `life_time` (_float<small> - seconds</small>_) - 形状的生命周期。默认情况下它只持续一帧。将其设置<code>0</code>为永久形状。
- <a name="carla.DebugHelper.draw_string"></a>**<font color="#7fb800">draw_string</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**text**</font>, <font color="#00a6ed">**draw_shadow**=False</font>, <font color="#00a6ed">**color**=(255,0,0)</font>, <font color="#00a6ed">**life_time**=-1.0</font>)<button class="SnipetButton" id="carla.DebugHelper.draw_string-snipet_button">snippet &rarr;</button>  
在仿真的给定位置绘制一个字符串，该字符串只能在服务器端看到。
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)<small> - meters</small>_) - 仿真中文本居中的位置。
        - `text` (_str_) - 旨在向世界展示的文本。 
        - `draw_shadow` (_bool_) - 为字符串投射阴影，有助于可视化。默认情况下它是禁用的。
        - `color` (_[carla.Color](#carla.Color)_) - 用于为字符串着色的 RGB 代码。默认为红色。
        - `life_time` (_float<small> - seconds</small>_) - 形状的生命周期。默认情况下它只持续一帧。将其设置<code>0</code>为永久形状。

---

## carla.EnvironmentObject<a name="carla.EnvironmentObject"></a>
表示关卡中的几何图形的类，该几何图形可以是与其他环境对象（即：建筑物）形成的参与者的一部分。

### 实例变量
- <a name="carla.EnvironmentObject.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
包含环境对象在世界空间中的位置和方向。
- <a name="carla.EnvironmentObject.bounding_box"></a>**<font color="#f8805a">bounding_box</font>** (_[carla.BoundingBox](#carla.BoundingBox)_)  
包含世界空间中每个轴的位置、旋转和边界框长度的对象。  
- <a name="carla.EnvironmentObject.id"></a>**<font color="#f8805a">id</font>** (_int_)  
用于标识关卡中对象的唯一 ID。  
- <a name="carla.EnvironmentObject.name"></a>**<font color="#f8805a">name</font>** (_string_)  
环境对象 EnvironmentObject 的名称。  
- <a name="carla.EnvironmentObject.type"></a>**<font color="#f8805a">type</font>** (_[carla.CityObjectLabel](#carla.CityObjectLabel)_)  
Semantic tag.  

### 方法

##### Dunder 方法
- <a name="carla.EnvironmentObject.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
将环境对象解析为字符串并在命令行中显示它们。  
    - **返回：** _str_  

---

## carla.FloatColor<a name="carla.FloatColor"></a>
定义浮点 RGBA 颜色的类。

### 实例变量
- <a name="carla.FloatColor.r"></a>**<font color="#f8805a">r</font>** (_float_)  
红色。  
- <a name="carla.FloatColor.g"></a>**<font color="#f8805a">g</font>** (_float_)  
绿色。 
- <a name="carla.FloatColor.b"></a>**<font color="#f8805a">b</font>** (_float_)  
蓝色。  
- <a name="carla.FloatColor.a"></a>**<font color="#f8805a">a</font>** (_float_)  
Alpha 通道。

### 方法
- <a name="carla.FloatColor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**r**=0</font>, <font color="#00a6ed">**g**=0</font>, <font color="#00a6ed">**b**=0</font>, <font color="#00a6ed">**a**=1.0</font>)  
初始化颜色，默认为黑色。  
    - **参数：**
        - `r` (_float_)  
        - `g` (_float_)  
        - `b` (_float_)  
        - `a` (_float_)  

##### Dunder 方法
- <a name="carla.FloatColor.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.FloatColor](#carla.FloatColor)</font>)  
- <a name="carla.FloatColor.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.FloatColor](#carla.FloatColor)</font>)  

---

## carla.GBufferTextureID<a name="carla.GBufferTextureID"></a>
定义每个GBuffer纹理的标识符 (参见方法 `[carla.Sensor.listen_to_gbuffer](#carla.Sensor.listen_to_gbuffer)`)。 

### 实例变量
- <a name="carla.GBufferTextureID.SceneColor"></a>**<font color="#f8805a">SceneColor</font>**  
纹理“SceneColor”包含图像的最终颜色。
- <a name="carla.GBufferTextureID.SceneDepth"></a>**<font color="#f8805a">SceneDepth</font>**  
纹理“SceneDepth”包含深度缓冲区 - 以世界单位为线性。
- <a name="carla.GBufferTextureID.SceneStencil"></a>**<font color="#f8805a">SceneStencil</font>**  
纹理“SceneStencil”包含模板缓冲区。
- <a name="carla.GBufferTextureID.GBufferA"></a>**<font color="#f8805a">GBufferA</font>**  
纹理“GBufferA”包含 RGB 通道中的世界空间法线向量。Alpha 通道包含“每个对象的数据”。
- <a name="carla.GBufferTextureID.GBufferB"></a>**<font color="#f8805a">GBufferB</font>**  
纹理“GBufferB”分别包含 RGB 通道中的金属、镜面和粗糙度。Alpha 通道包含一个掩码，其中低 4 位表示着色模型，高 4 位包含选择性输出掩码。
- <a name="carla.GBufferTextureID.GBufferC"></a>**<font color="#f8805a">GBufferC</font>**  
纹理“GBufferC”包含 RGB 通道中的漫反射颜色，以及 Alpha 通道中的间接辐照度。
如果不允许静态闪电，Alpha 通道将包含环境光遮挡。
- <a name="carla.GBufferTextureID.GBufferD"></a>**<font color="#f8805a">GBufferD</font>**  
“GBufferD”的内容根据渲染对象的材质着色模型 (GBufferB) 的不同而有所不同：<br>
  - MSM_Subsurface (2), MSM_PreintegratedSkin (3), MSM_TwoSidedFoliage (6):<br>
    RGB: 次表面颜色。<br>
    A: 不透明度。<br>
  - MSM_ClearCoat (4):<br>
    R: 透明涂层。<br>
    G: 粗糙度。<br>
  - MSM_SubsurfaceProfile (5):<br>
    RGB: 次表面轮廓。<br>
  - MSM_Hair (7):<br>
    RG: 世界法线。<br>
    B: 背光值。<br>
  - MSM_Cloth (8):<br>
    RGB: 次表面颜色。<br>
    A: Cloth value.<br>
  - MSM_Eye (9):<br>
    RG: 眼切线。<br>
    B: 虹膜掩模。<br>
    A: 虹膜距离。
- <a name="carla.GBufferTextureID.GBufferE"></a>**<font color="#f8805a">GBufferE</font>**  
纹理“GBufferE”包含 RGBA 通道中预先计算的阴影因子。如果选择性输出掩码 (GBufferB) 没有设置其第 4 位，则此纹理不可用。
- <a name="carla.GBufferTextureID.GBufferF"></a>**<font color="#f8805a">GBufferF</font>**  
纹理“GBufferF”包含 RGB 通道中的世界空间切线和 Alpha 通道中的各向异性。如果选择性输出掩码 (GBufferB) 没有设置其第 5 位，则此纹理不可用。
- <a name="carla.GBufferTextureID.Velocity"></a>**<font color="#f8805a">Velocity</font>**  
纹理“速度”包含场景对象的屏幕空间速度。  
- <a name="carla.GBufferTextureID.SSAO"></a>**<font color="#f8805a">SSAO</font>**  
纹理“SSAO”包含屏幕空间环境光遮挡纹理。
- <a name="carla.GBufferTextureID.CustomDepth"></a>**<font color="#f8805a">CustomDepth</font>**  
纹理“CustomDepth”包含虚幻引擎自定义深度数据。
- <a name="carla.GBufferTextureID.CustomStencil"></a>**<font color="#f8805a">CustomStencil</font>**  
纹理“CustomStencil”包含虚幻引擎自定义模板数据。

---

## carla.GearPhysicsControl<a name="carla.GearPhysicsControl"></a>
通过定义档位以及何时运行该档位来提供对车辆变速箱详细信息的访问的类。[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl) 稍后将使用它来帮助仿真物理。

### 实例变量
- <a name="carla.GearPhysicsControl.ratio"></a>**<font color="#f8805a">ratio</font>** (_float_)  
齿轮的传动比。
- <a name="carla.GearPhysicsControl.down_ratio"></a>**<font color="#f8805a">down_ratio</font>** (_float_)  
当前 RPM 与自动变速箱应降档的 MaxRPM 之间的商。
- <a name="carla.GearPhysicsControl.up_ratio"></a>**<font color="#f8805a">up_ratio</font>** (_float_)  
当前 RPM 与自动变速箱应升档的 MaxRPM 之间的商。

### 方法
- <a name="carla.GearPhysicsControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**ratio**=1.0</font>, <font color="#00a6ed">**down_ratio**=0.5</font>, <font color="#00a6ed">**up_ratio**=0.65</font>)  
    - **参数：**
        - `ratio` (_float_)  
        - `down_ratio` (_float_)  
        - `up_ratio` (_float_)  

##### Dunder 方法
- <a name="carla.GearPhysicsControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.GearPhysicsControl](#carla.GearPhysicsControl)</font>)  
- <a name="carla.GearPhysicsControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.GearPhysicsControl](#carla.GearPhysicsControl)</font>)  
- <a name="carla.GearPhysicsControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.GeoLocation<a name="carla.GeoLocation"></a>
包含地理坐标仿真数据的类。[carla.Map](#carla.Map) 可以使用以下方法转换仿真位置 OpenDRIVE 文件中的 <b><georeference></b> 标记。

### 实例变量
- <a name="carla.GeoLocation.latitude"></a>**<font color="#f8805a">latitude</font>** (_float<small> - degrees</small>_)  
地图上某个点的北/南值。  
- <a name="carla.GeoLocation.longitude"></a>**<font color="#f8805a">longitude</font>** (_float<small> - degrees</small>_)  
地图上某个点的北/南值。  
- <a name="carla.GeoLocation.altitude"></a>**<font color="#f8805a">altitude</font>** (_float<small> - meters</small>_)  
相对于地面的高度。 

### 方法
- <a name="carla.GeoLocation.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**latitude**=0.0</font>, <font color="#00a6ed">**longitude**=0.0</font>, <font color="#00a6ed">**altitude**=0.0</font>)  
    - **参数：**
        - `latitude` (_float<small> - degrees</small>_)  
        - `longitude` (_float<small> - degrees</small>_)  
        - `altitude` (_float<small> - meters</small>_)  

##### 魔术方法
- <a name="carla.GeoLocation.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.GeoLocation](#carla.GeoLocation)</font>)  
- <a name="carla.GeoLocation.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.GeoLocation](#carla.GeoLocation)</font>)  
- <a name="carla.GeoLocation.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.GnssMeasurement<a name="carla.GnssMeasurement"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
定义由 <b>sensor.other.gnss</b> 注册的Gnss数据的类。它本质上是通过传感器的位置和 OpenDRIVE 地理参考来报告其位置。

### 实例变量
- <a name="carla.GnssMeasurement.altitude"></a>**<font color="#f8805a">altitude</font>** (_float<small> - meters</small>_)  
相对于地面的高度。  
- <a name="carla.GnssMeasurement.latitude"></a>**<font color="#f8805a">latitude</font>** (_float<small> - degrees</small>_)  
地图上某个点的北/南值。  
- <a name="carla.GnssMeasurement.longitude"></a>**<font color="#f8805a">longitude</font>** (_float<small> - degrees</small>_)  
地图上某个点的北/南值。 

### 方法

##### 魔术方法
- <a name="carla.GnssMeasurement.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.IMUMeasurement<a name="carla.IMUMeasurement"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
定义由 <b>sensor.other.imu</b> 注册的数据的类，涉及根据当前 [carla.World](#carla.World) 的传感器转换。它本质上充当加速计、陀螺仪和指南针。

### 实例变量
- <a name="carla.IMUMeasurement.accelerometer"></a>**<font color="#f8805a">accelerometer</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - m/s<sup>2</sup></small>_)  
线性加速度。
- <a name="carla.IMUMeasurement.compass"></a>**<font color="#f8805a">compass</font>** (_float<small> - radians</small>_)  
相对于北方的方向（在虚幻引擎中为 [0.0, -1.0, 0.0]）。
- <a name="carla.IMUMeasurement.gyroscope"></a>**<font color="#f8805a">gyroscope</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - rad/s</small>_)  
角速度。 

### 方法

##### 魔术方法
- <a name="carla.IMUMeasurement.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Image<a name="carla.Image"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
定义 32 位 BGRA 颜色图像的类，该图像将用作相机传感器检索的初始数据。有不同的相机传感器（目前有三种，RGB、深度和语义分割），每种传感器对图像都有不同的用途。在 [这里](ref_sensors.md) 了解更多关于它们的信息。

### 实例变量
- <a name="carla.Image.fov"></a>**<font color="#f8805a">fov</font>** (_float<small> - degrees</small>_)  
图像的水平视野。
- <a name="carla.Image.height"></a>**<font color="#f8805a">height</font>** (_int_)  
图像高度（以像素为单位）。
- <a name="carla.Image.width"></a>**<font color="#f8805a">width</font>** (_int_)  
图像宽度（以像素为单位）。
- <a name="carla.Image.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_bytes_)  
像素数据的扁平数组，使用 reshape 创建图像数组。

### 方法
- <a name="carla.Image.convert"></a>**<font color="#7fb800">convert</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**color_converter**</font>)  
按照`color_converter`模式转换图像。
    - **参数：**
        - `color_converter` (_[carla.ColorConverter](#carla.ColorConverter)_)  
- <a name="carla.Image.save_to_disk"></a>**<font color="#7fb800">save_to_disk</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>, <font color="#00a6ed">**color_converter**=Raw</font>)  
使用如下所示的转换器模式`color_converter`将图像保存到磁盘。默认转换模式是 <b>Raw</b>，不会对图像进行任何更改。
    - **参数：**
        - `path` (_str_) - 将包含图像的路径。  
        - `color_converter` (_[carla.ColorConverter](#carla.ColorConverter)_) - 默认<b>Raw</b>不会进行任何更改。 

##### 魔术方法
- <a name="carla.Image.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.Image.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
在形成图像的 [carla.Color](#carla.Color) 上面进行迭代。
- <a name="carla.Image.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.Image.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**color**=[carla.Color](#carla.Color)</font>)  
- <a name="carla.Image.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Junction<a name="carla.Junction"></a>
根据 OpenDRIVE 1.4 标准，体现 OpenDRIVE 文件中描述的道路交叉口的类。

### 实例变量
- <a name="carla.Junction.id"></a>**<font color="#f8805a">id</font>** (_int_)  
OpenDRIVE 文件中找到的标识符。
- <a name="carla.Junction.bounding_box"></a>**<font color="#f8805a">bounding_box</font>** (_[carla.BoundingBox](#carla.BoundingBox)_)  
封装连接车道的边界框。

### 方法

##### 获取器
- <a name="carla.Junction.get_waypoints"></a>**<font color="#7fb800">get_waypoints</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lane_type**</font>)  
返回航路点对的列表。列表中的每个元组首先包含交叉口边界内的初始路径点，然后包含最终路径点，描述沿交叉口的所述车道的起点和终点。车道遵循其 OpenDRIVE 定义，因此由于可能的偏差，可能有许多不同的元组具有相同的起始航路点，因为这被视为不同的车道。
    - **参数：**
        - `lane_type` (_[carla.LaneType](#carla.LaneType)_) - Type of lanes to get the waypoints.  
    - **返回：** _list(tuple([carla.Waypoint](#carla.Waypoint)))_  

---

## carla.LabelledPoint<a name="carla.LabelledPoint"></a>
使用语义标签表示空间位置的类。

### 实例变量
- <a name="carla.LabelledPoint.location"></a>**<font color="#f8805a">location</font>**  
三维空间中的位置。
- <a name="carla.LabelledPoint.label"></a>**<font color="#f8805a">label</font>**  
点的语义标签。 

---

## carla.Landmark<a name="carla.Landmark"></a>
定义影响道路的任何类型的交通地标或标志的类。这些类在地标的 [OpenDRIVE 1.4 standard](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf) 标准定义及其在仿真中的表示之间进行协调。此类检索在 OpenDRIVE 中定义地标的所有信息，并提供有关其影响哪些车道以及何时影响的信息。[carla.Waypoint](#carla.Waypoint) 对象将尝试检索其车道规则来访问地标。因此，某些属性取决于正在咨询地标的航路点，从而创建对象。

### 实例变量
- <a name="carla.Landmark.road_id"></a>**<font color="#f8805a">road_id</font>** (_int_)  
定义此地标的道路的 OpenDRIVE ID。由于 OpenDRIVE 道路定义，该道路可能与地标当前影响的道路不同。这种情况主要发生在不同路线的道路分叉处。 <small>示例：在交叉路口的一条分叉道路中定义了一个红绿灯，但它会影响所有可能的路线</small>。
- <a name="carla.Landmark.distance"></a>**<font color="#f8805a">distance</font>** (_float<small> - meters</small>_)  
地标与创建对象的路径点之间的距离（查询`get_landmarks`或`get_landmarks_of_type`）。
- <a name="carla.Landmark.s"></a>**<font color="#f8805a">s</font>** (_float<small> - meters</small>_)  
地标沿道路几何形状定位的距离`road_id`。
- <a name="carla.Landmark.t"></a>**<font color="#f8805a">t</font>** (_float<small> - meters</small>_)  
地标距道路边缘的横向距离`road_id`。 
- <a name="carla.Landmark.id"></a>**<font color="#f8805a">id</font>** (_str_)  
OpenDRIVE 文件中地标的唯一 ID。  
- <a name="carla.Landmark.name"></a>**<font color="#f8805a">name</font>** (_str_)  
OpenDRIVE 文件中的地标名称。 
- <a name="carla.Landmark.is_dynamic"></a>**<font color="#f8805a">is_dynamic</font>** (_bool_)  
指示地标的状态是否随时间发生变化，例如交通信号灯。 
- <a name="carla.Landmark.orientation"></a>**<font color="#f8805a">orientation</font>** (_[carla.LandmarkOrientation](#carla.LandmarkOrientation)<small> - degrees</small>_)  
指示地标面向的车道。
- <a name="carla.Landmark.z_offset"></a>**<font color="#f8805a">z_offset</font>** (_float<small> - meters</small>_)  
放置地标的高度。 
- <a name="carla.Landmark.country"></a>**<font color="#f8805a">country</font>** (_str_)  
定义地标的国家/地区代码（OpenDRIVE 默认为德国 2017）。  
- <a name="carla.Landmark.type"></a>**<font color="#f8805a">type</font>** (_str_)  
根据国家/地区代码输入地标的类型标识符。
- <a name="carla.Landmark.sub_type"></a>**<font color="#f8805a">sub_type</font>** (_str_)  
根据国家/地区代码的地标子类型标识符。  
- <a name="carla.Landmark.value"></a>**<font color="#f8805a">value</font>** (_float_)  
信号中打印的值（例如速度限制、最大重量等）。 
- <a name="carla.Landmark.unit"></a>**<font color="#f8805a">unit</font>** (_str_)  
属性 `value` 的测量单位。   
- <a name="carla.Landmark.height"></a>**<font color="#f8805a">height</font>** (_float<small> - meters</small>_)  
信号的总高度。  
- <a name="carla.Landmark.width"></a>**<font color="#f8805a">width</font>** (_float<small> - meters</small>_)  
信号的总宽度。  
- <a name="carla.Landmark.text"></a>**<font color="#f8805a">text</font>** (_str_)  
信号中的附加文本。 
- <a name="carla.Landmark.h_offset"></a>**<font color="#f8805a">h_offset</font>** (_float<small> - meters</small>_)  
信号相对于OpenDRIVE中`s`处`road_id`定义的方向偏移。 
- <a name="carla.Landmark.pitch"></a>**<font color="#f8805a">pitch</font>** (_float<small> - meters</small>_)  
信号的俯仰角 (Y-轴在虚幻引擎坐标系统).  
- <a name="carla.Landmark.roll"></a>**<font color="#f8805a">roll</font>** (_float_)  
信号的翻滚角 (X-轴在虚幻引擎坐标系统).  
- <a name="carla.Landmark.waypoint"></a>**<font color="#f8805a">waypoint</font>** (_[carla.Waypoint](#carla.Waypoint)_)  
放置在进行查询的车道和地标 `s` 处的航路点。这是地标有效的第一个航路点。
- <a name="carla.Landmark.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
仿真中地标的位置和方向。

### 方法

##### 获取器
- <a name="carla.Landmark.get_lane_validities"></a>**<font color="#7fb800">get_lane_validities</font>**(<font color="#00a6ed">**self**</font>)  
返回地标影响的车道。由于可能存在地标无效的特定车道，因此返回的是包含受影响的 __lane_id__ 范围的对列表： <small>示例：在有5条车道的道路中，其中3条不受影响：[(from_lane1,to_lane2),(from_lane4 ,to_lane5)]</small>。 
    - **返回：** _list(tuple(int))_  

---

## carla.LandmarkOrientation<a name="carla.LandmarkOrientation"></a>
用于定义道路中地标方向的辅助类。该定义并非直接从 OpenDRIVE 转换而来，而是为了便于理解而进行了转换。

### 实例变量
- <a name="carla.LandmarkOrientation.Positive"></a>**<font color="#f8805a">Positive</font>**  
地标朝向与道路几何图形定义方向相同的车辆（OpenDRIVE 中的 0 车道和负车道）。  
- <a name="carla.LandmarkOrientation.Negative"></a>**<font color="#f8805a">Negative</font>**  
地标朝向与道路几何定义相反方向行驶的车辆（OpenDRIVE中的正车道）。  
- <a name="carla.LandmarkOrientation.Both"></a>**<font color="#f8805a">Both</font>**  
都会影响沿道路两个方向行驶的车辆。 

---

## carla.LandmarkType<a name="carla.LandmarkType"></a>
Helper 类包含 [OpenDRIVE 标准](http://opendrive.org/docs/OpenDRIVEFormatSpecRev1.5M.pdf) （德国 2017）中默认国家/地区代码定义的一组常用地标类型。 __[carla.Landmark](#carla.Landmark) 不引用此类。地标类型是一个字符串，根据所使用的国家/地区代码的不同，其变化很大。此类仅通过将默认集中的一些最常用的描述为枚举来使管理它们变得更容易。 

### 实例变量
- <a name="carla.LandmarkType.Danger"></a>**<font color="#f8805a">Danger</font>**  
类型 101。 
- <a name="carla.LandmarkType.LanesMerging"></a>**<font color="#f8805a">LanesMerging</font>**  
类型 121。 
- <a name="carla.LandmarkType.CautionPedestrian"></a>**<font color="#f8805a">CautionPedestrian</font>**  
类型 133。  
- <a name="carla.LandmarkType.CautionBicycle"></a>**<font color="#f8805a">CautionBicycle</font>**  
类型 138。  
- <a name="carla.LandmarkType.LevelCrossing"></a>**<font color="#f8805a">LevelCrossing</font>**  
类型 150。  
- <a name="carla.LandmarkType.StopSign"></a>**<font color="#f8805a">StopSign</font>**  
类型 206。  
- <a name="carla.LandmarkType.YieldSign"></a>**<font color="#f8805a">YieldSign</font>**  
类型 205。 
- <a name="carla.LandmarkType.MandatoryTurnDirection"></a>**<font color="#f8805a">MandatoryTurnDirection</font>**  
类型 209。  
- <a name="carla.LandmarkType.MandatoryLeftRightDirection"></a>**<font color="#f8805a">MandatoryLeftRightDirection</font>**  
类型 211。 
- <a name="carla.LandmarkType.TwoChoiceTurnDirection"></a>**<font color="#f8805a">TwoChoiceTurnDirection</font>**  
类型 214。  
- <a name="carla.LandmarkType.Roundabout"></a>**<font color="#f8805a">Roundabout</font>**  
类型 215。  
- <a name="carla.LandmarkType.PassRightLeft"></a>**<font color="#f8805a">PassRightLeft</font>**  
类型 222。  
- <a name="carla.LandmarkType.AccessForbidden"></a>**<font color="#f8805a">AccessForbidden</font>**  
类型 250。  
- <a name="carla.LandmarkType.AccessForbiddenMotorvehicles"></a>**<font color="#f8805a">AccessForbiddenMotorvehicles</font>**  
类型 251。  
- <a name="carla.LandmarkType.AccessForbiddenTrucks"></a>**<font color="#f8805a">AccessForbiddenTrucks</font>**  
类型 253。  
- <a name="carla.LandmarkType.AccessForbiddenBicycle"></a>**<font color="#f8805a">AccessForbiddenBicycle</font>**  
类型 254。  
- <a name="carla.LandmarkType.AccessForbiddenWeight"></a>**<font color="#f8805a">AccessForbiddenWeight</font>**  
类型 263。  
- <a name="carla.LandmarkType.AccessForbiddenWidth"></a>**<font color="#f8805a">AccessForbiddenWidth</font>**  
类型 264。  
- <a name="carla.LandmarkType.AccessForbiddenHeight"></a>**<font color="#f8805a">AccessForbiddenHeight</font>**  
类型 265。  
- <a name="carla.LandmarkType.AccessForbiddenWrongDirection"></a>**<font color="#f8805a">AccessForbiddenWrongDirection</font>**  
类型 267。  
- <a name="carla.LandmarkType.ForbiddenUTurn"></a>**<font color="#f8805a">ForbiddenUTurn</font>**  
类型 272。  
- <a name="carla.LandmarkType.MaximumSpeed"></a>**<font color="#f8805a">MaximumSpeed</font>**  
类型 274。  
- <a name="carla.LandmarkType.ForbiddenOvertakingMotorvehicles"></a>**<font color="#f8805a">ForbiddenOvertakingMotorvehicles</font>**  
类型 276。  
- <a name="carla.LandmarkType.ForbiddenOvertakingTrucks"></a>**<font color="#f8805a">ForbiddenOvertakingTrucks</font>**  
类型 277。  
- <a name="carla.LandmarkType.AbsoluteNoStop"></a>**<font color="#f8805a">AbsoluteNoStop</font>**  
类型 283。  
- <a name="carla.LandmarkType.RestrictedStop"></a>**<font color="#f8805a">RestrictedStop</font>**  
类型 286。  
- <a name="carla.LandmarkType.HasWayNextIntersection"></a>**<font color="#f8805a">HasWayNextIntersection</font>**  
类型 301。  
- <a name="carla.LandmarkType.PriorityWay"></a>**<font color="#f8805a">PriorityWay</font>**  
类型 306。  
- <a name="carla.LandmarkType.PriorityWayEnd"></a>**<font color="#f8805a">PriorityWayEnd</font>**  
类型 307。  
- <a name="carla.LandmarkType.CityBegin"></a>**<font color="#f8805a">CityBegin</font>**  
类型 310。  
- <a name="carla.LandmarkType.CityEnd"></a>**<font color="#f8805a">CityEnd</font>**  
类型 311。  
- <a name="carla.LandmarkType.Highway"></a>**<font color="#f8805a">Highway</font>**  
类型 330。 
- <a name="carla.LandmarkType.DeadEnd"></a>**<font color="#f8805a">DeadEnd</font>**  
类型 357。  
- <a name="carla.LandmarkType.RecomendedSpeed"></a>**<font color="#f8805a">RecomendedSpeed</font>**  
类型 380。  
- <a name="carla.LandmarkType.RecomendedSpeedEnd"></a>**<font color="#f8805a">RecomendedSpeedEnd</font>**  
类型 381。  

---

## carla.LaneChange<a name="carla.LaneChange"></a>
定义左转、右转、双转或不转的权限的类（意味着只允许直行）。根据 OpenDRIVE 文件为每个 [carla.Waypoint](#carla.Waypoint) 存储此信息。[carla.Map.get_waypoint](#carla.Map.get_waypoint) 中的片段显示了如何使用航路点来了解允许哪些转弯。  

### 实例变量
- <a name="carla.LaneChange.NONE"></a>**<font color="#f8805a">NONE</font>**  
交通规则不允许右转或左转，只能直行。  
- <a name="carla.LaneChange.Right"></a>**<font color="#f8805a">Right</font>**  
交通规则允许右转。  
- <a name="carla.LaneChange.Left"></a>**<font color="#f8805a">Left</font>**  
交通规则允许左转。  
- <a name="carla.LaneChange.Both"></a>**<font color="#f8805a">Both</font>**  
交通规则都允许右转或左转。

---

## carla.LaneInvasionEvent<a name="carla.LaneInvasionEvent"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
为 <b>sensor.other.lane_invasion</b> 定义车道入侵的类。它仅在客户端工作，并依赖 OpenDRIVE 来提供可靠的信息。每次发生车道入侵时，传感器都会创建一个这样的情况，每个仿真步骤可能会发生多次。在 [这里](ref_sensors.md#lane-invasion-detector) 了解更多相关信息。 

### 实例变量
- <a name="carla.LaneInvasionEvent.actor"></a>**<font color="#f8805a">actor</font>** (_[carla.Actor](#carla.Actor)_)  
获取传感器所连接的参与者，即侵入另一车道的参与者。 
- <a name="carla.LaneInvasionEvent.crossed_lane_markings"></a>**<font color="#f8805a">crossed_lane_markings</font>** (_list([carla.LaneMarking](#carla.LaneMarking))_)  
传感器已穿过并检测到的车道标记列表。

### 方法

##### 魔术方法
- <a name="carla.LaneInvasionEvent.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.LaneMarking<a name="carla.LaneMarking"></a>
根据  [OpenDRIVE 1.4 标准](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf) 收集有关车道标记的所有信息的类。

### 实例变量
- <a name="carla.LaneMarking.color"></a>**<font color="#f8805a">color</font>** (_[carla.LaneMarkingColor](#carla.LaneMarkingColor)_)  
标记的实际颜色。  
- <a name="carla.LaneMarking.lane_change"></a>**<font color="#f8805a">lane_change</font>** (_[carla.LaneChange](#carla.LaneChange)_)  
跨越所述车道标记的权限。  
- <a name="carla.LaneMarking.type"></a>**<font color="#f8805a">type</font>** (_[carla.LaneMarkingType](#carla.LaneMarkingType)_)  
车道标记类型。  
- <a name="carla.LaneMarking.width"></a>**<font color="#f8805a">width</font>** (_float_)  
水平车道标记厚度。 

---

## carla.LaneMarkingColor<a name="carla.LaneMarkingColor"></a>
根据 OpenDRIVE 1.4 定义车道标记颜色的类。 

### 实例变量
- <a name="carla.LaneMarkingColor.Standard"></a>**<font color="#f8805a">Standard</font>**  
默认为白色。 
- <a name="carla.LaneMarkingColor.Blue"></a>**<font color="#f8805a">Blue</font>**  
- <a name="carla.LaneMarkingColor.Green"></a>**<font color="#f8805a">Green</font>**  
- <a name="carla.LaneMarkingColor.Red"></a>**<font color="#f8805a">Red</font>**  
- <a name="carla.LaneMarkingColor.White"></a>**<font color="#f8805a">White</font>**  
- <a name="carla.LaneMarkingColor.Yellow"></a>**<font color="#f8805a">Yellow</font>**  
- <a name="carla.LaneMarkingColor.Other"></a>**<font color="#f8805a">Other</font>**  

---

## carla.LaneMarkingType<a name="carla.LaneMarkingType"></a>
定义 OpenDRIVE 1.4 接受的车道标记类型的类。[carla.Map.get_waypoint](#carla.Map.get_waypoint) 中的片段显示了如何使用航点来检索有关相邻车道标记的信息。   <br><br> __关于双精度类型的注意事项：__ 车道标记是根据 OpenDRIVE 标准定义的，该标准确定一条线将被视为“BrokenSolid”或“SolidBroken”。每条道路都有一个中心车道标记，根据车道方向从左到右定义。其余车道标记按从中心车道到最近的道路外侧的顺序定义。

### 实例变量
- <a name="carla.LaneMarkingType.NONE"></a>**<font color="#f8805a">NONE</font>**  
- <a name="carla.LaneMarkingType.Other"></a>**<font color="#f8805a">Other</font>**  
- <a name="carla.LaneMarkingType.Broken"></a>**<font color="#f8805a">Broken</font>**  
- <a name="carla.LaneMarkingType.Solid"></a>**<font color="#f8805a">Solid</font>**  
- <a name="carla.LaneMarkingType.SolidSolid"></a>**<font color="#f8805a">SolidSolid</font>**  
- <a name="carla.LaneMarkingType.SolidBroken"></a>**<font color="#f8805a">SolidBroken</font>**  
- <a name="carla.LaneMarkingType.BrokenSolid"></a>**<font color="#f8805a">BrokenSolid</font>**  
- <a name="carla.LaneMarkingType.BrokenBroken"></a>**<font color="#f8805a">BrokenBroken</font>**  
- <a name="carla.LaneMarkingType.BottsDots"></a>**<font color="#f8805a">BottsDots</font>**  
- <a name="carla.LaneMarkingType.Grass"></a>**<font color="#f8805a">Grass</font>**  
- <a name="carla.LaneMarkingType.Curb"></a>**<font color="#f8805a">Curb</font>**  

---

## carla.LaneType<a name="carla.LaneType"></a>
定义 OpenDRIVE 1.4 接受的可能通道类型的类。该标准定义了道路信息。[carla.Map.get_waypoint](#carla.Map.get_waypoint) 中的片段利用路点来获取当前和相邻车道类型。

### 实例变量
- <a name="carla.LaneType.NONE"></a>**<font color="#f8805a">NONE</font>**  
- <a name="carla.LaneType.Driving"></a>**<font color="#f8805a">Driving</font>**  
- <a name="carla.LaneType.Stop"></a>**<font color="#f8805a">Stop</font>**  
- <a name="carla.LaneType.Shoulder"></a>**<font color="#f8805a">Shoulder</font>**  
- <a name="carla.LaneType.Biking"></a>**<font color="#f8805a">Biking</font>**  
- <a name="carla.LaneType.Sidewalk"></a>**<font color="#f8805a">Sidewalk</font>**  
- <a name="carla.LaneType.Border"></a>**<font color="#f8805a">Border</font>**  
- <a name="carla.LaneType.Restricted"></a>**<font color="#f8805a">Restricted</font>**  
- <a name="carla.LaneType.Parking"></a>**<font color="#f8805a">Parking</font>**  
- <a name="carla.LaneType.Bidirectional"></a>**<font color="#f8805a">Bidirectional</font>**  
- <a name="carla.LaneType.Median"></a>**<font color="#f8805a">Median</font>**  
- <a name="carla.LaneType.Special1"></a>**<font color="#f8805a">Special1</font>**  
- <a name="carla.LaneType.Special2"></a>**<font color="#f8805a">Special2</font>**  
- <a name="carla.LaneType.Special3"></a>**<font color="#f8805a">Special3</font>**  
- <a name="carla.LaneType.RoadWorks"></a>**<font color="#f8805a">RoadWorks</font>**  
- <a name="carla.LaneType.Tram"></a>**<font color="#f8805a">Tram</font>**  
- <a name="carla.LaneType.Rail"></a>**<font color="#f8805a">Rail</font>**  
- <a name="carla.LaneType.Entry"></a>**<font color="#f8805a">Entry</font>**  
- <a name="carla.LaneType.Exit"></a>**<font color="#f8805a">Exit</font>**  
- <a name="carla.LaneType.OffRamp"></a>**<font color="#f8805a">OffRamp</font>**  
- <a name="carla.LaneType.OnRamp"></a>**<font color="#f8805a">OnRamp</font>**  
- <a name="carla.LaneType.Any"></a>**<font color="#f8805a">Any</font>**  
除 NONE 之外的所有类型。 

---

## carla.LidarDetection<a name="carla.LidarDetection"></a>
[carla.LidarMeasurement](#carla.LidarMeasurement) 中包含的数据。其中每一个都代表云中的一个点及其位置和相关强度。 

### 实例变量
- <a name="carla.LidarDetection.point"></a>**<font color="#f8805a">point</font>** (_[carla.Location](#carla.Location)<small> - meters</small>_)  
xyz 坐标中的点。
- <a name="carla.LidarDetection.intensity"></a>**<font color="#f8805a">intensity</font>** (_float_)  
计算该点的强度作为 [0.0 , 1.0] 之间的标量值。 

### 方法

##### 魔术方法
- <a name="carla.LidarDetection.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.LidarMeasurement<a name="carla.LidarMeasurement"></a>
<small style="display:block;margin-top:-20px;">继承自 _[carla.SensorData](#carla.SensorData)_</small></br>
定义由 <b>sensor.lidar.ray_cast</b> 检索的LIDAR数据的类。这本质上是使用光线投射仿真旋转激光雷达。在 [这里](ref_sensors.md#lidar-raycast-sensor) 了解更多相关信息。 

### 实例变量
- <a name="carla.LidarMeasurement.channels"></a>**<font color="#f8805a">channels</font>** (_int_)  
发射的激光数量。 
- <a name="carla.LidarMeasurement.horizontal_angle"></a>**<font color="#f8805a">horizontal_angle</font>** (_float<small> - radians</small>_)  
测量时 LIDAR 旋转的水平角度。 
- <a name="carla.LidarMeasurement.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_bytes_)  
接收到的 4D 点列表。每个点由 [x,y,z] 坐标加上为该点计算的强度组成。 

### 方法
- <a name="carla.LidarMeasurement.save_to_disk"></a>**<font color="#7fb800">save_to_disk</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>)  
将点云作为描述来自 3D 扫描仪的数据的 <b>.ply</b> 文件保存到磁盘。生成的文件可以在[MeshLab](http://www.meshlab.net/) 中使用，MeshLab 是一个用于处理所述文件的开源系统。只需考虑到该轴可能与虚幻引擎不同，因此需要重新分配。 
    - **参数：**
        - `path` (_str_)  

##### 获取器
- <a name="carla.LidarMeasurement.get_point_count"></a>**<font color="#7fb800">get_point_count</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**channel**</font>)  
检索由此测量生成的按通道排序的点数。按通道排序可以识别每个点的原始通道。
    - **参数：**
        - `channel` (_int_)  

##### 魔术方法
- <a name="carla.LidarMeasurement.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.LidarMeasurement.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
Iterate over the [carla.LidarDetection](#carla.LidarDetection) retrieved as data.  
- <a name="carla.LidarMeasurement.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.LidarMeasurement.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**detection**=[carla.LidarDetection](#carla.LidarDetection)</font>)  
- <a name="carla.LidarMeasurement.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Light<a name="carla.Light"></a>
此类公开场景中存在的灯光（车辆灯光除外）。灯光的属性可以随意查询和改变。当仿真器进入夜间模式（太阳高度低于零）时，灯会自动打开。

### 实例变量
- <a name="carla.Light.color"></a>**<font color="#f8805a">color</font>** (_[carla.Color](#carla.Color)_)  
灯光的颜色。
- <a name="carla.Light.id"></a>**<font color="#f8805a">id</font>** (_int_)  
灯的标识符。
- <a name="carla.Light.intensity"></a>**<font color="#f8805a">intensity</font>** (_float<small> - lumens</small>_)  
光的强度。 
- <a name="carla.Light.is_on"></a>**<font color="#f8805a">is_on</font>** (_bool_)  
灯的开关。当灯亮时这是 __True__ 。当夜间模式启动时，此项设置为 __True__。
- <a name="carla.Light.location"></a>**<font color="#f8805a">location</font>** (_[carla.Location](#carla.Location)<small> - meters</small>_)  
灯光的位置。 
- <a name="carla.Light.light_group"></a>**<font color="#f8805a">light_group</font>** (_[carla.LightGroup](#carla.LightGroup)_)  
灯光所属的组。 
- <a name="carla.Light.light_state"></a>**<font color="#f8805a">light_state</font>** (_[carla.LightState](#carla.LightState)_)  
灯光的状态。总结其属性、组以及是否打开/关闭。  

### 方法
- <a name="carla.Light.turn_off"></a>**<font color="#7fb800">turn_off</font>**(<font color="#00a6ed">**self**</font>)  
关闭灯。 
- <a name="carla.Light.turn_on"></a>**<font color="#7fb800">turn_on</font>**(<font color="#00a6ed">**self**</font>)  
打开灯。

##### 设置器
- <a name="carla.Light.set_color"></a>**<font color="#7fb800">set_color</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**color**</font>)  
将灯光的颜色更改为`color`。 
    - **参数：**
        - `color` (_[carla.Color](#carla.Color)_)  
- <a name="carla.Light.set_intensity"></a>**<font color="#7fb800">set_intensity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**intensity**</font>)  
将光的强度更改为`intensity`。 
    - **参数：**
        - `intensity` (_float<small> - lumens</small>_)  
- <a name="carla.Light.set_light_group"></a>**<font color="#7fb800">set_light_group</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_group**</font>)  
将灯光更改为组`light_group`。
    - **参数：**
        - `light_group` (_[carla.LightGroup](#carla.LightGroup)_)  
- <a name="carla.Light.set_light_state"></a>**<font color="#7fb800">set_light_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_state**</font>)  
将灯光的状态更改为`light_state`。这可能会同时更改属性、分组以及打开/关闭灯。
    - **参数：**
        - `light_state` (_[carla.LightState](#carla.LightState)_)  

---

## carla.LightGroup<a name="carla.LightGroup"></a>
此类将场景中的灯光分为不同的组。这些可用组作为可用作标志的枚举值提供。

__笔记。__ 到目前为止，虽然有一个 `vehicle` 组，但车灯还不能用作[carla.Light](#carla.Light)对象。这些必须使用[carla.Vehicle](#carla.Vehicle)和 [carla.VehicleLightState](#carla.VehicleLightState) 进行管理。

### 实例变量
- <a name="carla.LightGroup.None"></a>**<font color="#f8805a">None</font>**  
所有灯。  
- <a name="carla.LightGroup.Vehicle"></a>**<font color="#f8805a">Vehicle</font>**  
- <a name="carla.LightGroup.Street"></a>**<font color="#f8805a">Street</font>**  
- <a name="carla.LightGroup.Building"></a>**<font color="#f8805a">Building</font>**  
- <a name="carla.LightGroup.Other"></a>**<font color="#f8805a">Other</font>**  

---

## carla.LightManager<a name="carla.LightManager"></a>
此类处理场景中的灯光。它的主要用途是在一次调用中获取和设置灯光组或灯光列表的状态。此类的实例可以通过 [carla.World.get_lightmanager](#carla.World.get_lightmanager)() 检索。

__笔记。__ 到目前为止，虽然有一个`vehicle`组，但车灯还不能用作[carla.Light](#carla.Light)对象。这些必须使用 [carla.Vehicle](#carla.Vehicle) 和 [carla.VehicleLightState](#carla.VehicleLightState) 进行管理。

### 方法
- <a name="carla.LightManager.is_active"></a>**<font color="#7fb800">is_active</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
返回一个列表，其中包含布尔值，说明其中的元素`lights`是否打开/关闭。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要查询的灯光列表。
    - **返回：** _list(bool)_  
- <a name="carla.LightManager.turn_off"></a>**<font color="#7fb800">turn_off</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
关闭`lights`中的所有灯。  
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要关闭的灯列表。
- <a name="carla.LightManager.turn_on"></a>**<font color="#7fb800">turn_on</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
打开 `lights` 中的所有灯。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要打开的灯列表。

##### 获取器
- <a name="carla.LightManager.get_all_lights"></a>**<font color="#7fb800">get_all_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_group**=[carla.LightGroup.None](#carla.LightGroup.None)</font>)  
返回包含特定组中的灯光的列表。默认情况下，该组是`None`。 
    - **参数：**
        - `light_group` (_[carla.LightGroup](#carla.LightGroup)_) - 用于过滤返回的灯光的组。默认为`None`。
    - **返回：** _list([carla.Light](#carla.Light))_  
- <a name="carla.LightManager.get_color"></a>**<font color="#7fb800">get_color</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
返回一个列表，其中包含`lights`中每个元素的颜色。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要查询的灯光列表。
    - **返回：** _list([carla.Color](#carla.Color))_  
    - **设置器：** _[carla.LightManager.set_color](#carla.LightManager.set_color)_  
- <a name="carla.LightManager.get_intensity"></a>**<font color="#7fb800">get_intensity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
返回一个列表，其中包含`lights`中每个元素的强度。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要查询的灯光列表。
    - **返回：** _list(float)<small> - lumens</small>_  
    - **设置器：** _[carla.LightManager.set_intensity](#carla.LightManager.set_intensity)_  
- <a name="carla.LightManager.get_light_group"></a>**<font color="#7fb800">get_light_group</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
返回一个列表，其中包含 `lights` 中每个元素的组。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要查询的灯光列表。
    - **返回：** _list([carla.LightGroup](#carla.LightGroup))_  
    - **设置器：** _[carla.LightManager.set_light_group](#carla.LightManager.set_light_group)_  
- <a name="carla.LightManager.get_light_state"></a>**<font color="#7fb800">get_light_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>)  
返回一个列表，其中包含 `lights` 中每个元素的所有属性的状态。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要查询的灯光列表。
    - **返回： ** _list([carla.LightState](#carla.LightState))_  
    - **设置器：** _[carla.LightManager.set_light_state](#carla.LightManager.set_light_state)_  
- <a name="carla.LightManager.get_turned_off_lights"></a>**<font color="#7fb800">get_turned_off_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_group**</font>)  
返回一个列表，其中包含场景中关闭的灯光（按组过滤）。 
    - **参数：**
        - `light_group` (_[carla.LightGroup](#carla.LightGroup)_) -  要查询的灯光列表。
    - **返回：** _list([carla.Light](#carla.Light))_  
- <a name="carla.LightManager.get_turned_on_lights"></a>**<font color="#7fb800">get_turned_on_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_group**</font>)  
返回一个列表，其中包含场景中打开的灯光（按组过滤）。
    - **参数：**
        - `light_group` (_[carla.LightGroup](#carla.LightGroup)_) - List of lights to be queried.  
    - **返回：** _list([carla.Light](#carla.Light))_  

##### 设置器
- <a name="carla.LightManager.set_active"></a>**<font color="#7fb800">set_active</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**active**</font>)  
打开/关闭`lights`中的元素。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要打开/关闭的灯列表。 
        - `active` (_list(bool)_) - 要应用的布尔值列表。
- <a name="carla.LightManager.set_color"></a>**<font color="#7fb800">set_color</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**color**</font>)  
`lights` 到 `color`中元素的颜色。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `color` (_[carla.Color](#carla.Color)_) - 要应用的颜色。
    - **获取器：** _[carla.LightManager.get_color](#carla.LightManager.get_color)_  
- <a name="carla.LightManager.set_colors"></a>**<font color="#7fb800">set_colors</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**colors**</font>)  
将`lights`中每个元素的颜色更改为相应的`colors`。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `colors` (_list([carla.Color](#carla.Color))_) - 要应用的颜色列表。
- <a name="carla.LightManager.set_day_night_cycle"></a>**<font color="#7fb800">set_day_night_cycle</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**active**</font>)  
所有场景灯都有昼夜循环，随着太阳的高度自动打开和关闭。这会干扰需要完全控制场景灯光的情况，因此将其设置为 __False__ 会停用它。可以通过将其设置为 __True__ 来重新激活它。
    - **参数：**
        - `active` (_bool_) - (取消)激活昼夜循环。  
- <a name="carla.LightManager.set_intensities"></a>**<font color="#7fb800">set_intensities</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**intensities**</font>)  
将`lights`中每个元素的强度更改为相应的 `intensities`。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。 
        - `intensities` (_list(float)<small> - lumens</small>_) - 要应用的强度列表。 
- <a name="carla.LightManager.set_intensity"></a>**<font color="#7fb800">set_intensity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**intensity**</font>)  
将`lights`中每个元素的强度改为`intensity`。
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `intensity` (_float<small> - lumens</small>_) -  要应用的强度。
    - **获取器：** _[carla.LightManager.get_intensity](#carla.LightManager.get_intensity)_  
- <a name="carla.LightManager.set_light_group"></a>**<font color="#7fb800">set_light_group</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**light_group**</font>)  
将`lights` 中每个元素的组更改为`light_group`。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `light_group` (_[carla.LightGroup](#carla.LightGroup)_) -  要应用的组。
    - **获取器：** _[carla.LightManager.get_light_group](#carla.LightManager.get_light_group)_  
- <a name="carla.LightManager.set_light_groups"></a>**<font color="#7fb800">set_light_groups</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**light_groups**</font>)  
将`lights`中每个元素的组更改为相应的 `light_groups`。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `light_groups` (_list([carla.LightGroup](#carla.LightGroup))_) - 要应用的组列表。
- <a name="carla.LightManager.set_light_state"></a>**<font color="#7fb800">set_light_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**light_state**</font>)  
将`lights`中每个元素的属性状态改为`light_state`。   
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `light_state` (_[carla.LightState](#carla.LightState)_) - 要应用的属性的状态。  
    - **获取器：** _[carla.LightManager.get_light_state](#carla.LightManager.get_light_state)_  
- <a name="carla.LightManager.set_light_states"></a>**<font color="#7fb800">set_light_states</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lights**</font>, <font color="#00a6ed">**light_states**</font>)  
将`lights`中每个元素的属性状态更改为相应的 `light_states`。 
    - **参数：**
        - `lights` (_list([carla.Light](#carla.Light))_) - 要更改的灯光列表。
        - `light_states` (_list([carla.LightState](#carla.LightState))_) - 要应用的属性的状态列表。
  

---

## carla.LightState<a name="carla.LightState"></a>
这个类代表除了标识符和位置之外的所有灯光变量，它们应该是静态的。使用此类可以在一次调用中管理灯光的所有参数化。

### 实例变量
- <a name="carla.LightState.intensity"></a>**<font color="#f8805a">intensity</font>** (_float<small> - lumens</small>_)  
光的强度。 
- <a name="carla.LightState.color"></a>**<font color="#f8805a">color</font>** (_[carla.Color](#carla.Color)_)  
灯光的颜色。 
- <a name="carla.LightState.group"></a>**<font color="#f8805a">group</font>** (_[carla.LightGroup](#carla.LightGroup)_)  
灯光所属的组。
- <a name="carla.LightState.active"></a>**<font color="#f8805a">active</font>** (_bool_)  
灯的开关。当灯亮时为 __True__ 。  

### 方法
- <a name="carla.LightState.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**intensity**=0.0</font>, <font color="#00a6ed">**color**=[carla.Color](#carla.Color)()</font>, <font color="#00a6ed">**group**=[carla.LightGroup.None](#carla.LightGroup.None)</font>, <font color="#00a6ed">**active**=False</font>)  
    - **参数：**
        - `intensity` (_float<small> - lumens</small>_) - 光的强度。默认为 `0.0`。
        - `color` (_[carla.Color](#carla.Color)_) - 光的颜色。默认为黑色。
        - `group` (_[carla.LightGroup](#carla.LightGroup)_) - 灯光所属的组。默认为通用组 `None`。 
        - `active` (_bool_) - 灯光开关。默认为`False`，灯关闭。 

---

## carla.Location<a name="carla.Location"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Vector3D](#carla.Vector3D)_</small></br>
代表世界上的一个地方。

### 实例变量
- <a name="carla.Location.x"></a>**<font color="#f8805a">x</font>** (_float<small> - meters</small>_)  
X 轴上从原点到点的距离。
- <a name="carla.Location.y"></a>**<font color="#f8805a">y</font>** (_float<small> - meters</small>_)  
Y 轴上从原点到点的距离。
- <a name="carla.Location.z"></a>**<font color="#f8805a">z</font>** (_float<small> - meters</small>_)  
Z 轴上从原点到点的距离。

### 方法
- <a name="carla.Location.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>, <font color="#00a6ed">**z**=0.0</font>)  
    - **参数：**
        - `x` (_float_)  
        - `y` (_float_)  
        - `z` (_float_)  
- <a name="carla.Location.distance"></a>**<font color="#7fb800">distance</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
返回从该位置到另一位置的欧几里得距离。
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)_) - 用于计算距离的另一个点。
    - **返回：** _float<small> - meters</small>_  

##### 魔术方法
- <a name="carla.Location.__abs__"></a>**<font color="#7fb800">\__abs__</font>**(<font color="#00a6ed">**self**</font>)  
返回具有 x、y 和 z 分量绝对值的位置。
    - **返回：Return:** _[carla.Location](#carla.Location)_  
- <a name="carla.Location.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
如果两个位置是空间中的同一点，则返回 __True__ 。
    - **返回：** _bool_  
- <a name="carla.Location.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
如果两个位置是空间中的不同点，则返回 __True__ 。
    - **返回：** _bool_  
- <a name="carla.Location.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
将轴的值解析为字符串。 
    - **返回：** _str_  

---

## carla.Map<a name="carla.Map"></a>
包含道路信息和航点管理的类。从描述道路的 OpenDRIVE 文件中检索数据。定义了一个查询系统，它与 [carla.Waypoint](#carla.Waypoint) 携手合作，将几何信息从 .xodr 转换为自然世界点。Carla 目前正在使用 [OpenDRIVE 1.4 标准](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf) 。

### 实例变量
- <a name="carla.Map.name"></a>**<font color="#f8805a">name</font>** (_str_)  
地图的名称。它对应于从 Carla 服务器加载的虚幻引擎中的 .umap，然后引用 .xodr 道路描述。

### 方法
- <a name="carla.Map.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**name**</font>, <font color="#00a6ed">**xodr_content**</font>)  
此类的构造函数。尽管在初始化世界时会自动生成地图，但在无渲染模式下使用此方法有助于在不运行任何 Carla 服务器的情况下使用 .xodr。
    - **参数：**
        - `name` (_str_) - 当前地图的名称。  
        - `xodr_content` (_str_) - 字符串格式的 .xodr 内容。
    - **返回：** _list([carla.Transform](#carla.Transform))_  
- <a name="carla.Map.cook_in_memory_map"></a>**<font color="#7fb800">cook_in_memory_map</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>)  
从 Carla 映射生成一个二进制文件，其中包含流量管理器使用的信息。此方法仅在地图导入过程中使用。
    - **参数：**
        - `path` (_str_) - 存储的二进制映射文件的预期位置的路径。
- <a name="carla.Map.generate_waypoints"></a>**<font color="#7fb800">generate_waypoints</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
返回一个路点列表，每个车道的路点之间都有一定的距离，并以其为中心。路点不按任何特定顺序列出。请记住，同一道路、路段和车道内距离超过2厘米的路点将具有相同的标识符。
    - **参数：**
        - `distance` (_float<small> - meters</small>_) - 路径点之间的近似距离。 
    - **返回：** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.Map.save_to_disk"></a>**<font color="#7fb800">save_to_disk</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>)  
将当前映射的 .xodr OpenDRIVE 文件保存到磁盘。
    - **参数：**
        - `path` - 保存文件的路径。  
- <a name="carla.Map.to_opendrive"></a>**<font color="#7fb800">to_opendrive</font>**(<font color="#00a6ed">**self**</font>)  
以字符串形式返回当前地图的 .xodr OpenDRIVe 文件。
    - **返回：** _str_  
- <a name="carla.Map.transform_to_geolocation"></a>**<font color="#7fb800">transform_to_geolocation</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
将仿真中一个点的给定位置 `location` 转换为 [carla.GeoLocation](#carla.GeoLocation)，他表示世界坐标系。地图的地理位置在 OpenDRIVE 标签<b><georeference></b>内定义。
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)_)  
    - **返回：** _[carla.GeoLocation](#carla.GeoLocation)_  

##### 获取器
- <a name="carla.Map.get_all_landmarks"></a>**<font color="#7fb800">get_all_landmarks</font>**(<font color="#00a6ed">**self**</font>)  
返回地图中的所有地标。使用此方法检索到的地标有一个 __null__ 航路点。
    - **返回：** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Map.get_all_landmarks_from_id"></a>**<font color="#7fb800">get_all_landmarks_from_id</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**opendrive_id**</font>)  
返回具有特定 OpenDRIVE ID 的地标。使用此方法检索的地标具有 __null__ 航路点。
    - **参数：**
        - `opendrive_id` (_string_) - 地标的 OpenDRIVE ID。  
    - **返回：** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Map.get_all_landmarks_of_type"></a>**<font color="#7fb800">get_all_landmarks_of_type</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**type**</font>)  
返回特定类型的地标。使用此方法检索的地标具有 __null__ 航路点。
    - **参数：**
        - `type` (_string_) - 地标的类型。
    - **返回：** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Map.get_crosswalks"></a>**<font color="#7fb800">get_crosswalks</font>**(<font color="#00a6ed">**self**</font>)  
以闭合多边形的形式返回包含所有人行横道区域的位置列表。重复第一个点，表示多边形的起点和终点。
    - **返回：** _list([carla.Location](#carla.Location))_  
- <a name="carla.Map.get_landmark_group"></a>**<font color="#7fb800">get_landmark_group</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**landmark**</font>)  
返回与指定地标（包括其自身）位于同一组中的地标。如果地标不属于任何组，则返回空列表。
    - **参数：**
        - `landmark` (_[carla.Landmark](#carla.Landmark)_) - 属于该组的地标。 
    - **返回：** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Map.get_spawn_points"></a>**<font color="#7fb800">get_spawn_points</font>**(<font color="#00a6ed">**self**</font>)  
返回地图创建者提出的建议列表，以用作车辆的生成点。该列表包括具有特定位置和方向的[carla.Transform](#carla.Transform) 对象。为了避免 Z 型碰撞，上述地点会稍微在空中，因此车辆在出发前会稍微下坠。
    - **返回：** _list([carla.Transform](#carla.Transform))_  
- <a name="carla.Map.get_topology"></a>**<font color="#7fb800">get_topology</font>**(<font color="#00a6ed">**self**</font>)  
返回描述 OpenDRIVE 文件拓扑的最小图的元组列表。这些元组包含位于道路起点或终点的成对路点。第一个是起点，第二个代表另一个可以到达的路终点。该图可以加载到[NetworkX](https://networkx.github.io/) 中进行使用。输出可能如下所示： <b>[(w0, w1), (w0, w2), (w1, w3), (w2, w3), (w0, w4)]</b> 。 
    - **返回：** _list(tuple([carla.Waypoint](#carla.Waypoint), [carla.Waypoint](#carla.Waypoint)))_  
- <a name="carla.Map.get_waypoint"></a>**<font color="#7fb800">get_waypoint</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**project_to_road**=True</font>, <font color="#00a6ed">**lane_type**=[carla.LaneType.Driving](#carla.LaneType.Driving)</font>)<button class="SnipetButton" id="carla.Map.get_waypoint-snipet_button">snippet &rarr;</button>  
返回可以位于精确位置或转换到最近车道中心的航路点。所述车道类型可以使用诸如 `LaneType.Driving & LaneType.Shoulder` 的标志来定义。如果未找到航路点，该方法将返回<b>None</b> ，这种情况仅在尝试检索确切位置的航路点时可能会发生。这可以轻松检查某个点是否在某条道路内，否则，它将返回相应的路径点。
    - **参数：**
        - `location` (_[carla.Location](#carla.Location)<small> - 米</small>_) - 用作 [carla.Waypoint](#carla.Waypoint) 参考的任务。  
        - `project_to_road` (_bool_) - 如果是 **True**，路径点将位于最近车道的中心。这是默认设置。如果为 **False**，路径点将恰好位于 `location`。 <b>None</b> 表示该位置不属于道路。  
        - `lane_type` (_[carla.LaneType](#carla.LaneType)_) - 将对最近车道的搜索限制为可以标记的一种或多种车道类型。
    - **返回：** _[carla.Waypoint](#carla.Waypoint)_  
- <a name="carla.Map.get_waypoint_xodr"></a>**<font color="#7fb800">get_waypoint_xodr</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**road_id**</font>, <font color="#00a6ed">**lane_id**</font>, <font color="#00a6ed">**s**</font>)  
如果传递的所有参数都正确，则返回一个航路点。否则，返回 __None__。  
    - **参数：**
        - `road_id` (_int_) - 获取路点的道路id。
        - `lane_id` (_int_) - 获取航路点的车道id。
        - `s` (_float<small> - 米</small>_) - 指定从道路起点开始的长度。 
    - **返回：** _[carla.Waypoint](#carla.Waypoint)_  

##### 魔术方法
- <a name="carla.Map.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.MapLayer<a name="carla.MapLayer"></a>
表示地图的每个可管理层的类。可以用作标志。__警告：只有“Opt”地图才能使用地图图层。__  

### 实例变量
- <a name="carla.MapLayer.NONE"></a>**<font color="#f8805a">NONE</font>**  
未选择任何图层。 
- <a name="carla.MapLayer.Buildings"></a>**<font color="#f8805a">Buildings</font>**  
- <a name="carla.MapLayer.Decals"></a>**<font color="#f8805a">Decals</font>**  
- <a name="carla.MapLayer.Foliage"></a>**<font color="#f8805a">Foliage</font>**  
- <a name="carla.MapLayer.Ground"></a>**<font color="#f8805a">Ground</font>**  
- <a name="carla.MapLayer.ParkedVehicles"></a>**<font color="#f8805a">ParkedVehicles</font>**  
- <a name="carla.MapLayer.Particles"></a>**<font color="#f8805a">Particles</font>**  
- <a name="carla.MapLayer.Props"></a>**<font color="#f8805a">Props</font>**  
- <a name="carla.MapLayer.StreetLights"></a>**<font color="#f8805a">StreetLights</font>**  
- <a name="carla.MapLayer.Walls"></a>**<font color="#f8805a">Walls</font>**  
- <a name="carla.MapLayer.All"></a>**<font color="#f8805a">All</font>**  
选定所有图层。

---

## carla.MaterialParameter<a name="carla.MaterialParameter"></a>
表示材料参数的类。并非场景中的所有对象都包含所有参数。

### 实例变量
- <a name="carla.MaterialParameter.Normal"></a>**<font color="#f8805a">Normal</font>**  
对象的法线贴图。存在于一切物体中。
- <a name="carla.MaterialParameter.Diffuse"></a>**<font color="#f8805a">Diffuse</font>**  
物体的漫反射纹理。存在于一切物体中。
- <a name="carla.MaterialParameter.AO_Roughness_Metallic_Emissive"></a>**<font color="#f8805a">AO_Roughness_Metallic_Emissive</font>**  
每个颜色通道代表材质属性的纹理（R：环境光遮挡，G：粗糙度，B：金属，A：某些物体中的发射/高度贴图）。
- <a name="carla.MaterialParameter.Emissive"></a>**<font color="#f8805a">Emissive</font>**  
发射纹理。存在于一些物体中。

---

## carla.ObstacleDetectionEvent<a name="carla.ObstacleDetectionEvent"></a>
<small style="display:block;margin-top:-20px;">继承自 _[carla.SensorData](#carla.SensorData)_</small></br>
定义 sensor.other.obstacle 障碍物数据的类。在 [这里](ref_sensors.md#obstacle-detector) 了解更多相关信息。

### 实例变量
- <a name="carla.ObstacleDetectionEvent.actor"></a>**<font color="#f8805a">actor</font>** (_[carla.Actor](#carla.Actor)_)  
传感器所连接的参与者。
- <a name="carla.ObstacleDetectionEvent.other_actor"></a>**<font color="#f8805a">other_actor</font>** (_[carla.Actor](#carla.Actor)_)  
被视为障碍的参与者或物体。 
- <a name="carla.ObstacleDetectionEvent.distance"></a>**<font color="#f8805a">distance</font>** (_float<small> - 米</small>_)  
参与者 `actor` 和其他人 `other` 之间的距离。  

### 方法

##### 魔术方法
- <a name="carla.ObstacleDetectionEvent.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.OpendriveGenerationParameters<a name="carla.OpendriveGenerationParameters"></a>
此类定义使用 OpenDRIVE 文件生成世界时使用的参数。  

### 实例变量
- <a name="carla.OpendriveGenerationParameters.vertex_distance"></a>**<font color="#f8805a">vertex_distance</font>** (_float_)  
生成的网格顶点之间的距离。__默认值为 `2.0`__。 
- <a name="carla.OpendriveGenerationParameters.max_road_length"></a>**<font color="#f8805a">max_road_length</font>** (_float_)  
单个网格部分的最大道路长度。地图的网格被分为多个部分，以避免传播问题。__默认值为 `50.0`__。 
- <a name="carla.OpendriveGenerationParameters.wall_height"></a>**<font color="#f8805a">wall_height</font>** (_float_)  
在道路边界上创建的墙壁的高度。这些可以防止车辆从道路上掉下来。__默认值为 `1.0`__。 
- <a name="carla.OpendriveGenerationParameters.additional_width"></a>**<font color="#f8805a">additional_width</font>** (_float_)  
另外还有应用的连接车道。复杂的情况往往发生在路口，稍微增加一点就可以防止车辆坠落路面。__默认值为 `0.6`__。 
- <a name="carla.OpendriveGenerationParameters.smooth_junctions"></a>**<font color="#f8805a">smooth_junctions</font>** (_bool_)  
如果为 __True__，交叉点处的网格将被平滑，以防止道路阻塞其他道路的问题。__默认为 `True`__。 
- <a name="carla.OpendriveGenerationParameters.enable_mesh_visibility"></a>**<font color="#f8805a">enable_mesh_visibility</font>** (_bool_)  
如果为 __True__，将渲染道路网格。将其设置为 False 应该会减少渲染开销。__默认为 `True`__。 
- <a name="carla.OpendriveGenerationParameters.enable_pedestrian_navigation"></a>**<font color="#f8805a">enable_pedestrian_navigation</font>** (_bool_)  
如果为 __True__，将使用 Recast 工具启用行人导航。对于非常大的地图，建议禁用此选项。__默认为 `True`__。 

---

## carla.OpticalFlowImage<a name="carla.OpticalFlowImage"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
定义表示视场中检测到的光流的二维浮点（32 位）向量的光流图像的类。矢量的分量表示物体在图像平面中的位移。每个组件输出归一化范围 [-2,2] 内的值，该范围缩放为 [-2 size，2 size]，大小是相应组件中的总分辨率。 

### 实例变量
- <a name="carla.OpticalFlowImage.fov"></a>**<font color="#f8805a">fov</font>** (_float<small> - degrees</small>_)  
图像的水平视野。 
- <a name="carla.OpticalFlowImage.height"></a>**<font color="#f8805a">height</font>** (_int_)  
图像高度（以像素为单位）。
- <a name="carla.OpticalFlowImage.width"></a>**<font color="#f8805a">width</font>** (_int_)  
图像宽度（以像素为单位）。
- <a name="carla.OpticalFlowImage.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_bytes_)  
展平像素数据数组，使用 reshape 创建图像数组。

### 方法

##### 获取器
- <a name="carla.OpticalFlowImage.get_color_coded_flow"></a>**<font color="#7fb800">get_color_coded_flow</font>**(<font color="#00a6ed">**self**</font>)  
可视化助手。将光流图像转换为 RGB 图像。
    - **返回：** _[carla.Image](#carla.Image)_  

##### 魔术方法
- <a name="carla.OpticalFlowImage.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.OpticalFlowImage.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
迭代形成图像的 [carla.OpticalFlowPixel](#carla.OpticalFlowPixel) 。
- <a name="carla.OpticalFlowImage.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.OpticalFlowImage.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**color**=[carla.Color](#carla.Color)</font>)  
- <a name="carla.OpticalFlowImage.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.OpticalFlowPixel<a name="carla.OpticalFlowPixel"></a>
定义表示光流像素的二维向量的类。

### 实例变量
- <a name="carla.OpticalFlowPixel.x"></a>**<font color="#f8805a">x</font>** (_float_)  
x 分量中的光流。
- <a name="carla.OpticalFlowPixel.y"></a>**<font color="#f8805a">y</font>** (_float_)  
y 分量中的光流。

### 方法
- <a name="carla.OpticalFlowPixel.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0</font>, <font color="#00a6ed">**y**=0</font>)  
初始化光流像素。默认为零。
    - **参数：**
        - `x` (_float_)  
        - `y` (_float_)  

##### 魔术方法
- <a name="carla.OpticalFlowPixel.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.OpticalFlowPixel](#carla.OpticalFlowPixel)</font>)  
- <a name="carla.OpticalFlowPixel.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.OpticalFlowPixel](#carla.OpticalFlowPixel)</font>)  
- <a name="carla.OpticalFlowPixel.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Osm2Odr<a name="carla.Osm2Odr"></a>
将 OpenStreetMap 地图转换为 OpenDRIVE 格式的类，以便可以在 Carla 中加载。在 [文档](tuto_G_openstreetmap.md) 中了解有关此功能的更多信息。  

### 方法
- <a name="carla.Osm2Odr.convert"></a>**<font color="#7fb800">convert</font>**(<font color="#00a6ed">**osm_file**</font>, <font color="#00a6ed">**settings**</font>)  
获取 <code>.osm</code> 文件（OpenStreetMap 格式）的内容并返回描述所述地图的 <code>.xodr</code>（OpenDRIVE 格式）的内容。传递一些参数化来进行转换。
    - **参数：**
        - `osm_file` (_str_) - 输入 OpenStreetMap 文件的内容解析为字符串。
        - `settings` (_[carla.OSM2ODRSettings](#carla.OSM2ODRSettings)_) - 转换的参数化。  
    - **返回：** _str_  

---

## carla.Osm2OdrSettings<a name="carla.Osm2OdrSettings"></a>
包含 [carla.Osm2Odr](#carla.Osm2Odr) 将使用的参数化的帮助程序类，用于将 OpenStreetMap 地图转换为 OpenDRIVE 格式。在 [文档](tuto_G_openstreetmap.md) 中了解有关此功能的更多信息。

### 实例变量
- <a name="carla.Osm2OdrSettings.use_offsets"></a>**<font color="#f8805a">use_offsets</font>** (_bool_)  
允许使用偏移量进行转换。偏移量将移动地图的原点位置。默认值为 __False__。 
- <a name="carla.Osm2OdrSettings.offset_x"></a>**<font color="#f8805a">offset_x</font>** (_float<small> - meters</small>_)  
X 轴偏移。默认值为 __0.0__。
- <a name="carla.Osm2OdrSettings.offset_y"></a>**<font color="#f8805a">offset_y</font>** (_float<small> - meters</small>_)  
Y 轴偏移。默认值为 __0.0__。 
- <a name="carla.Osm2OdrSettings.default_lane_width"></a>**<font color="#f8805a">default_lane_width</font>** (_float<small> - meters</small>_)  
生成的 XODR 地图中描述的车道宽度。默认值为 __4.0__。 
- <a name="carla.Osm2OdrSettings.elevation_layer_height"></a>**<font color="#f8805a">elevation_layer_height</font>** (_float<small> - meters</small>_)  
定义分隔两个不同 [OpenStreetMap 图层](https://wiki.openstreetmap.org/wiki/Key:layer) 的高度。默认值为 __0.0__。 
- <a name="carla.Osm2OdrSettings.center_map"></a>**<font color="#f8805a">center_map</font>** (_bool_)  
启用此选项后，地图的几何图形将发生位移，以使坐标原点与整个道路地图的边界框中心相匹配。
- <a name="carla.Osm2OdrSettings.proj_string"></a>**<font color="#f8805a">proj_string</font>** (_str_)  
定义将用于计算从地理坐标到笛卡尔坐标的投影的 [proj4](https://github.com/OSGeo/proj.4) 字符串。该字符串将写入生成的 OpenDRIVE 中，除非启用了 `use_offsets` 或 `center_map` 选项，因为这些选项会覆盖字符串中的某些定义。
- <a name="carla.Osm2OdrSettings.generate_traffic_lights"></a>**<font color="#f8805a">generate_traffic_lights</font>** (_bool_)  
指示是否在OpenDRIVE中生成红绿灯数据。 `set_traffic_light_excluded_way_types(way_types)` 定义的道路类型不会生成交通信号灯。
- <a name="carla.Osm2OdrSettings.all_junctions_with_traffic_lights"></a>**<font color="#f8805a">all_junctions_with_traffic_lights</font>** (_bool_)  
禁用时，转换器将仅从 OpenStreetMaps 数据生成交通灯数据。启用后，所有路口都会生成交通信号灯。

### 方法

##### 设置器
- <a name="carla.Osm2OdrSettings.set_osm_way_types"></a>**<font color="#7fb800">set_osm_way_types</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**way_types**</font>)  
定义将导入到 OpenDRIVE 的 OpenStreetMaps 道路类型。默认情况下，导入的道路类型为`motorway, motorway_link, trunk, trunk_link, primary, primary_link, secondary, secondary_link, tertiary, tertiary_link, unclassified, residential`。有关道路类型的完整列表，请查看 [此处](https://wiki.openstreetmap.org/wiki/Main_Page) 。
    - **参数：**
        - `way_types` (_list(str)_) - 道路类型列表。
- <a name="carla.Osm2OdrSettings.set_traffic_light_excluded_way_types"></a>**<font color="#7fb800">set_traffic_light_excluded_way_types</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**way_types**</font>)  
定义即使启用`generate_traffic_lights`也不会生成交通灯的 OpenStreetMaps 道路类型。默认情况下，排除的道路类型为 `motorway_link, primary_link, secondary_link, tertiary_link`。 
    - **参数：**
        - `way_types` (_list(str)_) - 道路类型列表。

---

## carla.RadarDetection<a name="carla.RadarDetection"></a>
[carla.RadarMeasurement](#carla.RadarMeasurement) 中包含的数据。其中每一个都代表传感器.<b>sensor.other.radar</b>  记录的云中的点之一，并包含与雷达相关的距离、角度和速度。 

### Instance Variables
- <a name="carla.RadarDetection.altitude"></a>**<font color="#f8805a">altitude</font>** (_float<small> - radians</small>_)  
Altitude angle of the detection.  
- <a name="carla.RadarDetection.azimuth"></a>**<font color="#f8805a">azimuth</font>** (_float<small> - radians</small>_)  
Azimuth angle of the detection.  
- <a name="carla.RadarDetection.depth"></a>**<font color="#f8805a">depth</font>** (_float<small> - meters</small>_)  
Distance from the sensor to the detection position.  
- <a name="carla.RadarDetection.velocity"></a>**<font color="#f8805a">velocity</font>** (_float<small> - m/s</small>_)  
The velocity of the detected object towards the sensor.  

### Methods

##### Dunder methods
- <a name="carla.RadarDetection.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.RadarMeasurement<a name="carla.RadarMeasurement"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
Class that defines and gathers the measures registered by a <b>sensor.other.radar</b>, representing a wall of points in front of the sensor with a distance, angle and velocity in relation to it. The data consists of a [carla.RadarDetection](#carla.RadarDetection) array. Learn more about this [here](ref_sensors.md#radar-sensor).  

### Instance Variables
- <a name="carla.RadarMeasurement.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_bytes_)  
The complete information of the [carla.RadarDetection](#carla.RadarDetection) the radar has registered.  

### Methods

##### Getters
- <a name="carla.RadarMeasurement.get_detection_count"></a>**<font color="#7fb800">get_detection_count</font>**(<font color="#00a6ed">**self**</font>)  
Retrieves the number of entries generated, same as **<font color="#7fb800">\__str__()</font>**.  

##### Dunder methods
- <a name="carla.RadarMeasurement.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.RadarMeasurement.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
Iterate over the [carla.RadarDetection](#carla.RadarDetection) retrieved as data.  
- <a name="carla.RadarMeasurement.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.RadarMeasurement.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**detection**=[carla.RadarDetection](#carla.RadarDetection)</font>)  
- <a name="carla.RadarMeasurement.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Rotation<a name="carla.Rotation"></a>
Class that represents a 3D rotation and therefore, an orientation in space. CARLA uses the Unreal Engine coordinates system. This is a Z-up left-handed system.  <br>
<br>The constructor method follows a specific order of declaration: `(pitch, yaw, roll)`, which corresponds to `(Y-rotation,Z-rotation,X-rotation)`.  <br> <br>![UE4_Rotation](https://d26ilriwvtzlb.cloudfront.net/8/83/BRMC_9.jpg) *Unreal Engine's coordinates system*.  

### Instance Variables
- <a name="carla.Rotation.pitch"></a>**<font color="#f8805a">pitch</font>** (_float<small> - degrees</small>_)  
Y-axis rotation angle.  
- <a name="carla.Rotation.yaw"></a>**<font color="#f8805a">yaw</font>** (_float<small> - degrees</small>_)  
Z-axis rotation angle.  
- <a name="carla.Rotation.roll"></a>**<font color="#f8805a">roll</font>** (_float<small> - degrees</small>_)  
X-axis rotation angle.  

### Methods
- <a name="carla.Rotation.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pitch**=0.0</font>, <font color="#00a6ed">**yaw**=0.0</font>, <font color="#00a6ed">**roll**=0.0</font>)  
    - **Parameters:**
        - `pitch` (_float<small> - degrees</small>_) - Y-axis rotation angle.  
        - `yaw` (_float<small> - degrees</small>_) - Z-axis rotation angle.  
        - `roll` (_float<small> - degrees</small>_) - X-axis rotation angle.  
    - **Warning:** <font color="#ED2F2F">_The declaration order is different in CARLA <code>(pitch,yaw,roll)</code>, and in the Unreal Engine Editor <code>(roll,pitch,yaw)</code>. When working in a build from source, don't mix up the axes' rotations._</font>  

##### Getters
- <a name="carla.Rotation.get_forward_vector"></a>**<font color="#7fb800">get_forward_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes the vector pointing forward according to the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Rotation.get_right_vector"></a>**<font color="#7fb800">get_right_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes the vector pointing to the right according to the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Rotation.get_up_vector"></a>**<font color="#7fb800">get_up_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes the vector pointing upwards according to the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  

##### Dunder methods
- <a name="carla.Rotation.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Rotation](#carla.Rotation)</font>)  
Returns __True__ if both rotations represent the same orientation for every axis.  
    - **Return:** _bool_  
- <a name="carla.Rotation.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Rotation](#carla.Rotation)</font>)  
Returns __True__ if both rotations represent the same orientation for every axis.  
    - **Return:** _bool_  
- <a name="carla.Rotation.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the axis' orientations to string.  

---

## carla.RssActorConstellationData<a name="carla.RssActorConstellationData"></a>
Data structure that is provided within the callback registered by RssSensor.register_actor_constellation_callback().  

### Instance Variables
- <a name="carla.RssActorConstellationData.ego_match_object"></a>**<font color="#f8805a">ego_match_object</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/structad_1_1map_1_1match_1_1Object.html">ad.map.match.Object</a>_)  
The ego map matched information.  
- <a name="carla.RssActorConstellationData.ego_route"></a>**<font color="#f8805a">ego_route</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/structad_1_1map_1_1route_1_1FullRoute.html">ad.map.route.FullRoute</a>_)  
The ego route.  
- <a name="carla.RssActorConstellationData.ego_dynamics_on_route"></a>**<font color="#f8805a">ego_dynamics_on_route</font>** (_[carla.RssEgoDynamicsOnRoute](#carla.RssEgoDynamicsOnRoute)_)  
Current ego vehicle dynamics regarding the route.  
- <a name="carla.RssActorConstellationData.other_match_object"></a>**<font color="#f8805a">other_match_object</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/structad_1_1map_1_1match_1_1Object.html">ad.map.match.Object</a>_)  
The other object's map matched information. This is only valid if 'other_actor' is not 'None'.  
- <a name="carla.RssActorConstellationData.other_actor"></a>**<font color="#f8805a">other_actor</font>** (_[carla.Actor](#carla.Actor)_)  
The other actor. This is 'None' in case of query of default parameters or articial objects of kind <a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/namespacead_1_1rss_1_1world.html#a6432f1ef8d0657b4f21ed5966aca1625">ad.rss.world.ObjectType.ArtificialObject</a> with no dedicated '[carla.Actor](#carla.Actor)' (as e.g. for the [road boundaries](ref_sensors.md#rss-sensor) at the moment).  

### Methods

##### Dunder methods
- <a name="carla.RssActorConstellationData.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.RssActorConstellationResult<a name="carla.RssActorConstellationResult"></a>
Data structure that should be returned by the callback registered by RssSensor.register_actor_constellation_callback().  

### Instance Variables
- <a name="carla.RssActorConstellationResult.rss_calculation_mode"></a>**<font color="#f8805a">rss_calculation_mode</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss_map_integration/namespacead_1_1rss_1_1map.html#adcb01232986ed83a0c540cd5d03ef495">ad.rss.map.RssMode</a>_)  
The calculation mode to be applied with the actor.  
- <a name="carla.RssActorConstellationResult.restrict_speed_limit_mode"></a>**<font color="#f8805a">restrict_speed_limit_mode</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss_map_integration/classad_1_1rss_1_1map_1_1RssSceneCreation.html#a403aae6dce3c77a8aec01dd9808dd964">ad.rss.map.RestrictSpeedLimitMode</a>_)  
The mode for restricting speed limit.  
- <a name="carla.RssActorConstellationResult.ego_vehicle_dynamics"></a>**<font color="#f8805a">ego_vehicle_dynamics</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1RssDynamics.html">ad.rss.world.RssDynamics</a>_)  
The RSS dynamics to be applied for the ego vehicle.  
- <a name="carla.RssActorConstellationResult.actor_object_type"></a>**<font color="#f8805a">actor_object_type</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/namespacead_1_1rss_1_1world.html#a6432f1ef8d0657b4f21ed5966aca1625">ad.rss.world.ObjectType</a>_)  
The RSS object type to be used for the actor.  
- <a name="carla.RssActorConstellationResult.actor_dynamics"></a>**<font color="#f8805a">actor_dynamics</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1RssDynamics.html">ad.rss.world.RssDynamics</a>_)  
The RSS dynamics to be applied for the actor.  

### Methods

##### Dunder methods
- <a name="carla.RssActorConstellationResult.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.RssEgoDynamicsOnRoute<a name="carla.RssEgoDynamicsOnRoute"></a>
Part of the data contained inside a [carla.RssResponse](#carla.RssResponse) describing the state of the vehicle. The parameters include its current dynamics, and how it is heading regarding the target route.  

### Instance Variables
- <a name="carla.RssEgoDynamicsOnRoute.ego_speed"></a>**<font color="#f8805a">ego_speed</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Speed.html">ad.physics.Speed</a>_)  
The ego vehicle's speed.  
- <a name="carla.RssEgoDynamicsOnRoute.min_stopping_distance"></a>**<font color="#f8805a">min_stopping_distance</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Distance.html">ad.physics.Distance</a>_)  
The current minimum stopping distance.  
- <a name="carla.RssEgoDynamicsOnRoute.ego_center"></a>**<font color="#f8805a">ego_center</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/structad_1_1map_1_1point_1_1ENUPoint.html">ad.map.point.ENUPoint</a>_)  
The considered enu position of the ego vehicle.  
- <a name="carla.RssEgoDynamicsOnRoute.ego_heading"></a>**<font color="#f8805a">ego_heading</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/classad_1_1map_1_1point_1_1ENUHeading.html">ad.map.point.ENUHeading</a>_)  
The considered heading of the ego vehicle.  
- <a name="carla.RssEgoDynamicsOnRoute.ego_center_within_route"></a>**<font color="#f8805a">ego_center_within_route</font>** (_bool_)  
States if the ego vehicle's center is within the route.  
- <a name="carla.RssEgoDynamicsOnRoute.crossing_border"></a>**<font color="#f8805a">crossing_border</font>** (_bool_)  
States if the vehicle is already crossing one of the lane borders.  
- <a name="carla.RssEgoDynamicsOnRoute.route_heading"></a>**<font color="#f8805a">route_heading</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/classad_1_1map_1_1point_1_1ENUHeading.html">ad.map.point.ENUHeading</a>_)  
The considered heading of the route.  
- <a name="carla.RssEgoDynamicsOnRoute.route_nominal_center"></a>**<font color="#f8805a">route_nominal_center</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/structad_1_1map_1_1point_1_1ENUPoint.html">ad.map.point.ENUPoint</a>_)  
The considered nominal center of the current route.  
- <a name="carla.RssEgoDynamicsOnRoute.heading_diff"></a>**<font color="#f8805a">heading_diff</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_map_access/apidoc/html/classad_1_1map_1_1point_1_1ENUHeading.html">ad.map.point.ENUHeading</a>_)  
The considered heading diff towards the route.  
- <a name="carla.RssEgoDynamicsOnRoute.route_speed_lat"></a>**<font color="#f8805a">route_speed_lat</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Speed.html">ad.physics.Speed</a>_)  
The ego vehicle's speed component _lat_ regarding the route.  
- <a name="carla.RssEgoDynamicsOnRoute.route_speed_lon"></a>**<font color="#f8805a">route_speed_lon</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Speed.html">ad.physics.Speed</a>_)  
The ego vehicle's speed component _lon_ regarding the route.  
- <a name="carla.RssEgoDynamicsOnRoute.route_accel_lat"></a>**<font color="#f8805a">route_accel_lat</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Acceleration.html">ad.physics.Acceleration</a>_)  
The ego vehicle's acceleration component _lat_ regarding the route.  
- <a name="carla.RssEgoDynamicsOnRoute.route_accel_lon"></a>**<font color="#f8805a">route_accel_lon</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Acceleration.html">ad.physics.Acceleration</a>_)  
The ego vehicle's acceleration component _lon_ regarding the route.  
- <a name="carla.RssEgoDynamicsOnRoute.avg_route_accel_lat"></a>**<font color="#f8805a">avg_route_accel_lat</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Acceleration.html">ad.physics.Acceleration</a>_)  
The ego vehicle's acceleration component _lat_ regarding the route smoothened by an average filter.  
- <a name="carla.RssEgoDynamicsOnRoute.avg_route_accel_lon"></a>**<font color="#f8805a">avg_route_accel_lon</font>** (_<a href="https://ad-map-access.readthedocs.io/en/latest/ad_physics/apidoc/html/classad_1_1physics_1_1Acceleration.html">ad.physics.Acceleration</a>_)  
The ego acceleration component _lon_ regarding the route smoothened by an average filter.  

### Methods

##### Dunder methods
- <a name="carla.RssEgoDynamicsOnRoute.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.RssLogLevel<a name="carla.RssLogLevel"></a>
Enum declaration used in [carla.RssSensor](#carla.RssSensor) to set the log level.  

### Instance Variables
- <a name="carla.RssLogLevel.trace"></a>**<font color="#f8805a">trace</font>**  
- <a name="carla.RssLogLevel.debug"></a>**<font color="#f8805a">debug</font>**  
- <a name="carla.RssLogLevel.info"></a>**<font color="#f8805a">info</font>**  
- <a name="carla.RssLogLevel.warn"></a>**<font color="#f8805a">warn</font>**  
- <a name="carla.RssLogLevel.err"></a>**<font color="#f8805a">err</font>**  
- <a name="carla.RssLogLevel.critical"></a>**<font color="#f8805a">critical</font>**  
- <a name="carla.RssLogLevel.off"></a>**<font color="#f8805a">off</font>**  

---

## carla.RssResponse<a name="carla.RssResponse"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
Class that contains the output of a [carla.RssSensor](#carla.RssSensor). This is the result of the RSS calculations performed for the parent vehicle of the sensor.

A [carla.RssRestrictor](#carla.RssRestrictor) will use the data to modify the [carla.VehicleControl](#carla.VehicleControl) of the vehicle.  

### Instance Variables
- <a name="carla.RssResponse.response_valid"></a>**<font color="#f8805a">response_valid</font>** (_bool_)  
States if the response is valid. It is __False__ if calculations failed or an exception occured.  
- <a name="carla.RssResponse.proper_response"></a>**<font color="#f8805a">proper_response</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1state_1_1ProperResponse.html">ad.rss.state.ProperResponse</a>_)  
The proper response that the RSS calculated for the vehicle.  
- <a name="carla.RssResponse.rss_state_snapshot"></a>**<font color="#f8805a">rss_state_snapshot</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1state_1_1RssStateSnapshot.html">ad.rss.state.RssStateSnapshot</a>_)  
Detailed RSS states at the current moment in time.  
- <a name="carla.RssResponse.ego_dynamics_on_route"></a>**<font color="#f8805a">ego_dynamics_on_route</font>** (_[carla.RssEgoDynamicsOnRoute](#carla.RssEgoDynamicsOnRoute)_)  
Current ego vehicle dynamics regarding the route.  
- <a name="carla.RssResponse.world_model"></a>**<font color="#f8805a">world_model</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1WorldModel.html">ad.rss.world.WorldModel</a>_)  
World model used for calculations.  
- <a name="carla.RssResponse.situation_snapshot"></a>**<font color="#f8805a">situation_snapshot</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1situation_1_1SituationSnapshot.html">ad.rss.situation.SituationSnapshot</a>_)  
Detailed RSS situations extracted from the world model.  

### Methods

##### Dunder methods
- <a name="carla.RssResponse.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.RssRestrictor<a name="carla.RssRestrictor"></a>
These objects apply restrictions to a [carla.VehicleControl](#carla.VehicleControl). It is part of the Carla implementation of the [C++ Library for Responsibility Sensitive Safety](https://github.com/intel/ad-rss-lib). This class works hand in hand with a [rss sensor](ref_sensors.md#rss-sensor), which provides the data of the restrictions to be applied.  

### Methods
- <a name="carla.RssRestrictor.restrict_vehicle_control"></a>**<font color="#7fb800">restrict_vehicle_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vehicle_control**</font>, <font color="#00a6ed">**proper_response**</font>, <font color="#00a6ed">**ego_dynamics_on_route**</font>, <font color="#00a6ed">**vehicle_physics**</font>)  
Applies the safety restrictions given by a [carla.RssSensor](#carla.RssSensor) to a [carla.VehicleControl](#carla.VehicleControl).  
    - **Parameters:**
        - `vehicle_control` (_[carla.VehicleControl](#carla.VehicleControl)_) - The input vehicle control to be restricted.  
        - `proper_response` (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1state_1_1ProperResponse.html">ad.rss.state.ProperResponse</a>_) - Part of the response generated by the sensor. Contains restrictions to be applied to the acceleration of the vehicle.  
        - `ego_dynamics_on_route` (_[carla.RssEgoDynamicsOnRoute](#carla.RssEgoDynamicsOnRoute)_) - Part of the response generated by the sensor. Contains dynamics and heading of the vehicle regarding its route.  
        - `vehicle_physics` (_[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_) - The current physics of the vehicle. Used to apply the restrictions properly.  
    - **Return:** _[carla.VehicleControl](#carla.VehicleControl)_  

##### Setters
- <a name="carla.RssRestrictor.set_log_level"></a>**<font color="#7fb800">set_log_level</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**log_level**</font>)  
Sets the log level.  
    - **Parameters:**
        - `log_level` (_[carla.RssLogLevel](#carla.RssLogLevel)_) - New log level.  

---

## carla.RssRoadBoundariesMode<a name="carla.RssRoadBoundariesMode"></a>
Enum declaration used in [carla.RssSensor](#carla.RssSensor) to enable or disable the [stay on road](https://intel.github.io/ad-rss-lib/ad_rss_map_integration/HandleRoadBoundaries/) feature. In summary, this feature considers the road boundaries as virtual objects. The minimum safety distance check is applied to these virtual walls, in order to make sure the vehicle does not drive off the road.  

### Instance Variables
- <a name="carla.RssRoadBoundariesMode.On"></a>**<font color="#f8805a">On</font>**  
Enables the _stay on road_ feature.  
- <a name="carla.RssRoadBoundariesMode.Off"></a>**<font color="#f8805a">Off</font>**  
Disables the _stay on road_ feature.  

---

## carla.RssSensor<a name="carla.RssSensor"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Sensor](#carla.Sensor)_</small></br>
This sensor works a bit differently than the rest. Take look at the [specific documentation](adv_rss.md), and the [rss sensor reference](ref_sensors.md#rss-sensor) to gain full understanding of it.

The RSS sensor uses world information, and a [RSS library](https://github.com/intel/ad-rss-lib) to make safety checks on a vehicle. The output retrieved by the sensor is a [carla.RssResponse](#carla.RssResponse). This will be used by a [carla.RssRestrictor](#carla.RssRestrictor) to modify a [carla.VehicleControl](#carla.VehicleControl) before applying it to a vehicle.  

### Instance Variables
- <a name="carla.RssSensor.ego_vehicle_dynamics"></a>**<font color="#f8805a">ego_vehicle_dynamics</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1RssDynamics.html">ad.rss.world.RssDynamics</a>_)  
States the [RSS parameters](https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/) that the sensor will consider for the ego vehicle if no actor constellation callback is registered.  
- <a name="carla.RssSensor.other_vehicle_dynamics"></a>**<font color="#f8805a">other_vehicle_dynamics</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1RssDynamics.html">ad.rss.world.RssDynamics</a>_)  
States the [RSS parameters](https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/) that the sensor will consider for the rest of vehicles if no actor constellation callback is registered.  
- <a name="carla.RssSensor.pedestrian_dynamics"></a>**<font color="#f8805a">pedestrian_dynamics</font>** (_<a href="https://intel.github.io/ad-rss-lib/doxygen/ad_rss/structad_1_1rss_1_1world_1_1RssDynamics.html">ad.rss.world.RssDynamics</a>_)  
States the [RSS parameters](https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/) that the sensor will consider for pedestrians if no actor constellation callback is registered.  
- <a name="carla.RssSensor.road_boundaries_mode"></a>**<font color="#f8805a">road_boundaries_mode</font>** (_[carla.RssRoadBoundariesMode](#carla.RssRoadBoundariesMode)_)  
Switches the [stay on road](https://intel.github.io/ad-rss-lib/ad_rss_map_integration/HandleRoadBoundaries/) feature. By default is __Off__.  
- <a name="carla.RssSensor.routing_targets"></a>**<font color="#f8805a">routing_targets</font>** (_vector<[carla.Transform](#carla.Transform)>_)  
The current list of targets considered to route the vehicle. If no routing targets are defined, a route is generated at random.  

### Methods
- <a name="carla.RssSensor.append_routing_target"></a>**<font color="#7fb800">append_routing_target</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**routing_target**</font>)  
Appends a new target position to the current route of the vehicle.  
    - **Parameters:**
        - `routing_target` (_[carla.Transform](#carla.Transform)_) - New target point for the route. Choose these after the intersections to force the route to take the desired turn.  
- <a name="carla.RssSensor.drop_route"></a>**<font color="#7fb800">drop_route</font>**(<font color="#00a6ed">**self**</font>)  
Discards the current route. If there are targets remaining in **<font color="#f8805a">routing_targets</font>**, creates a new route using those. Otherwise, a new route is created at random.  
- <a name="carla.RssSensor.register_actor_constellation_callback"></a>**<font color="#7fb800">register_actor_constellation_callback</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback**</font>)  
Register a callback to customize a [carla.RssActorConstellationResult](#carla.RssActorConstellationResult). By this callback the settings of RSS parameters are done per actor constellation and the settings (ego_vehicle_dynamics, other_vehicle_dynamics and pedestrian_dynamics) have no effect.  
    - **Parameters:**
        - `callback` - The function to be called whenever a RSS situation is about to be calculated.  
- <a name="carla.RssSensor.reset_routing_targets"></a>**<font color="#7fb800">reset_routing_targets</font>**(<font color="#00a6ed">**self**</font>)  
Erases the targets that have been appended to the route.  

##### Setters
- <a name="carla.RssSensor.set_log_level"></a>**<font color="#7fb800">set_log_level</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**log_level**</font>)  
Sets the log level.  
    - **Parameters:**
        - `log_level` (_[carla.RssLogLevel](#carla.RssLogLevel)_) - New log level.  
- <a name="carla.RssSensor.set_map_log_level"></a>**<font color="#7fb800">set_map_log_level</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**log_level**</font>)  
Sets the map log level.  
    - **Parameters:**
        - `log_level` (_[carla.RssLogLevel](#carla.RssLogLevel)_) - New map log level.  

##### Dunder methods
- <a name="carla.RssSensor.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.SemanticLidarDetection<a name="carla.SemanticLidarDetection"></a>
Data contained inside a [carla.SemanticLidarMeasurement](#carla.SemanticLidarMeasurement). Each of these represents one of the points in the cloud with its location, the cosine of the incident angle, index of the object hit, and its semantic tag.  

### Instance Variables
- <a name="carla.SemanticLidarDetection.point"></a>**<font color="#f8805a">point</font>** (_[carla.Location](#carla.Location)<small> - meters</small>_)  
[x,y,z] coordinates of the point.  
- <a name="carla.SemanticLidarDetection.cos_inc_angle"></a>**<font color="#f8805a">cos_inc_angle</font>** (_float_)  
Cosine of the incident angle between the ray, and the normal of the hit object.  
- <a name="carla.SemanticLidarDetection.object_idx"></a>**<font color="#f8805a">object_idx</font>** (_uint_)  
ID of the actor hit by the ray.  
- <a name="carla.SemanticLidarDetection.object_tag"></a>**<font color="#f8805a">object_tag</font>** (_uint_)  
语义标签 of the component hit by the ray.  

### Methods

##### Dunder methods
- <a name="carla.SemanticLidarDetection.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.SemanticLidarMeasurement<a name="carla.SemanticLidarMeasurement"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.SensorData](#carla.SensorData)_</small></br>
Class that defines the semantic LIDAR data retrieved by a <b>sensor.lidar.ray_cast_semantic</b>. This essentially simulates a rotating LIDAR using ray-casting. Learn more about this [here](ref_sensors.md#semanticlidar-raycast-sensor).  

### Instance Variables
- <a name="carla.SemanticLidarMeasurement.channels"></a>**<font color="#f8805a">channels</font>** (_int_)  
Number of lasers shot.  
- <a name="carla.SemanticLidarMeasurement.horizontal_angle"></a>**<font color="#f8805a">horizontal_angle</font>** (_float<small> - radians</small>_)  
Horizontal angle the LIDAR is rotated at the time of the measurement.  
- <a name="carla.SemanticLidarMeasurement.raw_data"></a>**<font color="#f8805a">raw_data</font>** (_bytes_)  
Received list of raw detection points. Each point consists of [x,y,z] coordinates plus the cosine of the incident angle, the index of the hit actor, and its semantic tag.  

### Methods
- <a name="carla.SemanticLidarMeasurement.save_to_disk"></a>**<font color="#7fb800">save_to_disk</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**path**</font>)  
Saves the point cloud to disk as a <b>.ply</b> file describing data from 3D scanners. The files generated are ready to be used within [MeshLab](http://www.meshlab.net/), an open-source system for processing said files. Just take into account that axis may differ from Unreal Engine and so, need to be reallocated.  
    - **Parameters:**
        - `path` (_str_)  

##### Getters
- <a name="carla.SemanticLidarMeasurement.get_point_count"></a>**<font color="#7fb800">get_point_count</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**channel**</font>)  
Retrieves the number of points sorted by channel that are generated by this measure. Sorting by channel allows to identify the original channel for every point.  
    - **Parameters:**
        - `channel` (_int_)  

##### Dunder methods
- <a name="carla.SemanticLidarMeasurement.__getitem__"></a>**<font color="#7fb800">\__getitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>)  
- <a name="carla.SemanticLidarMeasurement.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
Iterate over the [carla.SemanticLidarDetection](#carla.SemanticLidarDetection) retrieved as data.  
- <a name="carla.SemanticLidarMeasurement.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="carla.SemanticLidarMeasurement.__setitem__"></a>**<font color="#7fb800">\__setitem__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**pos**=int</font>, <font color="#00a6ed">**detection**=[carla.SemanticLidarDetection](#carla.SemanticLidarDetection)</font>)  
- <a name="carla.SemanticLidarMeasurement.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Sensor<a name="carla.Sensor"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Actor](#carla.Actor)_</small></br>
Sensors compound a specific family of actors quite diverse and unique. They are normally spawned as attachment/sons of a vehicle (take a look at [carla.World](#carla.World) to learn about actor spawning). Sensors are thoroughly designed to retrieve different types of data that they are listening to. The data they receive is shaped as different subclasses inherited from [carla.SensorData](#carla.SensorData) (depending on the sensor).

  Most sensors can be divided in two groups: those receiving data on every tick (cameras, point clouds and some specific sensors) and those who only receive under certain circumstances (trigger detectors). CARLA provides a specific set of sensors and their blueprint can be found in [carla.BlueprintLibrary](#carla.BlueprintLibrary). All the information on their preferences and settlement can be found [here](ref_sensors.md), but the list of those available in CARLA so far goes as follow.
  <br><b>Receive data on every tick.</b>
  - [Depth camera](ref_sensors.md#depth-camera).
  - [Gnss sensor](ref_sensors.md#gnss-sensor).
  - [IMU sensor](ref_sensors.md#imu-sensor).
  - [Lidar raycast](ref_sensors.md#lidar-raycast-sensor).
  - [SemanticLidar raycast](ref_sensors.md#semanticlidar-raycast-sensor).
  - [Radar](ref_sensors.md#radar-sensor).
  - [RGB camera](ref_sensors.md#rgb-camera).
  - [RSS sensor](ref_sensors.md#rss-sensor).
  - [Semantic Segmentation camera](ref_sensors.md#semantic-segmentation-camera).
  <br><b>Only receive data when triggered.</b>
  - [Collision detector](ref_sensors.md#collision-detector).
  - [Lane invasion detector](ref_sensors.md#lane-invasion-detector).
  - [Obstacle detector](ref_sensors.md#obstacle-detector).  

### Instance Variables
- <a name="carla.Sensor.is_listening"></a>**<font color="#f8805a">is_listening</font>** (_boolean_)  
When <b>True</b> the sensor will be waiting for data.  

### Methods
- <a name="carla.Sensor.is_listening"></a>**<font color="#7fb800">is_listening</font>**(<font color="#00a6ed">**self**</font>)  
Returns whether the sensor is in a listening state.  
- <a name="carla.Sensor.is_listening_gbuffer"></a>**<font color="#7fb800">is_listening_gbuffer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**gbuffer_id**</font>)  
Returns whether the sensor is in a listening state for a specific GBuffer texture.  
    - **Parameters:**
        - `gbuffer_id` (_[carla.GBufferTextureID](#carla.GBufferTextureID)_) - The ID of the target Unreal Engine GBuffer texture.  
- <a name="carla.Sensor.listen"></a>**<font color="#7fb800">listen</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback**</font>)<button class="SnipetButton" id="carla.Sensor.listen-snipet_button">snippet &rarr;</button>  
The function the sensor will be calling to every time a new measurement is received. This function needs for an argument containing an object type [carla.SensorData](#carla.SensorData) to work with.  
    - **Parameters:**
        - `callback` (_function_) - The called function with one argument containing the sensor data.  
- <a name="carla.Sensor.listen_to_gbuffer"></a>**<font color="#7fb800">listen_to_gbuffer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**gbuffer_id**</font>, <font color="#00a6ed">**callback**</font>)  
The function the sensor will be calling to every time the desired GBuffer texture is received.<br> This function needs for an argument containing an object type [carla.SensorData](#carla.SensorData) to work with.  
    - **Parameters:**
        - `gbuffer_id` (_[carla.GBufferTextureID](#carla.GBufferTextureID)_) - The ID of the target Unreal Engine GBuffer texture.  
        - `callback` (_function_) - The called function with one argument containing the received GBuffer texture.  
- <a name="carla.Sensor.stop"></a>**<font color="#7fb800">stop</font>**(<font color="#00a6ed">**self**</font>)  
Commands the sensor to stop listening for data.  
- <a name="carla.Sensor.stop_gbuffer"></a>**<font color="#7fb800">stop_gbuffer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**gbuffer_id**</font>)  
Commands the sensor to stop listening for the specified GBuffer texture.  
    - **Parameters:**
        - `gbuffer_id` (_[carla.GBufferTextureID](#carla.GBufferTextureID)_) - The ID of the Unreal Engine GBuffer texture.  

##### Dunder methods
- <a name="carla.Sensor.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.SensorData<a name="carla.SensorData"></a>
Base class for all the objects containing data generated by a [carla.Sensor](#carla.Sensor). This objects should be the argument of the function said sensor is listening to, in order to work with them. Each of these sensors needs for a specific type of sensor data. Hereunder is a list of the sensors and their corresponding data.<br>
  - Cameras (RGB, depth and semantic segmentation): [carla.Image](#carla.Image).<br>
  - Collision detector: [carla.CollisionEvent](#carla.CollisionEvent).<br>
  - GNSS sensor: [carla.GnssMeasurement](#carla.GnssMeasurement).<br>
  - IMU sensor: [carla.IMUMeasurement](#carla.IMUMeasurement).<br>
  - Lane invasion detector: [carla.LaneInvasionEvent](#carla.LaneInvasionEvent).<br>
  - LIDAR sensor: [carla.LidarMeasurement](#carla.LidarMeasurement).<br>
  - Obstacle detector: [carla.ObstacleDetectionEvent](#carla.ObstacleDetectionEvent).<br>
  - Radar sensor: [carla.RadarMeasurement](#carla.RadarMeasurement).<br>
  - RSS sensor: [carla.RssResponse](#carla.RssResponse).<br>
  - Semantic LIDAR sensor: [carla.SemanticLidarMeasurement](#carla.SemanticLidarMeasurement).  

### Instance Variables
- <a name="carla.SensorData.frame"></a>**<font color="#f8805a">frame</font>** (_int_)  
Frame count when the data was generated.  
- <a name="carla.SensorData.timestamp"></a>**<font color="#f8805a">timestamp</font>** (_float<small> - seconds</small>_)  
Simulation-time when the data was generated.  
- <a name="carla.SensorData.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Sensor's transform when the data was generated.  

---

## carla.TextureColor<a name="carla.TextureColor"></a>
Class representing a texture object to be uploaded to the server. Pixel format is RGBA, uint8 per channel.  

### Instance Variables
- <a name="carla.TextureColor.width"></a>**<font color="#f8805a">width</font>** (_int_)  
X-coordinate size of the texture.  
- <a name="carla.TextureColor.height"></a>**<font color="#f8805a">height</font>** (_int_)  
Y-coordinate size of the texture.  

### Methods
- <a name="carla.TextureColor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>, <font color="#00a6ed">**height**</font>)  
Initializes a the texture with a (`width`, `height`) size.  
    - **Parameters:**
        - `width` (_int_)  
        - `height` (_int_)  
- <a name="carla.TextureColor.get"></a>**<font color="#7fb800">get</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**</font>, <font color="#00a6ed">**y**</font>)  
Get the (x,y) pixel data.  
    - **Parameters:**
        - `x` (_int_)  
        - `y` (_int_)  
    - **Return:** _[carla.Color](#carla.Color)_  
- <a name="carla.TextureColor.set"></a>**<font color="#7fb800">set</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**</font>, <font color="#00a6ed">**y**</font>, <font color="#00a6ed">**value**</font>)  
Sets the (x,y) pixel data with `value`.  
    - **Parameters:**
        - `x` (_int_)  
        - `y` (_int_)  
        - `value` (_[carla.Color](#carla.Color)_)  

##### Setters
- <a name="carla.TextureColor.set_dimensions"></a>**<font color="#7fb800">set_dimensions</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>, <font color="#00a6ed">**height**</font>)  
Resizes the texture to te specified dimensions.  
    - **Parameters:**
        - `width` (_int_)  
        - `height` (_int_)  

---

## carla.TextureFloatColor<a name="carla.TextureFloatColor"></a>
Class representing a texture object to be uploaded to the server. Pixel format is RGBA, float per channel.  

### Instance Variables
- <a name="carla.TextureFloatColor.width"></a>**<font color="#f8805a">width</font>** (_int_)  
X-coordinate size of the texture.  
- <a name="carla.TextureFloatColor.height"></a>**<font color="#f8805a">height</font>** (_int_)  
Y-coordinate size of the texture.  

### Methods
- <a name="carla.TextureFloatColor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>, <font color="#00a6ed">**height**</font>)  
Initializes a the texture with a (`width`, `height`) size.  
    - **Parameters:**
        - `width` (_int_)  
        - `height` (_int_)  
- <a name="carla.TextureFloatColor.get"></a>**<font color="#7fb800">get</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**</font>, <font color="#00a6ed">**y**</font>)  
Get the (x,y) pixel data.  
    - **Parameters:**
        - `x` (_int_)  
        - `y` (_int_)  
    - **Return:** _[carla.FloatColor](#carla.FloatColor)_  
- <a name="carla.TextureFloatColor.set"></a>**<font color="#7fb800">set</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**</font>, <font color="#00a6ed">**y**</font>, <font color="#00a6ed">**value**</font>)  
Sets the (x,y) pixel data with `value`.  
    - **Parameters:**
        - `x` (_int_)  
        - `y` (_int_)  
        - `value` (_[carla.FloatColor](#carla.FloatColor)_)  

##### Setters
- <a name="carla.TextureFloatColor.set_dimensions"></a>**<font color="#7fb800">set_dimensions</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**width**</font>, <font color="#00a6ed">**height**</font>)  
Resizes the texture to te specified dimensions.  
    - **Parameters:**
        - `width` (_int_)  
        - `height` (_int_)  

---

## carla.Timestamp<a name="carla.Timestamp"></a>
Class that contains time information for simulated data. This information is automatically retrieved as part of the [carla.WorldSnapshot](#carla.WorldSnapshot) the client gets on every frame, but might also be used in many other situations such as a [carla.Sensor](#carla.Sensor) retrieveing data.  

### Instance Variables
- <a name="carla.Timestamp.frame"></a>**<font color="#f8805a">frame</font>** (_int_)  
The number of frames elapsed since the simulator was launched.  
- <a name="carla.Timestamp.elapsed_seconds"></a>**<font color="#f8805a">elapsed_seconds</font>** (_float<small> - seconds</small>_)  
Simulated seconds elapsed since the beginning of the current episode.  
- <a name="carla.Timestamp.delta_seconds"></a>**<font color="#f8805a">delta_seconds</font>** (_float<small> - seconds</small>_)  
Simulated seconds elapsed since the previous frame.  
- <a name="carla.Timestamp.platform_timestamp"></a>**<font color="#f8805a">platform_timestamp</font>** (_float<small> - seconds</small>_)  
Time register of the frame at which this measurement was taken given by the OS in seconds.  

### Methods
- <a name="carla.Timestamp.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**frame**</font>, <font color="#00a6ed">**elapsed_seconds**</font>, <font color="#00a6ed">**delta_seconds**</font>, <font color="#00a6ed">**platform_timestamp**</font>)  
    - **Parameters:**
        - `frame` (_int_)  
        - `elapsed_seconds` (_float<small> - seconds</small>_)  
        - `delta_seconds` (_float<small> - seconds</small>_)  
        - `platform_timestamp` (_float<small> - seconds</small>_)  

##### Dunder methods
- <a name="carla.Timestamp.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Timestamp](#carla.Timestamp)</font>)  
- <a name="carla.Timestamp.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Timestamp](#carla.Timestamp)</font>)  
- <a name="carla.Timestamp.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.TrafficLight<a name="carla.TrafficLight"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.TrafficSign](#carla.TrafficSign)_</small></br>
A traffic light actor, considered a specific type of traffic sign. As traffic lights will mostly appear at junctions, they belong to a group which contains the different traffic lights in it. Inside the group, traffic lights are differenciated by their pole index.
     
  Within a group the state of traffic lights is changed in a cyclic pattern: one index is chosen and it spends a few seconds in green, yellow and eventually red. The rest of the traffic lights remain frozen in red this whole time, meaning that there is a gap in the last seconds of the cycle where all the traffic lights are red. However, the state of a traffic light can be changed manually.  

### Instance Variables
- <a name="carla.TrafficLight.state"></a>**<font color="#f8805a">state</font>** (_[carla.TrafficLightState](#carla.TrafficLightState)_)  
Current state of the traffic light.  

### Methods
- <a name="carla.TrafficLight.freeze"></a>**<font color="#7fb800">freeze</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**freeze**</font>)  
Stops all the traffic lights in the scene at their current state.  
    - **Parameters:**
        - `freeze` (_bool_)  
- <a name="carla.TrafficLight.is_frozen"></a>**<font color="#7fb800">is_frozen</font>**(<font color="#00a6ed">**self**</font>)  
The client returns <b>True</b> if a traffic light is frozen according to last tick. The method does not call the simulator.  
    - **Return:** _bool_  
- <a name="carla.TrafficLight.reset_group"></a>**<font color="#7fb800">reset_group</font>**(<font color="#00a6ed">**self**</font>)  
Resets the state of the traffic lights of the group to the initial state at the start of the simulation.  
    - **Note:** <font color="#8E8E8E">_This method calls the simulator.
_</font>  

##### Getters
- <a name="carla.TrafficLight.get_affected_lane_waypoints"></a>**<font color="#7fb800">get_affected_lane_waypoints</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of waypoints indicating the positions and lanes where the traffic light is having an effect.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.TrafficLight.get_elapsed_time"></a>**<font color="#7fb800">get_elapsed_time</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the time in seconds since current light state started according to last tick. The method does not call the simulator.  
    - **Return:** _float<small> - seconds</small>_  
- <a name="carla.TrafficLight.get_green_time"></a>**<font color="#7fb800">get_green_time</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the time set for the traffic light to be green, according to last tick. The method does not call the simulator.  
    - **Return:** _float<small> - seconds</small>_  
    - **Setter:** _[carla.TrafficLight.set_green_time](#carla.TrafficLight.set_green_time)_  
- <a name="carla.TrafficLight.get_group_traffic_lights"></a>**<font color="#7fb800">get_group_traffic_lights</font>**(<font color="#00a6ed">**self**</font>)  
Returns all traffic lights in the group this one belongs to.  
    - **Return:** _list([carla.TrafficLight](#carla.TrafficLight))_  
    - **Note:** <font color="#8E8E8E">_This method calls the simulator.
_</font>  
- <a name="carla.TrafficLight.get_light_boxes"></a>**<font color="#7fb800">get_light_boxes</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of the bounding boxes encapsulating each light box of the traffic light.  
    - **Return:** _list([carla.BoundingBox](#carla.BoundingBox))_  
- <a name="carla.TrafficLight.get_opendrive_id"></a>**<font color="#7fb800">get_opendrive_id</font>**(<font color="#00a6ed">**self**</font>)  
Returns the OpenDRIVE id of this traffic light.  
    - **Return:** _str_  
- <a name="carla.TrafficLight.get_pole_index"></a>**<font color="#7fb800">get_pole_index</font>**(<font color="#00a6ed">**self**</font>)  
Returns the index of the pole that identifies it as part of the traffic light group of a junction.  
    - **Return:** _int_  
- <a name="carla.TrafficLight.get_red_time"></a>**<font color="#7fb800">get_red_time</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the time set for the traffic light to be red, according to last tick. The method does not call the simulator.  
    - **Return:** _float<small> - seconds</small>_  
    - **Setter:** _[carla.TrafficLight.set_red_time](#carla.TrafficLight.set_red_time)_  
- <a name="carla.TrafficLight.get_state"></a>**<font color="#7fb800">get_state</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the state of the traffic light according to last tick. The method does not call the simulator.  
    - **Return:** _[carla.TrafficLightState](#carla.TrafficLightState)_  
    - **Setter:** _[carla.TrafficLight.set_state](#carla.TrafficLight.set_state)_  
- <a name="carla.TrafficLight.get_stop_waypoints"></a>**<font color="#7fb800">get_stop_waypoints</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of waypoints indicating the stop position for the traffic light. These waypoints are computed from the trigger boxes of the traffic light that indicate where a vehicle should stop.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.TrafficLight.get_yellow_time"></a>**<font color="#7fb800">get_yellow_time</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the time set for the traffic light to be yellow, according to last tick. The method does not call the simulator.  
    - **Return:** _float<small> - seconds</small>_  
    - **Setter:** _[carla.TrafficLight.set_yellow_time](#carla.TrafficLight.set_yellow_time)_  

##### Setters
- <a name="carla.TrafficLight.set_green_time"></a>**<font color="#7fb800">set_green_time</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**green_time**</font>)  
    - **Parameters:**
        - `green_time` (_float<small> - seconds</small>_) - Sets a given time for the green light to be active.  
    - **Getter:** _[carla.TrafficLight.get_green_time](#carla.TrafficLight.get_green_time)_  
- <a name="carla.TrafficLight.set_red_time"></a>**<font color="#7fb800">set_red_time</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**red_time**</font>)  
Sets a given time for the red state to be active.  
    - **Parameters:**
        - `red_time` (_float<small> - seconds</small>_)  
    - **Getter:** _[carla.TrafficLight.get_red_time](#carla.TrafficLight.get_red_time)_  
- <a name="carla.TrafficLight.set_state"></a>**<font color="#7fb800">set_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**state**</font>)<button class="SnipetButton" id="carla.TrafficLight.set_state-snipet_button">snippet &rarr;</button>  
Sets a given state to a traffic light actor.  
    - **Parameters:**
        - `state` (_[carla.TrafficLightState](#carla.TrafficLightState)_)  
    - **Getter:** _[carla.TrafficLight.get_state](#carla.TrafficLight.get_state)_  
- <a name="carla.TrafficLight.set_yellow_time"></a>**<font color="#7fb800">set_yellow_time</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**yellow_time**</font>)  
Sets a given time for the yellow light to be active.  
    - **Parameters:**
        - `yellow_time` (_float<small> - seconds</small>_)  
    - **Getter:** _[carla.TrafficLight.get_yellow_time](#carla.TrafficLight.get_yellow_time)_  

##### Dunder methods
- <a name="carla.TrafficLight.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.TrafficLightState<a name="carla.TrafficLightState"></a>
交通灯的所有可能状态。这些可以在特定时间步长更改或手动更改。[carla.TrafficLight.set_state](#carla.TrafficLight.set_state) 中的片段会动态更改交通灯的状态。

### Instance Variables
- <a name="carla.TrafficLightState.Red"></a>**<font color="#f8805a">Red</font>**  
- <a name="carla.TrafficLightState.Yellow"></a>**<font color="#f8805a">Yellow</font>**  
- <a name="carla.TrafficLightState.Green"></a>**<font color="#f8805a">Green</font>**  
- <a name="carla.TrafficLightState.Off"></a>**<font color="#f8805a">Off</font>**  
- <a name="carla.TrafficLightState.Unknown"></a>**<font color="#f8805a">Unknown</font>**  

---

## carla.TrafficManager<a name="carla.TrafficManager"></a>
The traffic manager is a module built on top of the CARLA API in C++. It handles any group of vehicles set to autopilot mode to populate the simulation with realistic urban traffic conditions and give the chance to user to customize some behaviours. The architecture of the traffic manager is divided in five different goal-oriented stages and a PID controller where the information flows until eventually, a [carla.VehicleControl](#carla.VehicleControl) is applied to every vehicle registered in a traffic manager.
In order to learn more, visit the [documentation](adv_traffic_manager.md) regarding this module.  

### Methods
- <a name="carla.TrafficManager.auto_lane_change"></a>**<font color="#7fb800">auto_lane_change</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enable**</font>)  
Turns on or off lane changing behaviour for a vehicle.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The vehicle whose settings are changed.  
        - `enable` (_bool_) - __True__ is default and enables lane changes. __False__ will disable them.  
- <a name="carla.TrafficManager.collision_detection"></a>**<font color="#7fb800">collision_detection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**reference_actor**</font>, <font color="#00a6ed">**other_actor**</font>, <font color="#00a6ed">**detect_collision**</font>)  
Tunes on/off collisions between a vehicle and another specific actor. In order to ignore all other vehicles, traffic lights or walkers, use the specific __ignore__ methods described in this same section.  
    - **Parameters:**
        - `reference_actor` (_[carla.Actor](#carla.Actor)_) - Vehicle that is going to ignore collisions.  
        - `other_actor` (_[carla.Actor](#carla.Actor)_) - The actor that `reference_actor` is going to ignore collisions with.  
        - `detect_collision` (_bool_) - __True__ is default and enables collisions. __False__ will disable them.  
- <a name="carla.TrafficManager.distance_to_leading_vehicle"></a>**<font color="#7fb800">distance_to_leading_vehicle</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**distance**</font>)  
Sets the minimum distance in meters that a vehicle has to keep with the others. The distance is in meters and will affect the minimum moving distance. It is computed from front to back of the vehicle objects.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose minimum distance is being changed.  
        - `distance` (_float<small> - meters</small>_) - Meters between both vehicles.  
- <a name="carla.TrafficManager.force_lane_change"></a>**<font color="#7fb800">force_lane_change</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**direction**</font>)  
Forces a vehicle to change either to the lane on its left or right, if existing, as indicated in `direction`. This method applies the lane change no matter what, disregarding possible collisions.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle being forced to change lanes.  
        - `direction` (_bool_) - Destination lane. __True__ is the one on the right and __False__ is the left one.  
- <a name="carla.TrafficManager.global_lane_offset"></a>**<font color="#7fb800">global_lane_offset</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**offset**</font>)  
Sets a global lane offset displacement from the center line. Positive values imply a right offset while negative ones mean a left one.
Default is 0. Numbers high enough to cause the vehicle to drive through other lanes might break the controller.  
    - **Parameters:**
        - `offset` (_float_) - Lane offset displacement from the center line.  
- <a name="carla.TrafficManager.global_percentage_speed_difference"></a>**<font color="#7fb800">global_percentage_speed_difference</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**percentage**</font>)  
Sets the difference the vehicle's intended speed and its current speed limit. Speed limits can be exceeded by setting the `perc` to a negative value.
Default is 30. Exceeding a speed limit can be done using negative percentages.  
    - **Parameters:**
        - `percentage` (_float_) - Percentage difference between intended speed and the current limit.  
- <a name="carla.TrafficManager.ignore_lights_percentage"></a>**<font color="#7fb800">ignore_lights_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**perc**</font>)  
During the traffic light stage, which runs every frame, this method sets the percent chance that traffic lights will be ignored for a vehicle.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that is going to ignore traffic lights.  
        - `perc` (_float_) - Between 0 and 100. Amount of times traffic lights will be ignored.  
- <a name="carla.TrafficManager.ignore_signs_percentage"></a>**<font color="#7fb800">ignore_signs_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**perc**</font>)  
During the traffic light stage, which runs every frame, this method sets the percent chance that stop signs will be ignored for a vehicle.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that is going to ignore stop signs.  
        - `perc` (_float_) - Between 0 and 100. Amount of times stop signs will be ignored.  
- <a name="carla.TrafficManager.ignore_vehicles_percentage"></a>**<font color="#7fb800">ignore_vehicles_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**perc**</font>)  
During the collision detection stage, which runs every frame, this method sets a percent chance that collisions with another vehicle will be ignored for a vehicle.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The vehicle that is going to ignore other vehicles.  
        - `perc` (_float_) - Between 0 and 100. Amount of times collisions will be ignored.  
- <a name="carla.TrafficManager.ignore_walkers_percentage"></a>**<font color="#7fb800">ignore_walkers_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**perc**</font>)  
During the collision detection stage, which runs every frame, this method sets a percent chance that collisions with walkers will be ignored for a vehicle.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The vehicle that is going to ignore walkers on scene.  
        - `perc` (_float_) - Between 0 and 100. Amount of times collisions will be ignored.  
- <a name="carla.TrafficManager.keep_right_rule_percentage"></a>**<font color="#7fb800">keep_right_rule_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**perc**</font>)  
During the localization stage, this method sets a percent chance that vehicle will follow the *keep right* rule, and stay in the right lane.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose behaviour is being changed.  
        - `perc` (_float_) - Between 0 and 100. Amount of times the vehicle will follow the keep right rule.  
- <a name="carla.TrafficManager.random_left_lanechange_percentage"></a>**<font color="#7fb800">random_left_lanechange_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**percentage**</font>)  
Adjust probability that in each timestep the actor will perform a left lane change, dependent on lane change availability.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that you wish to query.  
        - `percentage` (_float_) - The probability of lane change in percentage units (between 0 and 100).  
- <a name="carla.TrafficManager.random_right_lanechange_percentage"></a>**<font color="#7fb800">random_right_lanechange_percentage</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**percentage**</font>)  
Adjust probability that in each timestep the actor will perform a right lane change, dependent on lane change availability.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that you wish to query.  
        - `percentage` (_float_) - The probability of lane change in percentage units (between 0 and 100).  
- <a name="carla.TrafficManager.shut_down"></a>**<font color="#7fb800">shut_down</font>**(<font color="#00a6ed">**self**</font>)  
Shuts down the traffic manager.  
- <a name="carla.TrafficManager.update_vehicle_lights"></a>**<font color="#7fb800">update_vehicle_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**do_update**</font>)  
Sets if the Traffic Manager is responsible of updating the vehicle lights, or not.
Default is __False__. The traffic manager will not change the vehicle light status of a vehicle, unless its auto_update_status is st to __True__.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose lights status is being changed.  
        - `do_update` (_bool_) - If __True__ the traffic manager will manage the vehicle lights for the specified vehicle.  
- <a name="carla.TrafficManager.vehicle_lane_offset"></a>**<font color="#7fb800">vehicle_lane_offset</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**offset**</font>)  
Sets a lane offset displacement from the center line. Positive values imply a right offset while negative ones mean a left one.
Default is 0. Numbers high enough to cause the vehicle to drive through other lanes might break the controller.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose lane offset behaviour is being changed.  
        - `offset` (_float_) - Lane offset displacement from the center line.  
- <a name="carla.TrafficManager.vehicle_percentage_speed_difference"></a>**<font color="#7fb800">vehicle_percentage_speed_difference</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**percentage**</font>)  
Sets the difference the vehicle's intended speed and its current speed limit. Speed limits can be exceeded by setting the `perc` to a negative value.
Default is 30. Exceeding a speed limit can be done using negative percentages.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose speed behaviour is being changed.  
        - `percentage` (_float_) - Percentage difference between intended speed and the current limit.  

##### Getters
- <a name="carla.TrafficManager.get_all_actions"></a>**<font color="#7fb800">get_all_actions</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>)  
Returns all known actions (i.e. road options and waypoints) that an actor controlled by the Traffic Manager will perform in its next steps.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that you wish to query.  
    - **Return:** _list of lists with each element as follows - [Road option (string e.g. 'Left', 'Right', 'Straight'), Next waypoint ([carla.Waypoint](#carla.Waypoint))]_  
- <a name="carla.TrafficManager.get_next_action"></a>**<font color="#7fb800">get_next_action</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>)  
Returns the next known road option and waypoint that an actor controlled by the Traffic Manager will follow.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that you wish to query.  
    - **Return:** _list of two elements - [Road option (string e.g. 'Left', 'Right', 'Straight'), Next waypoint ([carla.Waypoint](#carla.Waypoint))]_  
- <a name="carla.TrafficManager.get_port"></a>**<font color="#7fb800">get_port</font>**(<font color="#00a6ed">**self**</font>)  
Returns the port where the Traffic Manager is connected. If the object is a TM-Client, it will return the port of its TM-Server. Read the [documentation](#adv_traffic_manager.md#multiclient-and-multitm-management) to learn the difference.  
    - **Return:** _uint16_  

##### Setters
- <a name="carla.TrafficManager.set_boundaries_respawn_dormant_vehicles"></a>**<font color="#7fb800">set_boundaries_respawn_dormant_vehicles</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**lower_bound**=25.0</font>, <font color="#00a6ed">**upper_bound**=actor_active_distance</font>)  
Sets the upper and lower boundaries for dormant actors to be respawned near the hero vehicle.  
    - **Parameters:**
        - `lower_bound` (_float_) - The minimum distance in meters from the hero vehicle that a dormant actor will be respawned.  
        - `upper_bound` (_float_) - The maximum distance in meters from the hero vehicle that a dormant actor will be respawned.  
    - **Warning:** <font color="#ED2F2F">_The `upper_bound` cannot be higher than the `actor_active_distance`. The `lower_bound` cannot be less than 25.
_</font>  
- <a name="carla.TrafficManager.set_desired_speed"></a>**<font color="#7fb800">set_desired_speed</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**speed**</font>)  
Sets the speed of a vehicle to the specified value.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - Vehicle whose speed is being changed.  
        - `speed` (_float_) - Desired speed at which the vehicle will move.  
- <a name="carla.TrafficManager.set_global_distance_to_leading_vehicle"></a>**<font color="#7fb800">set_global_distance_to_leading_vehicle</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
Sets the minimum distance in meters that vehicles have to keep with the rest. The distance is in meters and will affect the minimum moving distance. It is computed from center to center of the vehicle objects.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - Meters between vehicles.  
- <a name="carla.TrafficManager.set_hybrid_physics_mode"></a>**<font color="#7fb800">set_hybrid_physics_mode</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=False</font>)  
Enables or disables the hybrid physics mode. In this mode, vehicle's farther than a certain radius from the ego vehicle will have their physics disabled. Computation cost will be reduced by not calculating vehicle dynamics. Vehicles will be teleported.  
    - **Parameters:**
        - `enabled` (_bool_) - If __True__, enables the hybrid physics.  
- <a name="carla.TrafficManager.set_hybrid_physics_radius"></a>**<font color="#7fb800">set_hybrid_physics_radius</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**r**=50.0</font>)  
With hybrid physics on, changes the radius of the area of influence where physics are enabled.  
    - **Parameters:**
        - `r` (_float<small> - meters</small>_) - New radius where physics are enabled.  
- <a name="carla.TrafficManager.set_osm_mode"></a>**<font color="#7fb800">set_osm_mode</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**mode_switch**=True</font>)  
Enables or disables the OSM mode. This mode allows the user to run TM in a map created with the [OSM feature](tuto_G_openstreetmap.md). These maps allow having dead-end streets. Normally, if vehicles cannot find the next waypoint, TM crashes. If OSM mode is enabled, it will show a warning, and destroy vehicles when necessary.  
    - **Parameters:**
        - `mode_switch` (_bool_) - If __True__, the OSM mode is enabled.  
- <a name="carla.TrafficManager.set_path"></a>**<font color="#7fb800">set_path</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**path**</font>)  
Sets a list of locations for a vehicle to follow while controlled by the Traffic Manager.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that must follow the given path.  
        - `path` (_list_) - The list of [carla.Locations](#carla.Locations) for the actor to follow.  
    - **Warning:** <font color="#ED2F2F">_Ensure that the road topology doesn't impede the given path.
_</font>  
- <a name="carla.TrafficManager.set_random_device_seed"></a>**<font color="#7fb800">set_random_device_seed</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**value**</font>)  
Sets a specific random seed for the Traffic Manager, thereby setting it to be deterministic.  
    - **Parameters:**
        - `value` (_int_) - Seed value for the random number generation of the Traffic Manager.  
- <a name="carla.TrafficManager.set_respawn_dormant_vehicles"></a>**<font color="#7fb800">set_respawn_dormant_vehicles</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**mode_switch**=False</font>)  
If __True__, vehicles in large maps will respawn near the hero vehicle when they become dormant. Otherwise, they will stay dormant until they are within `actor_active_distance` of the hero vehicle again.  
    - **Parameters:**
        - `mode_switch` (_bool_)  
- <a name="carla.TrafficManager.set_route"></a>**<font color="#7fb800">set_route</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**path**</font>)  
Sets a list of route instructions for a vehicle to follow while controlled by the Traffic Manager. The possible route instructions are 'Left', 'Right', 'Straight'.  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor)_) - The actor that must follow the given route instructions.  
        - `path` (_list_) - The list of route instructions (string) for the vehicle to follow.  
    - **Warning:** <font color="#ED2F2F">_Ensure that the lane topology doesn't impede the given route.
_</font>  
- <a name="carla.TrafficManager.set_synchronous_mode"></a>**<font color="#7fb800">set_synchronous_mode</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**mode_switch**=True</font>)  
Sets the Traffic Manager to [synchronous mode](adv_traffic_manager.md#synchronous-mode). In a [multiclient situation](adv_traffic_manager.md#multiclient), only the TM-Server can tick. Similarly, in a [multiTM situation](adv_traffic_manager.md#multitm), only one TM-Server must tick. Use this method in the client that does the world tick, and right after setting the world to synchronous mode, to set which TM will be the master while in sync.  
    - **Parameters:**
        - `mode_switch` (_bool_) - If __True__, the TM synchronous mode is enabled.  
    - **Warning:** <font color="#ED2F2F">_If the server is set to synchronous mode, the TM <b>must</b> be set to synchronous mode too in the same client that does the tick.
_</font>  

---

## carla.TrafficSign<a name="carla.TrafficSign"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Actor](#carla.Actor)_</small></br>
Traffic signs appearing in the simulation except for traffic lights. These have their own class inherited from this in [carla.TrafficLight](#carla.TrafficLight). Right now, speed signs, stops and yields are mainly the ones implemented, but many others are borne in mind.  

### Instance Variables
- <a name="carla.TrafficSign.trigger_volume"></a>**<font color="#f8805a">trigger_volume</font>**  
A [carla.BoundingBox](#carla.BoundingBox) situated near a traffic sign where the [carla.Actor](#carla.Actor) who is inside can know about it.  

---

## carla.Transform<a name="carla.Transform"></a>
该类定义了一个变换，即位置和旋转的组合，而不进行缩放。

### Instance Variables
- <a name="carla.Transform.location"></a>**<font color="#f8805a">location</font>** (_[carla.Location](#carla.Location)_)  
描述坐标系统中的一个点。 
- <a name="carla.Transform.rotation"></a>**<font color="#f8805a">rotation</font>** (_[carla.Rotation](#carla.Rotation)<small> - degrees (pitch, yaw, roll)</small>_)  
描述根据虚幻引擎的轴系统进行对象的旋转。 

### Methods
- <a name="carla.Transform.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**rotation**</font>)  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_)  
        - `rotation` (_[carla.Rotation](#carla.Rotation)<small> - degrees (pitch, yaw, roll)</small>_)  
- <a name="carla.Transform.transform"></a>**<font color="#7fb800">transform</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**in_point**</font>)  
Translates a 3D point from local to global coordinates using the current transformation as frame of reference.  
    - **Parameters:**
        - `in_point` (_[carla.Location](#carla.Location)_) - Location in the space to which the transformation will be applied.  
- <a name="carla.Transform.transform_vector"></a>**<font color="#7fb800">transform_vector</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**in_vector**</font>)  
Rotates a vector using the current transformation as frame of reference, without applying translation. Use this to transform, for example, a velocity.  
    - **Parameters:**
        - `in_vector` (_[carla.Vector3D](#carla.Vector3D)_) - Vector to which the transformation will be applied.  

##### Getters
- <a name="carla.Transform.get_forward_vector"></a>**<font color="#7fb800">get_forward_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes a forward vector using the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Transform.get_inverse_matrix"></a>**<font color="#7fb800">get_inverse_matrix</font>**(<font color="#00a6ed">**self**</font>)  
Computes the 4-matrix representation of the inverse transformation.  
    - **Return:** _list(list(float))_  
- <a name="carla.Transform.get_matrix"></a>**<font color="#7fb800">get_matrix</font>**(<font color="#00a6ed">**self**</font>)  
Computes the 4-matrix representation of the transformation.  
    - **Return:** _list(list(float))_  
- <a name="carla.Transform.get_right_vector"></a>**<font color="#7fb800">get_right_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes a right vector using the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Transform.get_up_vector"></a>**<font color="#7fb800">get_up_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes an up vector using the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  

##### Dunder methods
- <a name="carla.Transform.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Transform](#carla.Transform)</font>)  
Returns __True__ if both location and rotation are equal for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.Transform.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Transform](#carla.Transform)</font>)  
Returns __True__ if any location and rotation are not equal for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.Transform.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses both location and rotation to string.  
    - **Return:** _str_  

---

## carla.Vector2D<a name="carla.Vector2D"></a>
Helper class to perform 2D operations.  

### Instance Variables
- <a name="carla.Vector2D.x"></a>**<font color="#f8805a">x</font>** (_float_)  
X-axis value.  
- <a name="carla.Vector2D.y"></a>**<font color="#f8805a">y</font>** (_float_)  
Y-axis value.  

### Methods
- <a name="carla.Vector2D.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>)  
    - **Parameters:**
        - `x` (_float_)  
        - `y` (_float_)  
- <a name="carla.Vector2D.length"></a>**<font color="#7fb800">length</font>**(<font color="#00a6ed">**self**</font>)  
Computes the length of the vector.  
    - **Return:** _float_  
- <a name="carla.Vector2D.make_unit_vector"></a>**<font color="#7fb800">make_unit_vector</font>**(<font color="#00a6ed">**self**</font>)  
Returns a vector with the same direction and unitary length.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Vector2D.squared_length"></a>**<font color="#7fb800">squared_length</font>**(<font color="#00a6ed">**self**</font>)  
Computes the squared length of the vector.  
    - **Return:** _float_  

##### Dunder methods
- <a name="carla.Vector2D.__add__"></a>**<font color="#7fb800">\__add__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector2D](#carla.Vector2D)</font>)  
- <a name="carla.Vector2D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector2D](#carla.Vector2D)</font>)  
Returns __True__ if values for every axis are equal.  
    - **Return:** _bool_  
- <a name="carla.Vector2D.__mul__"></a>**<font color="#7fb800">\__mul__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector2D](#carla.Vector2D)</font>)  
- <a name="carla.Vector2D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**bool**=[carla.Vector2D](#carla.Vector2D)</font>)  
Returns __True__ if the value for any axis is different.  
    - **Return:** _bool_  
- <a name="carla.Vector2D.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Returns the axis values for the vector parsed as string.  
    - **Return:** _str_  
- <a name="carla.Vector2D.__sub__"></a>**<font color="#7fb800">\__sub__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector2D](#carla.Vector2D)</font>)  
- <a name="carla.Vector2D.__truediv__"></a>**<font color="#7fb800">\__truediv__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector2D](#carla.Vector2D)</font>)  

---

## carla.Vector3D<a name="carla.Vector3D"></a>
Helper class to perform 3D operations.  

### Instance Variables
- <a name="carla.Vector3D.x"></a>**<font color="#f8805a">x</font>** (_float_)  
X-axis value.  
- <a name="carla.Vector3D.y"></a>**<font color="#f8805a">y</font>** (_float_)  
Y-axis value.  
- <a name="carla.Vector3D.z"></a>**<font color="#f8805a">z</font>** (_float_)  
Z-axis value.  

### Methods
- <a name="carla.Vector3D.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>, <font color="#00a6ed">**z**=0.0</font>)  
    - **Parameters:**
        - `x` (_float_)  
        - `y` (_float_)  
        - `z` (_float_)  
- <a name="carla.Vector3D.cross"></a>**<font color="#7fb800">cross</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the cross product between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Vector3D.distance"></a>**<font color="#7fb800">distance</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the distance between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.distance_2d"></a>**<font color="#7fb800">distance_2d</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the 2-dimensional distance between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.distance_squared"></a>**<font color="#7fb800">distance_squared</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the squared distance between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.distance_squared_2d"></a>**<font color="#7fb800">distance_squared_2d</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the 2-dimensional squared distance between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.dot"></a>**<font color="#7fb800">dot</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the dot product between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.dot_2d"></a>**<font color="#7fb800">dot_2d</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the 2-dimensional dot product between two vectors.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  
- <a name="carla.Vector3D.length"></a>**<font color="#7fb800">length</font>**(<font color="#00a6ed">**self**</font>)  
Computes the length of the vector.  
    - **Return:** _float_  
- <a name="carla.Vector3D.make_unit_vector"></a>**<font color="#7fb800">make_unit_vector</font>**(<font color="#00a6ed">**self**</font>)  
Returns a vector with the same direction and unitary length.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Vector3D.squared_length"></a>**<font color="#7fb800">squared_length</font>**(<font color="#00a6ed">**self**</font>)  
Computes the squared length of the vector.  
    - **Return:** _float_  

##### Getters
- <a name="carla.Vector3D.get_vector_angle"></a>**<font color="#7fb800">get_vector_angle</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**vector**</font>)  
Computes the angle between a pair of 3D vectors in radians.  
    - **Parameters:**
        - `vector` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Return:** _float_  

##### Dunder methods
- <a name="carla.Vector3D.__abs__"></a>**<font color="#7fb800">\__abs__</font>**(<font color="#00a6ed">**self**</font>)  
Returns a Vector3D with the absolute value of the components x, y and z.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Vector3D.__add__"></a>**<font color="#7fb800">\__add__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
Returns __True__ if values for every axis are equal.  
    - **Return:** _bool_  
- <a name="carla.Vector3D.__mul__"></a>**<font color="#7fb800">\__mul__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
Returns __True__ if the value for any axis is different.  
    - **Return:** _bool_  
- <a name="carla.Vector3D.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Returns the axis values for the vector parsed as string.  
    - **Return:** _str_  
- <a name="carla.Vector3D.__sub__"></a>**<font color="#7fb800">\__sub__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__truediv__"></a>**<font color="#7fb800">\__truediv__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  

---

## carla.Vehicle<a name="carla.Vehicle"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Actor](#carla.Actor)_</small></br>
One of the most important groups of actors in CARLA. These include any type of vehicle from cars to trucks, motorbikes, vans, bycicles and also official vehicles such as police cars. A wide set of these actors is provided in [carla.BlueprintLibrary](#carla.BlueprintLibrary) to facilitate differente requirements. Vehicles can be either manually controlled or set to an autopilot mode that will be conducted client-side by the <b>traffic manager</b>.  

### Instance Variables
- <a name="carla.Vehicle.bounding_box"></a>**<font color="#f8805a">bounding_box</font>** (_[carla.BoundingBox](#carla.BoundingBox)_)  
Bounding box containing the geometry of the vehicle. Its location and rotation are relative to the vehicle it is attached to.  

### Methods
- <a name="carla.Vehicle.apply_ackermann_control"></a>**<font color="#7fb800">apply_ackermann_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**control**</font>)  
Applies an Ackermann control object on the next tick.  
    - **Parameters:**
        - `control` (_[carla.VehicleAckermannControl](#carla.VehicleAckermannControl)_)  
- <a name="carla.Vehicle.apply_ackermann_controller_settings"></a>**<font color="#7fb800">apply_ackermann_controller_settings</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**settings**</font>)  
Applies a new Ackermann control settings to this vehicle in the next tick.  
    - **Parameters:**
        - `settings` (_[carla.AckermannControllerSettings](#carla.AckermannControllerSettings)_)  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulator._</font>  
- <a name="carla.Vehicle.apply_control"></a>**<font color="#7fb800">apply_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**control**</font>)  
Applies a control object on the next tick, containing driving parameters such as throttle, steering or gear shifting.  
    - **Parameters:**
        - `control` (_[carla.VehicleControl](#carla.VehicleControl)_)  
- <a name="carla.Vehicle.apply_physics_control"></a>**<font color="#7fb800">apply_physics_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**physics_control**</font>)  
Applies a physics control object in the next tick containing the parameters that define the vehicle as a corporeal body. E.g.: moment of inertia, mass, drag coefficient and many more.  
    - **Parameters:**
        - `physics_control` (_[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_)  
- <a name="carla.Vehicle.close_door"></a>**<font color="#7fb800">close_door</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**door_idx**</font>)  
Close the door `door_idx` if the vehicle has it. Use [carla.VehicleDoor.All](#carla.VehicleDoor.All) to close all available doors.  
    - **Parameters:**
        - `door_idx` (_[carla.VehicleDoor](#carla.VehicleDoor)_) - door index.  
- <a name="carla.Vehicle.enable_carsim"></a>**<font color="#7fb800">enable_carsim</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**simfile_path**</font>)  
Enables the CarSim physics solver for this particular vehicle. In order for this function to work, there needs to be a valid license manager running on the server side. The control inputs are redirected to CarSim which will provide the position and orientation of the vehicle for every frame.  
    - **Parameters:**
        - `simfile_path` (_str_) - Path to the `.simfile` file with the parameters of the simulation.  
- <a name="carla.Vehicle.enable_chrono_physics"></a>**<font color="#7fb800">enable_chrono_physics</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**max_substeps**</font>, <font color="#00a6ed">**max_substep_delta_time**</font>, <font color="#00a6ed">**vehicle_json**</font>, <font color="#00a6ed">**powertrain_json**</font>, <font color="#00a6ed">**tire_json**</font>, <font color="#00a6ed">**base_json_path**</font>)  
Enables Chrono physics on a spawned vehicle.  
    - **Parameters:**
        - `max_substeps` (_int_) - Max number of Chrono substeps.  
        - `max_substep_delta_time` (_int_) - Max size of substep.  
        - `vehicle_json` (_str_) - Path to vehicle json file relative to `base_json_path`.  
        - `powertrain_json` (_str_) - Path to powertrain json file relative to `base_json_path`.  
        - `tire_json` (_str_) - Path to tire json file relative to `base_json_path`.  
        - `base_json_path` (_str_) - Path to `chrono/data/vehicle` folder. E.g., `/home/user/carla/Build/chrono-install/share/chrono/data/vehicle/` (the final `/` character is required).  
    - **Note:** <font color="#8E8E8E">_Ensure that you have started the CARLA server with the `ARGS="--chrono"` flag. You will not be able to use Chrono physics without this flag set.
_</font>  
    - **Warning:** <font color="#ED2F2F">_Collisions are not supported. When a collision is detected, physics will revert to the default CARLA physics.
_</font>  
- <a name="carla.Vehicle.is_at_traffic_light"></a>**<font color="#7fb800">is_at_traffic_light</font>**(<font color="#00a6ed">**self**</font>)  
Vehicles will be affected by a traffic light when the light is red and the vehicle is inside its bounding box. The client returns whether a traffic light is affecting this vehicle according to last tick (it does not call the simulator).  
    - **Return:** _bool_  
- <a name="carla.Vehicle.open_door"></a>**<font color="#7fb800">open_door</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**door_idx**</font>)  
Open the door `door_idx` if the vehicle has it. Use [carla.VehicleDoor.All](#carla.VehicleDoor.All) to open all available doors.  
    - **Parameters:**
        - `door_idx` (_[carla.VehicleDoor](#carla.VehicleDoor)_) - door index.  
- <a name="carla.Vehicle.show_debug_telemetry"></a>**<font color="#7fb800">show_debug_telemetry</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=True</font>)  
Enables or disables the telemetry on this vehicle. This shows information about the vehicles current state and forces applied to it in the spectator window. Only information for one vehicle can be shown so that, if you enable a second one, the previous will be automatically disabled.  
    - **Parameters:**
        - `enabled` (_bool_)  
- <a name="carla.Vehicle.use_carsim_road"></a>**<font color="#7fb800">use_carsim_road</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**</font>)  
Enables or disables the usage of CarSim vs terrain file specified in the `.simfile`. By default this option is disabled and CarSim uses unreal engine methods to process the geometry of the scene.  
    - **Parameters:**
        - `enabled` (_bool_)  

##### Getters
- <a name="carla.Vehicle.get_ackermann_controller_settings"></a>**<font color="#7fb800">get_ackermann_controller_settings</font>**(<font color="#00a6ed">**self**</font>)  
Returns the last Ackermann control settings applied to this vehicle.  
    - **Return:** _[carla.AckermannControllerSettings](#carla.AckermannControllerSettings)_  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulator to retrieve the value._</font>  
- <a name="carla.Vehicle.get_control"></a>**<font color="#7fb800">get_control</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the control applied in the last tick. The method does not call the simulator.  
    - **Return:** _[carla.VehicleControl](#carla.VehicleControl)_  
- <a name="carla.Vehicle.get_failure_state"></a>**<font color="#7fb800">get_failure_state</font>**(<font color="#00a6ed">**self**</font>)  
Vehicle have failure states, to  indicate that it is incapable of continuing its route. This function returns the vehicle's specific failure state, or in other words, the cause that resulted in it.  
    - **Return:** _[carla.VehicleFailureState](#carla.VehicleFailureState)_  
- <a name="carla.Vehicle.get_light_state"></a>**<font color="#7fb800">get_light_state</font>**(<font color="#00a6ed">**self**</font>)  
Returns a flag representing the vehicle light state, this represents which lights are active or not.  
    - **Return:** _[carla.VehicleLightState](#carla.VehicleLightState)_  
    - **Setter:** _[carla.Vehicle.set_light_state](#carla.Vehicle.set_light_state)_  
- <a name="carla.Vehicle.get_physics_control"></a>**<font color="#7fb800">get_physics_control</font>**(<font color="#00a6ed">**self**</font>)  
The simulator returns the last physics control applied to this vehicle.  
    - **Return:** _[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulator to retrieve the value._</font>  
- <a name="carla.Vehicle.get_speed_limit"></a>**<font color="#7fb800">get_speed_limit</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the speed limit affecting this vehicle according to last tick (it does not call the simulator). The speed limit is updated when passing by a speed limit signal, so a vehicle might have none right after spawning.  
    - **Return:** _float<small> - km/h</small>_  
- <a name="carla.Vehicle.get_traffic_light"></a>**<font color="#7fb800">get_traffic_light</font>**(<font color="#00a6ed">**self**</font>)  
Retrieves the traffic light actor affecting this vehicle (if any) according to last tick. The method does not call the simulator.  
    - **Return:** _[carla.TrafficLight](#carla.TrafficLight)_  
- <a name="carla.Vehicle.get_traffic_light_state"></a>**<font color="#7fb800">get_traffic_light_state</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the state of the traffic light affecting this vehicle according to last tick. The method does not call the simulator. If no traffic light is currently affecting the vehicle, returns <b>green</b>.  
    - **Return:** _[carla.TrafficLightState](#carla.TrafficLightState)_  
- <a name="carla.Vehicle.get_wheel_steer_angle"></a>**<font color="#7fb800">get_wheel_steer_angle</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**wheel_location**</font>)  
Returns the physics angle in degrees of a vehicle's wheel.  
    - **Parameters:**
        - `wheel_location` (_[carla.VehicleWheelLocation](#carla.VehicleWheelLocation)_)  
    - **Return:** _float_  
    - **Note:** <font color="#8E8E8E">_Returns the angle based on the physics of the wheel, not the visual angle.
_</font>  

##### Setters
- <a name="carla.Vehicle.set_autopilot"></a>**<font color="#7fb800">set_autopilot</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=True</font>, <font color="#00a6ed">**port**=8000</font>)  
Registers or deletes the vehicle from a Traffic Manager's list. When __True__, the Traffic Manager passed as parameter will move the vehicle around. The autopilot takes place client-side.  
    - **Parameters:**
        - `enabled` (_bool_)  
        - `port` (_uint16_) - The port of the TM-Server where the vehicle is to be registered or unlisted. If __None__ is passed, it will consider a TM at default port `8000`.  
- <a name="carla.Vehicle.set_light_state"></a>**<font color="#7fb800">set_light_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_state**</font>)  
Sets the light state of a vehicle using a flag that represents the lights that are on and off.  
    - **Parameters:**
        - `light_state` (_[carla.VehicleLightState](#carla.VehicleLightState)_)  
    - **Getter:** _[carla.Vehicle.get_light_state](#carla.Vehicle.get_light_state)_  
- <a name="carla.Vehicle.set_wheel_steer_direction"></a>**<font color="#7fb800">set_wheel_steer_direction</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**wheel_location**</font>, <font color="#00a6ed">**angle_in_deg**</font>)<button class="SnipetButton" id="carla.Vehicle.set_wheel_steer_direction-snipet_button">snippet &rarr;</button>  
Sets the angle of a vehicle's wheel visually.  
    - **Parameters:**
        - `wheel_location` (_[carla.VehicleWheelLocation](#carla.VehicleWheelLocation)_)  
        - `angle_in_deg` (_float_)  
    - **Warning:** <font color="#ED2F2F">_Does not affect the physics of the vehicle.
_</font>  

##### Dunder methods
- <a name="carla.Vehicle.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.VehicleAckermannControl<a name="carla.VehicleAckermannControl"></a>
Manages the basic movement of a vehicle using Ackermann driving controls.  

### Instance Variables
- <a name="carla.VehicleAckermannControl.steer"></a>**<font color="#f8805a">steer</font>** (_float_)  
Desired steer (rad). Positive value is to the right. Default is 0.0.  
- <a name="carla.VehicleAckermannControl.steer_speed"></a>**<font color="#f8805a">steer_speed</font>** (_float_)  
Steering velocity (rad/s). Zero steering angle velocity means change the steering angle as quickly as possible. Default is 0.0.  
- <a name="carla.VehicleAckermannControl.speed"></a>**<font color="#f8805a">speed</font>** (_float_)  
Desired speed (m/s). Default is 0.0.  
- <a name="carla.VehicleAckermannControl.acceleration"></a>**<font color="#f8805a">acceleration</font>** (_float_)  
Desired acceleration (m/s2) Default is 0.0.  
- <a name="carla.VehicleAckermannControl.jerk"></a>**<font color="#f8805a">jerk</font>** (_float_)  
Desired jerk (m/s3). Default is 0.0.  

### Methods
- <a name="carla.VehicleAckermannControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**steer**=0.0</font>, <font color="#00a6ed">**steer_speed**=0.0</font>, <font color="#00a6ed">**speed**=0.0</font>, <font color="#00a6ed">**acceleration**=0.0</font>, <font color="#00a6ed">**jerk**=0.0</font>)  
    - **Parameters:**
        - `steer` (_float_)  
        - `steer_speed` (_float_)  
        - `speed` (_float_)  
        - `acceleration` (_float_)  
        - `jerk` (_float_)  

##### Dunder methods
- <a name="carla.VehicleAckermannControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.AckermannVehicleControl](#carla.AckermannVehicleControl)</font>)  
- <a name="carla.VehicleAckermannControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.AckermannVehicleControl](#carla.AckermannVehicleControl)</font>)  
- <a name="carla.VehicleAckermannControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.VehicleControl<a name="carla.VehicleControl"></a>
Manages the basic movement of a vehicle using typical driving controls.  

### Instance Variables
- <a name="carla.VehicleControl.throttle"></a>**<font color="#f8805a">throttle</font>** (_float_)  
A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.steer"></a>**<font color="#f8805a">steer</font>** (_float_)  
A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.brake"></a>**<font color="#f8805a">brake</font>** (_float_)  
A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.hand_brake"></a>**<font color="#f8805a">hand_brake</font>** (_bool_)  
Determines whether hand brake will be used. Default is <b>False</b>.  
- <a name="carla.VehicleControl.reverse"></a>**<font color="#f8805a">reverse</font>** (_bool_)  
Determines whether the vehicle will move backwards. Default is <b>False</b>.  
- <a name="carla.VehicleControl.manual_gear_shift"></a>**<font color="#f8805a">manual_gear_shift</font>** (_bool_)  
Determines whether the vehicle will be controlled by changing gears manually. Default is <b>False</b>.  
- <a name="carla.VehicleControl.gear"></a>**<font color="#f8805a">gear</font>** (_int_)  
States which gear is the vehicle running on.  

### Methods
- <a name="carla.VehicleControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**throttle**=0.0</font>, <font color="#00a6ed">**steer**=0.0</font>, <font color="#00a6ed">**brake**=0.0</font>, <font color="#00a6ed">**hand_brake**=False</font>, <font color="#00a6ed">**reverse**=False</font>, <font color="#00a6ed">**manual_gear_shift**=False</font>, <font color="#00a6ed">**gear**=0</font>)  
    - **Parameters:**
        - `throttle` (_float_) - Scalar value between [0.0,1.0].  
        - `steer` (_float_) - Scalar value between [0.0,1.0].  
        - `brake` (_float_) - Scalar value between [0.0,1.0].  
        - `hand_brake` (_bool_)  
        - `reverse` (_bool_)  
        - `manual_gear_shift` (_bool_)  
        - `gear` (_int_)  

##### Dunder methods
- <a name="carla.VehicleControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehicleControl](#carla.VehicleControl)</font>)  
- <a name="carla.VehicleControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehicleControl](#carla.VehicleControl)</font>)  
- <a name="carla.VehicleControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.VehicleDoor<a name="carla.VehicleDoor"></a>
Possible index representing the possible doors that can be open. Notice that not all possible doors are able to open in some vehicles.  

### Instance Variables
- <a name="carla.VehicleDoor.FL"></a>**<font color="#f8805a">FL</font>**  
Front left door.  
- <a name="carla.VehicleDoor.FR"></a>**<font color="#f8805a">FR</font>**  
Front right door.  
- <a name="carla.VehicleDoor.RL"></a>**<font color="#f8805a">RL</font>**  
Back left door.  
- <a name="carla.VehicleDoor.RR"></a>**<font color="#f8805a">RR</font>**  
Back right door.  
- <a name="carla.VehicleDoor.All"></a>**<font color="#f8805a">All</font>**  
Represents all doors.  

---

## carla.VehicleFailureState<a name="carla.VehicleFailureState"></a>
Enum containing the different failure states of a vehicle, from which the it cannot recover. These are returned by __<font color="#7fb800">get_failure_state()</font>__ and only Rollover is currently implemented.  

### Instance Variables
- <a name="carla.VehicleFailureState.NONE"></a>**<font color="#f8805a">NONE</font>**  
- <a name="carla.VehicleFailureState.Rollover"></a>**<font color="#f8805a">Rollover</font>**  
- <a name="carla.VehicleFailureState.Engine"></a>**<font color="#f8805a">Engine</font>**  
- <a name="carla.VehicleFailureState.TirePuncture"></a>**<font color="#f8805a">TirePuncture</font>**  

---

## carla.VehicleLightState<a name="carla.VehicleLightState"></a>
Class that recaps the state of the lights of a vehicle, these can be used as a flags. E.g: `VehicleLightState.HighBeam & VehicleLightState.Brake` will return `True` when both are active. Lights are off by default in any situation and should be managed by the user via script. The blinkers blink automatically. _Warning: Right now, not all vehicles have been prepared to work with this functionality, this will be added to all of them in later updates_.  

### Instance Variables
- <a name="carla.VehicleLightState.NONE"></a>**<font color="#f8805a">NONE</font>**  
All lights off.  
- <a name="carla.VehicleLightState.Position"></a>**<font color="#f8805a">Position</font>**  
- <a name="carla.VehicleLightState.LowBeam"></a>**<font color="#f8805a">LowBeam</font>**  
- <a name="carla.VehicleLightState.HighBeam"></a>**<font color="#f8805a">HighBeam</font>**  
- <a name="carla.VehicleLightState.Brake"></a>**<font color="#f8805a">Brake</font>**  
- <a name="carla.VehicleLightState.RightBlinker"></a>**<font color="#f8805a">RightBlinker</font>**  
- <a name="carla.VehicleLightState.LeftBlinker"></a>**<font color="#f8805a">LeftBlinker</font>**  
- <a name="carla.VehicleLightState.Reverse"></a>**<font color="#f8805a">Reverse</font>**  
- <a name="carla.VehicleLightState.Fog"></a>**<font color="#f8805a">Fog</font>**  
- <a name="carla.VehicleLightState.Interior"></a>**<font color="#f8805a">Interior</font>**  
- <a name="carla.VehicleLightState.Special1"></a>**<font color="#f8805a">Special1</font>**  
This is reserved for certain vehicles that can have special lights, like a siren.  
- <a name="carla.VehicleLightState.Special2"></a>**<font color="#f8805a">Special2</font>**  
This is reserved for certain vehicles that can have special lights, like a siren.  
- <a name="carla.VehicleLightState.All"></a>**<font color="#f8805a">All</font>**  
All lights on.  

---

## carla.VehiclePhysicsControl<a name="carla.VehiclePhysicsControl"></a>
Summarizes the parameters that will be used to simulate a [carla.Vehicle](#carla.Vehicle) as a physical object. The specific settings for the wheels though are stipulated using [carla.WheelPhysicsControl](#carla.WheelPhysicsControl).  

### Instance Variables
- <a name="carla.VehiclePhysicsControl.torque_curve"></a>**<font color="#f8805a">torque_curve</font>** (_list([carla.Vector2D](#carla.Vector2D))_)  
Curve that indicates the torque measured in Nm for a specific RPM of the vehicle's engine.  
- <a name="carla.VehiclePhysicsControl.max_rpm"></a>**<font color="#f8805a">max_rpm</font>** (_float_)  
The maximum RPM of the vehicle's engine.  
- <a name="carla.VehiclePhysicsControl.moi"></a>**<font color="#f8805a">moi</font>** (_float<small> - kg*m<sup>2</sup></small>_)  
The moment of inertia of the vehicle's engine.  
- <a name="carla.VehiclePhysicsControl.damping_rate_full_throttle"></a>**<font color="#f8805a">damping_rate_full_throttle</font>** (_float_)  
Damping ratio when the throttle is maximum.  
- <a name="carla.VehiclePhysicsControl.damping_rate_zero_throttle_clutch_engaged"></a>**<font color="#f8805a">damping_rate_zero_throttle_clutch_engaged</font>** (_float_)  
Damping ratio when the throttle is zero with clutch engaged.  
- <a name="carla.VehiclePhysicsControl.damping_rate_zero_throttle_clutch_disengaged"></a>**<font color="#f8805a">damping_rate_zero_throttle_clutch_disengaged</font>** (_float_)  
Damping ratio when the throttle is zero with clutch disengaged.  
- <a name="carla.VehiclePhysicsControl.use_gear_autobox"></a>**<font color="#f8805a">use_gear_autobox</font>** (_bool_)  
If <b>True</b>, the vehicle will have an automatic transmission.  
- <a name="carla.VehiclePhysicsControl.gear_switch_time"></a>**<font color="#f8805a">gear_switch_time</font>** (_float<small> - seconds</small>_)  
Switching time between gears.  
- <a name="carla.VehiclePhysicsControl.clutch_strength"></a>**<font color="#f8805a">clutch_strength</font>** (_float<small> - kg*m<sup>2</sup>/s</small>_)  
Clutch strength of the vehicle.  
- <a name="carla.VehiclePhysicsControl.final_ratio"></a>**<font color="#f8805a">final_ratio</font>** (_float_)  
Fixed ratio from transmission to wheels.  
- <a name="carla.VehiclePhysicsControl.forward_gears"></a>**<font color="#f8805a">forward_gears</font>** (_list([carla.GearPhysicsControl](#carla.GearPhysicsControl))_)  
List of objects defining the vehicle's gears.  
- <a name="carla.VehiclePhysicsControl.mass"></a>**<font color="#f8805a">mass</font>** (_float<small> - kilograms</small>_)  
Mass of the vehicle.  
- <a name="carla.VehiclePhysicsControl.drag_coefficient"></a>**<font color="#f8805a">drag_coefficient</font>** (_float_)  
Drag coefficient of the vehicle's chassis.  
- <a name="carla.VehiclePhysicsControl.center_of_mass"></a>**<font color="#f8805a">center_of_mass</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - meters</small>_)  
Center of mass of the vehicle.  
- <a name="carla.VehiclePhysicsControl.steering_curve"></a>**<font color="#f8805a">steering_curve</font>** (_list([carla.Vector2D](#carla.Vector2D))_)  
Curve that indicates the maximum steering for a specific forward speed.  
- <a name="carla.VehiclePhysicsControl.use_sweep_wheel_collision"></a>**<font color="#f8805a">use_sweep_wheel_collision</font>** (_bool_)  
Enable the use of sweep for wheel collision. By default, it is disabled and it uses a simple raycast from the axis to the floor for each wheel. This option provides a better collision model in which the full volume of the wheel is checked against collisions.  
- <a name="carla.VehiclePhysicsControl.wheels"></a>**<font color="#f8805a">wheels</font>** (_list([carla.WheelPhysicsControl](#carla.WheelPhysicsControl))_)  
List of wheel physics objects. This list should have 4 elements, where index 0 corresponds to the front left wheel, index 1 corresponds to the front right wheel, index 2 corresponds to the back left wheel and index 3 corresponds to the back right wheel. For 2 wheeled vehicles, set the same values for both front and back wheels.  

### Methods
- <a name="carla.VehiclePhysicsControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**torque_curve**=[[0.0, 500.0], [5000.0, 500.0]]</font>, <font color="#00a6ed">**max_rpm**=5000.0</font>, <font color="#00a6ed">**moi**=1.0</font>, <font color="#00a6ed">**damping_rate_full_throttle**=0.15</font>, <font color="#00a6ed">**damping_rate_zero_throttle_clutch_engaged**=2.0</font>, <font color="#00a6ed">**damping_rate_zero_throttle_clutch_disengaged**=0.35</font>, <font color="#00a6ed">**use_gear_autobox**=True</font>, <font color="#00a6ed">**gear_switch_time**=0.5</font>, <font color="#00a6ed">**clutch_strength**=10.0</font>, <font color="#00a6ed">**final_ratio**=4.0</font>, <font color="#00a6ed">**forward_gears**=list()</font>, <font color="#00a6ed">**drag_coefficient**=0.3</font>, <font color="#00a6ed">**center_of_mass**=[0.0, 0.0, 0.0]</font>, <font color="#00a6ed">**steering_curve**=[[0.0, 1.0], [10.0, 0.5]]</font>, <font color="#00a6ed">**wheels**=list()</font>, <font color="#00a6ed">**use_sweep_wheel_collision**=False</font>, <font color="#00a6ed">**mass**=1000.0</font>)  
VehiclePhysicsControl constructor.  
    - **Parameters:**
        - `torque_curve` (_list([carla.Vector2D](#carla.Vector2D))_)  
        - `max_rpm` (_float_)  
        - `moi` (_float<small> - kg*m<sup>2</sup></small>_)  
        - `damping_rate_full_throttle` (_float_)  
        - `damping_rate_zero_throttle_clutch_engaged` (_float_)  
        - `damping_rate_zero_throttle_clutch_disengaged` (_float_)  
        - `use_gear_autobox` (_bool_)  
        - `gear_switch_time` (_float<small> - seconds</small>_)  
        - `clutch_strength` (_float<small> - kg*m<sup>2</sup>/s</small>_)  
        - `final_ratio` (_float_)  
        - `forward_gears` (_list([carla.GearPhysicsControl](#carla.GearPhysicsControl))_)  
        - `drag_coefficient` (_float_)  
        - `center_of_mass` (_[carla.Vector3D](#carla.Vector3D)_)  
        - `steering_curve` (_[carla.Vector2D](#carla.Vector2D)_)  
        - `wheels` (_list([carla.WheelPhysicsControl](#carla.WheelPhysicsControl))_)  
        - `use_sweep_wheel_collision` (_bool_)  
        - `mass` (_float<small> - kilograms</small>_)  

##### Dunder methods
- <a name="carla.VehiclePhysicsControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)</font>)  
- <a name="carla.VehiclePhysicsControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)</font>)  
- <a name="carla.VehiclePhysicsControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.VehicleWheelLocation<a name="carla.VehicleWheelLocation"></a>
`enum` representing the position of each wheel on a vehicle.  Used to identify the target wheel when setting an angle in [carla.Vehicle.set_wheel_steer_direction](#carla.Vehicle.set_wheel_steer_direction) or [carla.Vehicle.get_wheel_steer_angle](#carla.Vehicle.get_wheel_steer_angle).  

### Instance Variables
- <a name="carla.VehicleWheelLocation.FL_Wheel"></a>**<font color="#f8805a">FL_Wheel</font>**  
Front left wheel of a 4 wheeled vehicle.  
- <a name="carla.VehicleWheelLocation.FR_Wheel"></a>**<font color="#f8805a">FR_Wheel</font>**  
Front right wheel of a 4 wheeled vehicle.  
- <a name="carla.VehicleWheelLocation.BL_Wheel"></a>**<font color="#f8805a">BL_Wheel</font>**  
Back left wheel of a 4 wheeled vehicle.  
- <a name="carla.VehicleWheelLocation.BR_Wheel"></a>**<font color="#f8805a">BR_Wheel</font>**  
Back right wheel of a 4 wheeled vehicle.  
- <a name="carla.VehicleWheelLocation.Front_Wheel"></a>**<font color="#f8805a">Front_Wheel</font>**  
Front wheel of a 2 wheeled vehicle.  
- <a name="carla.VehicleWheelLocation.Back_Wheel"></a>**<font color="#f8805a">Back_Wheel</font>**  
Back wheel of a 2 wheeled vehicle.  

---

## carla.Walker<a name="carla.Walker"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Actor](#carla.Actor)_</small></br>
This class inherits from the [carla.Actor](#carla.Actor) and defines pedestrians in the simulation. Walkers are a special type of actor that can be controlled either by an AI ([carla.WalkerAIController](#carla.WalkerAIController)) or manually via script, using a series of [carla.WalkerControl](#carla.WalkerControl) to move these and their skeletons.  

### Methods
- <a name="carla.Walker.apply_control"></a>**<font color="#7fb800">apply_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**control**</font>)  
On the next tick, the control will move the walker in a certain direction with a certain speed. Jumps can be commanded too.  
    - **Parameters:**
        - `control` (_[carla.WalkerControl](#carla.WalkerControl)_)  
- <a name="carla.Walker.blend_pose"></a>**<font color="#7fb800">blend_pose</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blend_value**</font>)  
Set the blending value of the custom pose with the animation. The values can be:
  - 0: will show only the animation
  - 1: will show only the custom pose (set by the user with set_bones())
  - any other: will interpolate all the bone positions between animation and the custom pose.  
    - **Parameters:**
        - `blend_value` (_float<small> - value from 0 to 1 with the blend percentage</small>_)  
- <a name="carla.Walker.hide_pose"></a>**<font color="#7fb800">hide_pose</font>**(<font color="#00a6ed">**self**</font>)  
Hide the custom pose and show the animation (same as calling blend_pose(0)).  
- <a name="carla.Walker.show_pose"></a>**<font color="#7fb800">show_pose</font>**(<font color="#00a6ed">**self**</font>)  
Show the custom pose and hide the animation (same as calling blend_pose(1)).  

##### Getters
- <a name="carla.Walker.get_bones"></a>**<font color="#7fb800">get_bones</font>**(<font color="#00a6ed">**self**</font>)  
Return the structure with all the bone transformations from the actor. For each bone, we get the name and its transform in three different spaces:
  - name: bone name
  - world: transform in world coordinates
  - component: transform based on the pivot of the actor
  - relative: transform based on the bone parent.  
    - **Return:** _[carla.WalkerBoneControlOut](#carla.WalkerBoneControlOut)_  
    - **Setter:** _[carla.Walker.set_bones](#carla.Walker.set_bones)_  
- <a name="carla.Walker.get_control"></a>**<font color="#7fb800">get_control</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the control applied to this walker during last tick. The method does not call the simulator.  
    - **Return:** _[carla.WalkerControl](#carla.WalkerControl)_  
- <a name="carla.Walker.get_pose_from_animation"></a>**<font color="#7fb800">get_pose_from_animation</font>**(<font color="#00a6ed">**self**</font>)  
Make a copy of the current animation frame as the custom pose. Initially the custom pose is the neutral pedestrian pose.  

##### Setters
- <a name="carla.Walker.set_bones"></a>**<font color="#7fb800">set_bones</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**bones**</font>)  
Set the bones of the actor. For each bone we want to set we use a relative transform. Only the bones in this list will be set. For each bone you need to setup this info:
  - name: bone name
  - relative: transform based on the bone parent.  
    - **Parameters:**
        - `bones` (_[carla.WalkerBoneControlIn](#carla.WalkerBoneControlIn)<small> - list of pairs (bone_name, transform) for the bones that we want to set</small>_)  
    - **Getter:** _[carla.Walker.get_bones](#carla.Walker.get_bones)_  

##### Dunder methods
- <a name="carla.Walker.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WalkerAIController<a name="carla.WalkerAIController"></a>
<small style="display:block;margin-top:-20px;">Inherited from _[carla.Actor](#carla.Actor)_</small></br>
Class that conducts AI control for a walker. The controllers are defined as actors, but they are quite different from the rest. They need to be attached to a parent actor during their creation, which is the walker they will be controlling (take a look at [carla.World](#carla.World) if you are yet to learn on how to spawn actors). They also need for a special blueprint (already defined in [carla.BlueprintLibrary](#carla.BlueprintLibrary) as "controller.ai.walker"). This is an empty blueprint, as the AI controller will be invisible in the simulation but will follow its parent around to dictate every step of the way.  

### Methods
- <a name="carla.WalkerAIController.go_to_location"></a>**<font color="#7fb800">go_to_location</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**destination**</font>)  
Sets the destination that the pedestrian will reach.  
    - **Parameters:**
        - `destination` (_[carla.Location](#carla.Location)<small> - meters</small>_)  
- <a name="carla.WalkerAIController.start"></a>**<font color="#7fb800">start</font>**(<font color="#00a6ed">**self**</font>)  
Enables AI control for its parent walker.  
- <a name="carla.WalkerAIController.stop"></a>**<font color="#7fb800">stop</font>**(<font color="#00a6ed">**self**</font>)<button class="SnipetButton" id="carla.WalkerAIController.stop-snipet_button">snippet &rarr;</button>  
Disables AI control for its parent walker.  

##### Setters
- <a name="carla.WalkerAIController.set_max_speed"></a>**<font color="#7fb800">set_max_speed</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**speed**=1.4</font>)  
Sets a speed for the walker in meters per second.  
    - **Parameters:**
        - `speed` (_float<small> - m/s</small>_) - An easy walking speed is set by default.  

##### Dunder methods
- <a name="carla.WalkerAIController.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WalkerBoneControlIn<a name="carla.WalkerBoneControlIn"></a>
This class grants bone specific manipulation for walker. The skeletons of walkers have been unified for clarity and the transform applied to each bone are always relative to its parent. Take a look [here](tuto_G_control_walker_skeletons.md) to learn more on how to create a walker and define its movement.  

### Instance Variables
- <a name="carla.WalkerBoneControlIn.bone_transforms"></a>**<font color="#f8805a">bone_transforms</font>** (_list([name,transform])_)  
List with the data for each bone we want to set:
  - name: bone name
  - relative: transform based on the bone parent.  

### Methods
- <a name="carla.WalkerBoneControlIn.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**list(name,transform)**</font>)  
Initializes an object containing moves to be applied on tick. These are listed with the name of the bone and the transform that will be applied to it.  
    - **Parameters:**
        - `list(name,transform)` (_tuple_)  

##### Dunder methods
- <a name="carla.WalkerBoneControlIn.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WalkerBoneControlOut<a name="carla.WalkerBoneControlOut"></a>
This class is used to return all bone positions of a pedestrian. For each bone we get its _name_ and its transform in three different spaces (world, actor and relative).  

### Instance Variables
- <a name="carla.WalkerBoneControlOut.bone_transforms"></a>**<font color="#f8805a">bone_transforms</font>** (_list([name,world, actor, relative])_)  
List of one entry per bone with this information:
  - name: bone name
  - world: transform in world coordinates
  - component: transform based on the pivot of the actor
  - relative: transform based on the bone parent.  

### Methods

##### Dunder methods
- <a name="carla.WalkerBoneControlOut.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WalkerControl<a name="carla.WalkerControl"></a>
This class defines specific directions that can be commanded to a [carla.Walker](#carla.Walker) to control it via script.
  
  AI control can be settled for walkers, but the control used to do so is [carla.WalkerAIController](#carla.WalkerAIController).  

### Instance Variables
- <a name="carla.WalkerControl.direction"></a>**<font color="#f8805a">direction</font>** (_[carla.Vector3D](#carla.Vector3D)_)  
Vector using global coordinates that will correspond to the direction of the walker.  
- <a name="carla.WalkerControl.speed"></a>**<font color="#f8805a">speed</font>** (_float<small> - m/s</small>_)  
A scalar value to control the walker's speed.  
- <a name="carla.WalkerControl.jump"></a>**<font color="#f8805a">jump</font>** (_bool_)  
If <b>True</b>, the walker will perform a jump.  

### Methods
- <a name="carla.WalkerControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**direction**=[1.0, 0.0, 0.0]</font>, <font color="#00a6ed">**speed**=0.0</font>, <font color="#00a6ed">**jump**=False</font>)  
    - **Parameters:**
        - `direction` (_[carla.Vector3D](#carla.Vector3D)_)  
        - `speed` (_float<small> - m/s</small>_)  
        - `jump` (_bool_)  

##### Dunder methods
- <a name="carla.WalkerControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WalkerControl](#carla.WalkerControl)</font>)  
Compares every variable with `other` and returns <b>True</b> if these are all the same.  
- <a name="carla.WalkerControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WalkerControl](#carla.WalkerControl)</font>)  
Compares every variable with `other` and returns <b>True</b> if any of these differ.  
- <a name="carla.WalkerControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Waypoint<a name="carla.Waypoint"></a>
Waypoints in CARLA are described as 3D directed points. They have a [carla.Transform](#carla.Transform) which locates the waypoint in a road and orientates it according to the lane. They also store the road information belonging to said point regarding its lane and lane markings.   <br><br> All the information regarding waypoints and the [waypoint API](../../core_map/#navigation-in-carla) is retrieved as provided by the OpenDRIVE file. Once the client asks for the map object to the server, no longer communication will be needed.  

### Instance Variables
- <a name="carla.Waypoint.id"></a>**<font color="#f8805a">id</font>** (_int_)  
The identifier is generated using a hash combination of the <b>road</b>, <b>section</b>, <b>lane</b> and <b>s</b> values that correspond to said point in the OpenDRIVE geometry. The <b>s</b> precision is set to 2 centimeters, so 2 waypoints closer than 2 centimeters in the same road, section and lane, will have the same identificator.  
- <a name="carla.Waypoint.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Position and orientation of the waypoint according to the current lane information. This data is computed the first time it is accessed. It is not created right away in order to ease computing costs when lots of waypoints are created but their specific transform is not needed.  
- <a name="carla.Waypoint.road_id"></a>**<font color="#f8805a">road_id</font>** (_int_)  
OpenDRIVE road's id.  
- <a name="carla.Waypoint.section_id"></a>**<font color="#f8805a">section_id</font>** (_int_)  
OpenDRIVE section's id, based on the order that they are originally defined.  
- <a name="carla.Waypoint.is_junction"></a>**<font color="#f8805a">is_junction</font>** (_bool_)  
<b>True</b> if the current Waypoint is on a junction as defined by OpenDRIVE.  
- <a name="carla.Waypoint.junction_id"></a>**<font color="#f8805a">junction_id</font>** (_int_)  
OpenDRIVE junction's id. For more information refer to OpenDRIVE [documentation](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf#page=20).  
- <a name="carla.Waypoint.lane_id"></a>**<font color="#f8805a">lane_id</font>** (_int_)  
OpenDRIVE lane's id, this value can be positive or negative which represents the direction of the current lane with respect to the road. For more information refer to OpenDRIVE [documentation](http://www.opendrive.org/docs/OpenDRIVEFormatSpecRev1.4H.pdf#page=20).  
- <a name="carla.Waypoint.s"></a>**<font color="#f8805a">s</font>** (_float_)  
OpenDRIVE <b>s</b> value of the current position.  
- <a name="carla.Waypoint.lane_width"></a>**<font color="#f8805a">lane_width</font>** (_float_)  
Horizontal size of the road at current <b>s</b>.  
- <a name="carla.Waypoint.lane_change"></a>**<font color="#f8805a">lane_change</font>** (_[carla.LaneChange](#carla.LaneChange)_)  
Lane change definition of the current Waypoint's location, based on the traffic rules defined in the OpenDRIVE file. It states if a lane change can be done and in which direction.  
- <a name="carla.Waypoint.lane_type"></a>**<font color="#f8805a">lane_type</font>** (_[carla.LaneType](#carla.LaneType)_)  
The lane type of the current Waypoint, based on OpenDRIVE 1.4 standard.  
- <a name="carla.Waypoint.right_lane_marking"></a>**<font color="#f8805a">right_lane_marking</font>** (_[carla.LaneMarking](#carla.LaneMarking)_)  
The right lane marking information based on the direction of the Waypoint.  
- <a name="carla.Waypoint.left_lane_marking"></a>**<font color="#f8805a">left_lane_marking</font>** (_[carla.LaneMarking](#carla.LaneMarking)_)  
The left lane marking information based on the direction of the Waypoint.  

### Methods
- <a name="carla.Waypoint.next"></a>**<font color="#7fb800">next</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
Returns a list of waypoints at a certain approximate `distance` from the current one. It takes into account the road and its possible deviations without performing any lane change and returns one waypoint per option.
The list may be empty if the lane is not connected to any other at the specified distance.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The approximate distance where to get the next waypoints.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.Waypoint.next_until_lane_end"></a>**<font color="#7fb800">next_until_lane_end</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
Returns a list of waypoints from this to the end of the lane separated by a certain `distance`.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The approximate distance between waypoints.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.Waypoint.previous"></a>**<font color="#7fb800">previous</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
This method does not return the waypoint previously visited by an actor, but a list of waypoints at an approximate `distance` but in the opposite direction of the lane. Similarly to **<font color="#7fb800">next()</font>**, it takes into account the road and its possible deviations without performing any lane change and returns one waypoint per option.
The list may be empty if the lane is not connected to any other at the specified distance.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The approximate distance where to get the previous waypoints.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  
- <a name="carla.Waypoint.previous_until_lane_start"></a>**<font color="#7fb800">previous_until_lane_start</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>)  
Returns a list of waypoints from this to the start of the lane separated by a certain `distance`.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The approximate distance between waypoints.  
    - **Return:** _list([carla.Waypoint](#carla.Waypoint))_  

##### Getters
- <a name="carla.Waypoint.get_junction"></a>**<font color="#7fb800">get_junction</font>**(<font color="#00a6ed">**self**</font>)  
If the waypoint belongs to a junction this method returns the associated junction object. Otherwise returns null.  
    - **Return:** _[carla.Junction](#carla.Junction)_  
- <a name="carla.Waypoint.get_landmarks"></a>**<font color="#7fb800">get_landmarks</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>, <font color="#00a6ed">**stop_at_junction**=False</font>)  
Returns a list of landmarks in the road from the current waypoint until the specified distance.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The maximum distance to search for landmarks from the current waypoint.  
        - `stop_at_junction` (_bool_) - Enables or disables the landmark search through junctions.  
    - **Return:** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Waypoint.get_landmarks_of_type"></a>**<font color="#7fb800">get_landmarks_of_type</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**distance**</font>, <font color="#00a6ed">**type**</font>, <font color="#00a6ed">**stop_at_junction**=False</font>)  
Returns a list of landmarks in the road of a specified type from the current waypoint until the specified distance.  
    - **Parameters:**
        - `distance` (_float<small> - meters</small>_) - The maximum distance to search for landmarks from the current waypoint.  
        - `type` (_str_) - The type of landmarks to search.  
        - `stop_at_junction` (_bool_) - Enables or disables the landmark search through junctions.  
    - **Return:** _list([carla.Landmark](#carla.Landmark))_  
- <a name="carla.Waypoint.get_left_lane"></a>**<font color="#7fb800">get_left_lane</font>**(<font color="#00a6ed">**self**</font>)  
Generates a Waypoint at the center of the left lane based on the direction of the current Waypoint, taking into account if the lane change is allowed in this location.
Will return <b>None</b> if the lane does not exist.  
    - **Return:** _[carla.Waypoint](#carla.Waypoint)_  
- <a name="carla.Waypoint.get_right_lane"></a>**<font color="#7fb800">get_right_lane</font>**(<font color="#00a6ed">**self**</font>)  
Generates a waypoint at the center of the right lane based on the direction of the current waypoint, taking into account if the lane change is allowed in this location.
Will return <b>None</b> if the lane does not exist.  
    - **Return:** _[carla.Waypoint](#carla.Waypoint)_  

##### Dunder methods
- <a name="carla.Waypoint.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WeatherParameters<a name="carla.WeatherParameters"></a>
This class defines objects containing lighting and weather specifications that can later be applied in [carla.World](#carla.World). So far, these conditions only intervene with [sensor.camera.rgb](ref_sensors.md#rgb-camera). They neither affect the actor's physics nor other sensors.        
  Each of these parameters acts indepently from the rest. Increasing the rainfall will not automatically create puddles nor change the road's humidity. That makes for a better customization but means that realistic conditions need to be scripted. However an example of dynamic weather conditions working realistically can be found [here](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/dynamic_weather.py).  

### Instance Variables
- <a name="carla.WeatherParameters.cloudiness"></a>**<font color="#f8805a">cloudiness</font>** (_float_)  
Values range from 0 to 100, being 0 a clear sky and 100 one completely covered with clouds.  
- <a name="carla.WeatherParameters.precipitation"></a>**<font color="#f8805a">precipitation</font>** (_float_)  
Rain intensity values range from 0 to 100, being 0 none at all and 100 a heavy rain.  
- <a name="carla.WeatherParameters.precipitation_deposits"></a>**<font color="#f8805a">precipitation_deposits</font>** (_float_)  
Determines the creation of puddles. Values range from 0 to 100, being 0 none at all and 100 a road completely capped with water. Puddles are created with static noise, meaning that they will always appear at the same locations.  
- <a name="carla.WeatherParameters.wind_intensity"></a>**<font color="#f8805a">wind_intensity</font>** (_float_)  
Controls the strenght of the wind with values from 0, no wind at all, to 100, a strong wind. The wind does affect rain direction and leaves from trees, so this value is restricted to avoid animation issues.  
- <a name="carla.WeatherParameters.sun_azimuth_angle"></a>**<font color="#f8805a">sun_azimuth_angle</font>** (_float<small> - degrees</small>_)  
The azimuth angle of the sun. Values range from 0 to 360. Zero is an origin point in a sphere determined by Unreal Engine.  
- <a name="carla.WeatherParameters.sun_altitude_angle"></a>**<font color="#f8805a">sun_altitude_angle</font>** (_float<small> - degrees</small>_)  
Altitude angle of the sun. Values range from -90 to 90 corresponding to midnight and midday each.  
- <a name="carla.WeatherParameters.fog_density"></a>**<font color="#f8805a">fog_density</font>** (_float_)  
Fog concentration or thickness. It only affects the RGB camera sensor. Values range from 0 to 100.  
- <a name="carla.WeatherParameters.fog_distance"></a>**<font color="#f8805a">fog_distance</font>** (_float<small> - meters</small>_)  
Fog start distance. Values range from 0 to infinite.  
- <a name="carla.WeatherParameters.wetness"></a>**<font color="#f8805a">wetness</font>** (_float_)  
Wetness intensity. It only affects the RGB camera sensor. Values range from 0 to 100.  
- <a name="carla.WeatherParameters.fog_falloff"></a>**<font color="#f8805a">fog_falloff</font>** (_float_)  
Density of the fog (as in specific mass) from 0 to infinity. The bigger the value, the more dense and heavy it will be, and the fog will reach smaller heights. Corresponds to <a href="https://docs.unrealengine.com/en-US/Engine/Actors/FogEffects/HeightFog/index.html#:~:text=Using%20Exponential%20Height%20Fog%20Features,-The%20sections%20below&text=Add%20a%20second%20fog%20layer,height%20falloff%2C%20and%20height%20offset">Fog Height Falloff</a> in the UE docs. <br> If the value is 0, the fog will be lighter than air, and will cover the whole scene. <br> A value of 1 is approximately as dense as the air, and reaches normal-sized buildings. <br> For values greater than 5, the air will be so dense that it will be compressed on ground level.  
- <a name="carla.WeatherParameters.scattering_intensity"></a>**<font color="#f8805a">scattering_intensity</font>** (_float_)  
Controls how much the light will contribute to volumetric fog. When set to 0, there is no contribution.  
- <a name="carla.WeatherParameters.mie_scattering_scale"></a>**<font color="#f8805a">mie_scattering_scale</font>** (_float_)  
Controls interaction of light with large particles like pollen or air pollution resulting in a hazy sky with halos around the light sources. When set to 0, there is no contribution.  
- <a name="carla.WeatherParameters.rayleigh_scattering_scale"></a>**<font color="#f8805a">rayleigh_scattering_scale</font>** (_float_)  
Controls interaction of light with small particles like air molecules. Dependent on light wavelength, resulting in a blue sky in the day or red sky in the evening.  
- <a name="carla.WeatherParameters.dust_storm"></a>**<font color="#f8805a">dust_storm</font>** (_float_)  
Determines the strength of the dust storm weather. Values range from 0 to 100.  

### Methods
- <a name="carla.WeatherParameters.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**cloudiness**=0.0</font>, <font color="#00a6ed">**precipitation**=0.0</font>, <font color="#00a6ed">**precipitation_deposits**=0.0</font>, <font color="#00a6ed">**wind_intensity**=0.0</font>, <font color="#00a6ed">**sun_azimuth_angle**=0.0</font>, <font color="#00a6ed">**sun_altitude_angle**=0.0</font>, <font color="#00a6ed">**fog_density**=0.0</font>, <font color="#00a6ed">**fog_distance**=0.0</font>, <font color="#00a6ed">**wetness**=0.0</font>, <font color="#00a6ed">**fog_falloff**=0.0</font>, <font color="#00a6ed">**scattering_intensity**=0.0</font>, <font color="#00a6ed">**mie_scattering_scale**=0.0</font>, <font color="#00a6ed">**rayleigh_scattering_scale**=0.0331</font>)  
Method to initialize an object defining weather conditions. This class has some presets for different noon and sunset conditions listed in a note below.  
    - **Parameters:**
        - `cloudiness` (_float_) - 0 is a clear sky, 100 complete overcast.  
        - `precipitation` (_float_) - 0 is no rain at all, 100 a heavy rain.  
        - `precipitation_deposits` (_float_) - 0 means no puddles on the road, 100 means roads completely capped by rain.  
        - `wind_intensity` (_float_) - 0 is calm, 100 a strong wind.  
        - `sun_azimuth_angle` (_float<small> - degrees</small>_) - 0 is an arbitrary North, 180 its corresponding South.  
        - `sun_altitude_angle` (_float<small> - degrees</small>_) - 90 is midday, -90 is midnight.  
        - `fog_density` (_float_) - Concentration or thickness of the fog, from 0 to 100.  
        - `fog_distance` (_float<small> - meters</small>_) - Distance where the fog starts in meters.  
        - `wetness` (_float_) - Humidity percentages of the road, from 0 to 100.  
        - `fog_falloff` (_float_) - Density (specific mass) of the fog, from 0 to infinity.  
        - `scattering_intensity` (_float_) - Controls how much the light will contribute to volumetric fog. When set to 0, there is no contribution.  
        - `mie_scattering_scale` (_float_) - Controls interaction of light with large particles like pollen or air pollution resulting in a hazy sky with halos around the light sources. When set to 0, there is no contribution.  
        - `rayleigh_scattering_scale` (_float_) - Controls interaction of light with small particles like air molecules. Dependent on light wavelength, resulting in a blue sky in the day or red sky in the evening.  
    - **Note:** <font color="#8E8E8E">_ClearNoon, CloudyNoon, WetNoon, WetCloudyNoon, SoftRainNoon, MidRainyNoon, HardRainNoon, ClearSunset, CloudySunset, WetSunset, WetCloudySunset, SoftRainSunset, MidRainSunset, HardRainSunset. 
_</font>  

##### Dunder methods
- <a name="carla.WeatherParameters.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns <b>True</b> if both objects' variables are the same.  
    - **Return:** _bool_  
- <a name="carla.WeatherParameters.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**</font>)  
Returns <b>True</b> if both objects' variables are different.  
    - **Return:** _bool_  
- <a name="carla.WeatherParameters.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.WheelPhysicsControl<a name="carla.WheelPhysicsControl"></a>
Class that defines specific physical parameters for wheel objects that will be part of a [carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl) to simulate vehicle it as a material object.  

### Instance Variables
- <a name="carla.WheelPhysicsControl.tire_friction"></a>**<font color="#f8805a">tire_friction</font>** (_float_)  
A scalar value that indicates the friction of the wheel.  
- <a name="carla.WheelPhysicsControl.damping_rate"></a>**<font color="#f8805a">damping_rate</font>** (_float_)  
Damping rate of the wheel.  
- <a name="carla.WheelPhysicsControl.max_steer_angle"></a>**<font color="#f8805a">max_steer_angle</font>** (_float<small> - degrees</small>_)  
Maximum angle that the wheel can steer.  
- <a name="carla.WheelPhysicsControl.radius"></a>**<font color="#f8805a">radius</font>** (_float<small> - centimeters</small>_)  
Radius of the wheel.  
- <a name="carla.WheelPhysicsControl.max_brake_torque"></a>**<font color="#f8805a">max_brake_torque</font>** (_float<small> - N*m</small>_)  
Maximum brake torque.  
- <a name="carla.WheelPhysicsControl.max_handbrake_torque"></a>**<font color="#f8805a">max_handbrake_torque</font>** (_float<small> - N*m</small>_)  
Maximum handbrake torque.  
- <a name="carla.WheelPhysicsControl.position"></a>**<font color="#f8805a">position</font>** (_[carla.Vector3D](#carla.Vector3D)_)  
World position of the wheel. This is a read-only parameter.  
- <a name="carla.WheelPhysicsControl.long_stiff_value"></a>**<font color="#f8805a">long_stiff_value</font>** (_float<small> - kg per radian</small>_)  
Tire longitudinal stiffness per unit gravitational acceleration. Each vehicle has a custom value.  
- <a name="carla.WheelPhysicsControl.lat_stiff_max_load"></a>**<font color="#f8805a">lat_stiff_max_load</font>** (_float_)  
Maximum normalized tire load at which the tire can deliver no more lateral stiffness no matter how much extra load is applied to the tire. Each vehicle has a custom value.  
- <a name="carla.WheelPhysicsControl.lat_stiff_value"></a>**<font color="#f8805a">lat_stiff_value</font>** (_float_)  
Maximum stiffness per unit of lateral slip. Each vehicle has a custom value.  

### Methods
- <a name="carla.WheelPhysicsControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**tire_friction**=2.0</font>, <font color="#00a6ed">**damping_rate**=0.25</font>, <font color="#00a6ed">**max_steer_angle**=70.0</font>, <font color="#00a6ed">**radius**=30.0</font>, <font color="#00a6ed">**max_brake_torque**=1500.0</font>, <font color="#00a6ed">**max_handbrake_torque**=3000.0</font>, <font color="#00a6ed">**position**=(0.0,0.0,0.0)</font>)  
    - **Parameters:**
        - `tire_friction` (_float_)  
        - `damping_rate` (_float_)  
        - `max_steer_angle` (_float<small> - degrees</small>_)  
        - `radius` (_float<small> - centimerers</small>_)  
        - `max_brake_torque` (_float<small> - N*m</small>_)  
        - `max_handbrake_torque` (_float<small> - N*m</small>_)  
        - `position` (_[carla.Vector3D](#carla.Vector3D)<small> - meters</small>_)  

##### Dunder methods
- <a name="carla.WheelPhysicsControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WheelPhysicsControl](#carla.WheelPhysicsControl)</font>)  
- <a name="carla.WheelPhysicsControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WheelPhysicsControl](#carla.WheelPhysicsControl)</font>)  
- <a name="carla.WheelPhysicsControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.World<a name="carla.World"></a>
World objects are created by the client to have a place for the simulation to happen. The world contains the map we can see, meaning the asset, not the navigation map. Navigation maps are part of the [carla.Map](#carla.Map) class. It also manages the weather and actors present in it. There can only be one world per simulation, but it can be changed anytime.  

### Instance Variables
- <a name="carla.World.id"></a>**<font color="#f8805a">id</font>** (_int_)  
The ID of the episode associated with this world. Episodes are different sessions of a simulation. These change everytime a world is disabled or reloaded. Keeping track is useful to avoid possible issues.  
- <a name="carla.World.debug"></a>**<font color="#f8805a">debug</font>** (_[carla.DebugHelper](#carla.DebugHelper)_)  
Responsible for creating different shapes for debugging. Take a look at its class to learn more about it.  

### Methods
- <a name="carla.World.apply_color_texture_to_object"></a>**<font color="#7fb800">apply_color_texture_to_object</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**object_name**</font>, <font color="#00a6ed">**material_parameter**</font>, <font color="#00a6ed">**texture**</font>)  
Applies a `texture` object in the field corresponfing to `material_parameter` (normal, diffuse, etc) to the object in the scene corresponding to `object_name`.  
    - **Parameters:**
        - `object_name` (_str_)  
        - `material_parameter` (_[carla.MaterialParameter](#carla.MaterialParameter)_)  
        - `texture` (_TextureColor_)  
- <a name="carla.World.apply_color_texture_to_objects"></a>**<font color="#7fb800">apply_color_texture_to_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**objects_name_list**</font>, <font color="#00a6ed">**material_parameter**</font>, <font color="#00a6ed">**texture**</font>)  
Applies a `texture` object in the field corresponfing to `material_parameter` (normal, diffuse, etc) to the object in the scene corresponding to all objects in `objects_name_list`.  
    - **Parameters:**
        - `objects_name_list` (_list(str)_)  
        - `material_parameter` (_[carla.MaterialParameter](#carla.MaterialParameter)_)  
        - `texture` (_TextureColor_)  
- <a name="carla.World.apply_float_color_texture_to_object"></a>**<font color="#7fb800">apply_float_color_texture_to_object</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**object_name**</font>, <font color="#00a6ed">**material_parameter**</font>, <font color="#00a6ed">**texture**</font>)  
Applies a `texture` object in the field corresponfing to `material_parameter` (normal, diffuse, etc) to the object in the scene corresponding to `object_name`.  
    - **Parameters:**
        - `object_name` (_str_)  
        - `material_parameter` (_[carla.MaterialParameter](#carla.MaterialParameter)_)  
        - `texture` (_TextureFloatColor_)  
- <a name="carla.World.apply_float_color_texture_to_objects"></a>**<font color="#7fb800">apply_float_color_texture_to_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**objects_name_list**</font>, <font color="#00a6ed">**material_parameter**</font>, <font color="#00a6ed">**texture**</font>)  
Applies a `texture` object in the field corresponfing to `material_parameter` (normal, diffuse, etc) to the object in the scene corresponding to all objects in `objects_name_list`.  
    - **Parameters:**
        - `objects_name_list` (_list(str)_)  
        - `material_parameter` (_[carla.MaterialParameter](#carla.MaterialParameter)_)  
        - `texture` (_TextureFloatColor_)  
- <a name="carla.World.apply_settings"></a>**<font color="#7fb800">apply_settings</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**world_settings**</font>)  
This method applies settings contained in an object to the simulation running and returns the ID of the frame they were implemented.  
    - **Parameters:**
        - `world_settings` (_[carla.WorldSettings](#carla.WorldSettings)_)  
    - **Return:** _int_  
    - **Warning:** <font color="#ED2F2F">_If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync mode too. Read [this](adv_traffic_manager.md#synchronous-mode) to learn how to do it. 
_</font>  
- <a name="carla.World.apply_textures_to_object"></a>**<font color="#7fb800">apply_textures_to_object</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**object_name**</font>, <font color="#00a6ed">**diffuse_texture**</font>, <font color="#00a6ed">**emissive_texture**</font>, <font color="#00a6ed">**normal_texture**</font>, <font color="#00a6ed">**ao_roughness_metallic_emissive_texture**</font>)  
Applies all texture fields in [carla.MaterialParameter](#carla.MaterialParameter) to the object `object_name`. Empty textures here will not be applied.  
    - **Parameters:**
        - `object_name` (_str_)  
        - `diffuse_texture` (_TextureColor_)  
        - `emissive_texture` (_TextureFloatColor_)  
        - `normal_texture` (_TextureFloatColor_)  
        - `ao_roughness_metallic_emissive_texture` (_TextureFloatColor_)  
- <a name="carla.World.apply_textures_to_objects"></a>**<font color="#7fb800">apply_textures_to_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**objects_name_list**</font>, <font color="#00a6ed">**diffuse_texture**</font>, <font color="#00a6ed">**emissive_texture**</font>, <font color="#00a6ed">**normal_texture**</font>, <font color="#00a6ed">**ao_roughness_metallic_emissive_texture**</font>)  
Applies all texture fields in [carla.MaterialParameter](#carla.MaterialParameter) to all objects in `objects_name_list`. Empty textures here will not be applied.  
    - **Parameters:**
        - `objects_name_list` (_list(str)_)  
        - `diffuse_texture` (_TextureColor_)  
        - `emissive_texture` (_TextureFloatColor_)  
        - `normal_texture` (_TextureFloatColor_)  
        - `ao_roughness_metallic_emissive_texture` (_TextureFloatColor_)  
- <a name="carla.World.cast_ray"></a>**<font color="#7fb800">cast_ray</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**initial_location**</font>, <font color="#00a6ed">**final_location**</font>)  
Casts a ray from the specified initial_location to final_location. The function then detects all geometries intersecting the ray and returns a list of [carla.LabelledPoint](#carla.LabelledPoint) in order.  
    - **Parameters:**
        - `initial_location` (_[carla.Location](#carla.Location)_) - The initial position of the ray.  
        - `final_location` (_[carla.Location](#carla.Location)_) - The final position of the ray.  
    - **Return:** _list([carla.LabelledPoint](#carla.LabelledPoint))_  
- <a name="carla.World.enable_environment_objects"></a>**<font color="#7fb800">enable_environment_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**env_objects_ids**</font>, <font color="#00a6ed">**enable**</font>)<button class="SnipetButton" id="carla.World.enable_environment_objects-snipet_button">snippet &rarr;</button>  
Enable or disable a set of EnvironmentObject identified by their id. These objects will appear or disappear from the level.  
    - **Parameters:**
        - `env_objects_ids` (_set(int)_) - Set of EnvironmentObject ids to change.  
        - `enable` (_bool_) - State to be applied to all the EnvironmentObject of the set.  
- <a name="carla.World.freeze_all_traffic_lights"></a>**<font color="#7fb800">freeze_all_traffic_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**frozen**</font>)  
Freezes or unfreezes all traffic lights in the scene. Frozen traffic lights can be modified by the user but the time will not update them until unfrozen.  
    - **Parameters:**
        - `frozen` (_bool_)  
- <a name="carla.World.ground_projection"></a>**<font color="#7fb800">ground_projection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**search_distance**</font>)  
Projects the specified point downwards in the scene. The functions casts a ray from location in the direction (0,0,-1) (downwards) and returns a [carla.LabelledPoint](#carla.LabelledPoint) object with the first geometry this ray intersects (usually the ground). If no geometry is found in the search_distance range the function returns `None`.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) - The point to be projected.  
        - `search_distance` (_float_) - The maximum distance to perform the projection.  
    - **Return:** _[carla.LabelledPoint](#carla.LabelledPoint)_  
- <a name="carla.World.load_map_layer"></a>**<font color="#7fb800">load_map_layer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**map_layers**</font>)<button class="SnipetButton" id="carla.World.load_map_layer-snipet_button">snippet &rarr;</button>  
加载图层到指定层次。如果该层次已经加载则没有任何效果。  
    - **参数：**
        - `map_layers` (_[carla.MapLayer](#carla.MapLayer)_) - 加载到指定层次的掩膜。 
    - **警告：** <font color="#ED2F2F">_这仅仅影响分层（Opt）地图。最小布局包括道路、人行道、交通灯和交通标志。_</font>  
- <a name="carla.World.on_tick"></a>**<font color="#7fb800">on_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback**</font>)  
此方法用于异步模式。它从客户端定义的`callback` 函数启动回调，并返回回调的 ID。每当服务器发出时滴答信号时，就会调用该函数。它需要一个 [carla.WorldSnapshot](#carla.WorldSnapshot) 作为参数，这可以从 __<font color="#7fb800">wait_for_tick()</font>__ 获得。使用 __<font color="#7fb800">remove_on_tick()</font>__ 来停止回调。  
    - **参数：**
        - `callback` (_[carla.WorldSnapshot](#carla.WorldSnapshot)_) - 将快照作为强制参数的函数，当客户端收到滴答信号时将调用该函数。  
    - **Return:** _int_  
- <a name="carla.World.project_point"></a>**<font color="#7fb800">project_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**direction**</font>, <font color="#00a6ed">**search_distance**</font>)  
Projects the specified point to the desired direction in the scene. The functions casts a ray from location in a direction and returns a [carla.Labelled](#carla.Labelled) object with the first geometry this ray intersects. If no geometry is found in the search_distance range the function returns `None`.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) - The point to be projected.  
        - `direction` (_[carla.Vector3D](#carla.Vector3D)_) - The direction of projection.  
        - `search_distance` (_float_) - The maximum distance to perform the projection.  
    - **Return:** _[carla.LabelledPoint](#carla.LabelledPoint)_  
- <a name="carla.World.remove_on_tick"></a>**<font color="#7fb800">remove_on_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback_id**</font>)  
Stops the callback for `callback_id` started with __<font color="#7fb800">on_tick()</font>__.  
    - **Parameters:**
        - `callback_id` (_callback_) - The callback to be removed. The ID is returned when creating the callback.  
- <a name="carla.World.reset_all_traffic_lights"></a>**<font color="#7fb800">reset_all_traffic_lights</font>**(<font color="#00a6ed">**self**</font>)  
Resets the cycle of all traffic lights in the map to the initial state.  
- <a name="carla.World.spawn_actor"></a>**<font color="#7fb800">spawn_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**attach_to**=None</font>, <font color="#00a6ed">**attachment**=Rigid</font>)<button class="SnipetButton" id="carla.World.spawn_actor-snipet_button">snippet &rarr;</button>  
The method will create, return and spawn an actor into the world. The actor will need an available blueprint to be created and a transform (location and rotation). It can also be attached to a parent with a certain attachment type.  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_) - The reference from which the actor will be created.  
        - `transform` (_[carla.Transform](#carla.Transform)_) - Contains the location and orientation the actor will be spawned with.  
        - `attach_to` (_[carla.Actor](#carla.Actor)_) - The parent object that the spawned actor will follow around.  
        - `attachment` (_[carla.AttachmentType](#carla.AttachmentType)_) - Determines how fixed and rigorous should be the changes in position according to its parent object.  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.tick"></a>**<font color="#7fb800">tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seconds**=10.0</font>)  
该方法用于 __同步__ 模式，即服务器在计算下一帧之前等待客户端客户端滴答信号。这个方法将发送滴答信号，并让位于服务器。它返回由服务器计算的新帧ID。  
    - **参数：**
        - `seconds` (_float<small> - 秒</small>_) - 服务器应该等待滴答信号的最大时间。它默认设置为 <code>10.0</code> 。  
    - **返回：** _int_  
    - **注意：** <font color="#8E8E8E">_如果在同步模式下没有收到滴答信号，仿真将冻结。此外，如果从不同的客户端接收到许多滴答信号，则可能存在同步问题。请阅读有关 __同步模式__ 的文档以了解更多信息。  
_</font>  
- <a name="carla.World.try_spawn_actor"></a>**<font color="#7fb800">try_spawn_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**attach_to**=None</font>, <font color="#00a6ed">**attachment**=Rigid</font>)  
和 __<font color="#7fb800">spawn_actor()</font>__ 一样，但是单失败时候返回 <b>None</b> 而不是抛出异常。  
    - **参数：**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_) - 将从中创建参与者的引用。  
        - `transform` (_[carla.Transform](#carla.Transform)_) - 包含参与者将使用的位置和朝向。 
        - `attach_to` (_[carla.Actor](#carla.Actor)_) - 派生的参与者将跟随的父对象。  
        - `attachment` (_[carla.AttachmentType](#carla.AttachmentType)_) - 根据其父对象确定位置更改的固定和严格程度。  
    - **返回：** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.unload_map_layer"></a>**<font color="#7fb800">unload_map_layer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**map_layers**</font>)<button class="SnipetButton" id="carla.World.unload_map_layer-snipet_button">snippet &rarr;</button>  
将选定的图层卸载到指定层次。如果层已经卸载，则调用没有任何效果。 
    - **参数:**
        - `map_layers` (_[carla.MapLayer](#carla.MapLayer)_) - 要卸载图层的掩膜。 
    - **警告:** <font color="#ED2F2F">_这仅仅影响分层（Opt）地图。最小布局包括道路、人行道、交通灯和交通标志。_</font>  
- <a name="carla.World.wait_for_tick"></a>**<font color="#7fb800">wait_for_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seconds**=10.0</font>)  
该方法使用在异步模式。It makes the client wait for a server tick. When the next frame is computed, the server will tick and return a snapshot describing the new state of the world.  
    - **Parameters:**
        - `seconds` (_float<small> - seconds</small>_) - Maximum time the server should wait for a tick. It is set to <code>10.0</code> by default.  
    - **Return:** _[carla.WorldSnapshot](#carla.WorldSnapshot)_  

##### Getters
- <a name="carla.World.get_actor"></a>**<font color="#7fb800">get_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_id**</font>)  
Looks up for an actor by ID and returns <b>None</b> if not found.  
    - **Parameters:**
        - `actor_id` (_int_)  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.get_actors"></a>**<font color="#7fb800">get_actors</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_ids**=None</font>)  
Retrieves a list of [carla.Actor](#carla.Actor) elements, either using a list of IDs provided or just listing everyone on stage. If an ID does not correspond with any actor, it will be excluded from the list returned, meaning that both the list of IDs and the list of actors may have different lengths.  
    - **Parameters:**
        - `actor_ids` (_list_) - The IDs of the actors being searched. By default it is set to <b>None</b> and returns every actor on scene.  
    - **Return:** _[carla.ActorList](#carla.ActorList)_  
- <a name="carla.World.get_blueprint_library"></a>**<font color="#7fb800">get_blueprint_library</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of actor blueprints available to ease the spawn of these into the world.  
    - **Return:** _[carla.BlueprintLibrary](#carla.BlueprintLibrary)_  
- <a name="carla.World.get_environment_objects"></a>**<font color="#7fb800">get_environment_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**object_type**=Any</font>)  
Returns a list of EnvironmentObject with the requested semantic tag.  The method returns all the EnvironmentObjects in the level by default, but the query can be filtered by semantic tags with the argument `object_type`.  
    - **Parameters:**
        - `object_type` (_[carla.CityObjectLabel](#carla.CityObjectLabel)_) - Semantic tag of the EnvironmentObjects that are returned.  
    - **Return:** _array([carla.EnvironmentObject](#carla.EnvironmentObject))_  
- <a name="carla.World.get_level_bbs"></a>**<font color="#7fb800">get_level_bbs</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_type**=Any</font>)  
Returns an array of bounding boxes with location and rotation in world space. The method returns all the bounding boxes in the level by default, but the query can be filtered by semantic tags with the argument `actor_type`.  
    - **Parameters:**
        - `actor_type` (_[carla.CityObjectLabel](#carla.CityObjectLabel)_) - Semantic tag of the elements contained in the bounding boxes that are returned.  
    - **Return:** _array([carla.BoundingBox](#carla.BoundingBox))_  
- <a name="carla.World.get_lightmanager"></a>**<font color="#7fb800">get_lightmanager</font>**(<font color="#00a6ed">**self**</font>)  
Returns an instance of [carla.LightManager](#carla.LightManager) that can be used to handle the lights in the scene.  
    - **Return:** _[carla.LightManager](#carla.LightManager)_  
- <a name="carla.World.get_map"></a>**<font color="#7fb800">get_map</font>**(<font color="#00a6ed">**self**</font>)  
Asks the server for the XODR containing the map file, and returns this parsed as a [carla.Map](#carla.Map).  
    - **Return:** _[carla.Map](#carla.Map)_  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulation. It is expensive, and should only be called once.  
_</font>  
- <a name="carla.World.get_names_of_all_objects"></a>**<font color="#7fb800">get_names_of_all_objects</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of the names of all objects in the scene that can be painted with the apply texture functions.  
    - **Return:** _list(str)_  
- <a name="carla.World.get_random_location_from_navigation"></a>**<font color="#7fb800">get_random_location_from_navigation</font>**(<font color="#00a6ed">**self**</font>)  
This can only be used with walkers. It retrieves a random location to be used as a destination using the __<font color="#7fb800">go_to_location()</font>__ method in [carla.WalkerAIController](#carla.WalkerAIController). This location will be part of a sidewalk. Roads, crosswalks and grass zones are excluded. The method does not take into consideration locations of existing actors so if a collision happens when trying to spawn an actor, it will return an error. Take a look at [`generate_traffic.py`](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/generate_traffic.py) for an example.  
    - **Return:** _[carla.Location](#carla.Location)_  
- <a name="carla.World.get_settings"></a>**<font color="#7fb800">get_settings</font>**(<font color="#00a6ed">**self**</font>)  
Returns an object containing some data about the simulation such as synchrony between client and server or rendering mode.  
    - **Return:** _[carla.WorldSettings](#carla.WorldSettings)_  
- <a name="carla.World.get_snapshot"></a>**<font color="#7fb800">get_snapshot</font>**(<font color="#00a6ed">**self**</font>)  
Returns a snapshot of the world at a certain moment comprising all the information about the actors.  
    - **Return:** _[carla.WorldSnapshot](#carla.WorldSnapshot)_  
- <a name="carla.World.get_spectator"></a>**<font color="#7fb800">get_spectator</font>**(<font color="#00a6ed">**self**</font>)<button class="SnipetButton" id="carla.World.get_spectator-snipet_button">snippet &rarr;</button>  
Returns the spectator actor. The spectator is a special type of actor created by Unreal Engine, usually with ID=0, that acts as a camera and controls the view in the simulator window.  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.get_traffic_light"></a>**<font color="#7fb800">get_traffic_light</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**landmark**</font>)  
Provided a landmark, returns the traffic light object it describes.  
    - **Parameters:**
        - `landmark` (_[carla.Landmark](#carla.Landmark)_) - The landmark object describing a traffic light.  
    - **Return:** _[carla.TrafficLight](#carla.TrafficLight)_  
- <a name="carla.World.get_traffic_light_from_opendrive_id"></a>**<font color="#7fb800">get_traffic_light_from_opendrive_id</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**traffic_light_id**</font>)  
Returns the traffic light actor corresponding to the indicated OpenDRIVE id.  
    - **Parameters:**
        - `traffic_light_id` (_str_) - The OpenDRIVE id.  
    - **Return:** _[carla.TrafficLight](#carla.TrafficLight)_  
- <a name="carla.World.get_traffic_lights_from_waypoint"></a>**<font color="#7fb800">get_traffic_lights_from_waypoint</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**waypoint**</font>, <font color="#00a6ed">**distance**</font>)  
This function performs a search along the road in front of the specified waypoint and returns a list of traffic light actors found in the specified search distance.  
    - **Parameters:**
        - `waypoint` (_[carla.Waypoint](#carla.Waypoint)_) - The input waypoint.  
        - `distance` (_float_) - Search distance.  
    - **Return:** _list([carla.TrafficLight](#carla.TrafficLight))_  
- <a name="carla.World.get_traffic_lights_in_junction"></a>**<font color="#7fb800">get_traffic_lights_in_junction</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**junction_id**</font>)  
Returns the list of traffic light actors affecting the junction indicated in `junction_id`.  
    - **Parameters:**
        - `junction_id` (_int_) - The id of the junction.  
    - **Return:** _list([carla.TrafficLight](#carla.TrafficLight))_  
- <a name="carla.World.get_traffic_sign"></a>**<font color="#7fb800">get_traffic_sign</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**landmark**</font>)  
Provided a landmark, returns the traffic sign object it describes.  
    - **Parameters:**
        - `landmark` (_[carla.Landmark](#carla.Landmark)_) - The landmark object describing a traffic sign.  
    - **Return:** _[carla.TrafficSign](#carla.TrafficSign)_  
- <a name="carla.World.get_vehicles_light_states"></a>**<font color="#7fb800">get_vehicles_light_states</font>**(<font color="#00a6ed">**self**</font>)  
Returns a dict where the keys are [carla.Actor](#carla.Actor) IDs and the values are [carla.VehicleLightState](#carla.VehicleLightState) of that vehicle.  
    - **Return:** _dict_  
- <a name="carla.World.get_weather"></a>**<font color="#7fb800">get_weather</font>**(<font color="#00a6ed">**self**</font>)  
Retrieves an object containing weather parameters currently active in the simulation, mainly cloudiness, precipitation, wind and sun position.  
    - **Return:** _[carla.WeatherParameters](#carla.WeatherParameters)_  
    - **Setter:** _[carla.World.set_weather](#carla.World.set_weather)_  

##### Setters
- <a name="carla.World.set_pedestrians_cross_factor"></a>**<font color="#7fb800">set_pedestrians_cross_factor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**percentage**</font>)  
    - **Parameters:**
        - `percentage` (_float_) - Sets the percentage of pedestrians that can walk on the road or cross at any point on the road. Value should be between `0.0` and `1.0`. For example, a value of `0.1` would allow 10% of pedestrians to walk on the road. __Default is `0.0`__.  
    - **Note:** <font color="#8E8E8E">_Should be set before pedestrians are spawned.
_</font>  
- <a name="carla.World.set_pedestrians_seed"></a>**<font color="#7fb800">set_pedestrians_seed</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seed**</font>)  
    - **Parameters:**
        - `seed` (_int_) - Sets the seed to use for any random number generated in relation to pedestrians.  
    - **Note:** <font color="#8E8E8E">_Should be set before pedestrians are spawned. If you want to repeat the same exact bodies (blueprint) for each pedestrian, then use the same seed in the Python code (where the blueprint is choosen randomly) and here, otherwise the pedestrians will repeat the same paths but the bodies will be different.
_</font>  
- <a name="carla.World.set_weather"></a>**<font color="#7fb800">set_weather</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**weather**</font>)  
Changes the weather parameteres ruling the simulation to another ones defined in an object.  
    - **Parameters:**
        - `weather` (_[carla.WeatherParameters](#carla.WeatherParameters)_) - New conditions to be applied.  
    - **Getter:** _[carla.World.get_weather](#carla.World.get_weather)_  

##### Dunder methods
- <a name="carla.World.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
The content of the world is parsed and printed as a brief report of its current state.  
    - **Return:** _string_  

---

## carla.WorldSettings<a name="carla.WorldSettings"></a>
The simulation has some advanced configuration options that are contained in this class and can be managed using [carla.World](#carla.World) and its methods. These allow the user to choose between client-server synchrony/asynchrony, activation of "no rendering mode" and either if the simulation should run with a fixed or variable time-step. Check [this](adv_synchrony_timestep.md) out if you want to learn about it.  

### Instance Variables
- <a name="carla.WorldSettings.synchronous_mode"></a>**<font color="#f8805a">synchronous_mode</font>** (_bool_)  
States the synchrony between client and server. When set to true, the server will wait for a client tick in order to move forward. It is false by default.  
- <a name="carla.WorldSettings.no_rendering_mode"></a>**<font color="#f8805a">no_rendering_mode</font>** (_bool_)  
When enabled, the simulation will run no rendering at all. This is mainly used to avoid overhead during heavy traffic simulations. It is false by default.  
- <a name="carla.WorldSettings.fixed_delta_seconds"></a>**<font color="#f8805a">fixed_delta_seconds</font>** (_float_)  
Ensures that the time elapsed between two steps of the simulation is fixed. Set this to <b>0.0</b> to work with a variable time-step, as happens by default.  
- <a name="carla.WorldSettings.substepping"></a>**<font color="#f8805a">substepping</font>** (_bool_)  
Enable the physics substepping. This option allows computing some physics substeps between two render frames. If synchronous mode is set, the number of substeps and its time interval are fixed and computed are so they fulfilled the requirements of [carla.WorldSettings.max_substep](#carla.WorldSettings.max_substep) and [carla.WorldSettings.max_substep_delta_time](#carla.WorldSettings.max_substep_delta_time). These last two parameters need to be compatible with [carla.WorldSettings.fixed_delta_seconds](#carla.WorldSettings.fixed_delta_seconds). Enabled by default.  
- <a name="carla.WorldSettings.max_substep_delta_time"></a>**<font color="#f8805a">max_substep_delta_time</font>** (_float_)  
Maximum delta time of the substeps. If the [carla.WorldSettingsmax_substep](#carla.WorldSettingsmax_substep) is high enough, the substep delta time would be always below or equal to this value. By default, the value is set to 0.01.  
- <a name="carla.WorldSettings.max_substeps"></a>**<font color="#f8805a">max_substeps</font>** (_int_)  
The maximum number of physics substepping that are allowed. By default, the value is set to 10.  
- <a name="carla.WorldSettings.max_culling_distance"></a>**<font color="#f8805a">max_culling_distance</font>** (_float_)  
Configure the max draw distance for each mesh of the level.  
- <a name="carla.WorldSettings.deterministic_ragdolls"></a>**<font color="#f8805a">deterministic_ragdolls</font>** (_bool_)  
Defines wether to use deterministic physics for pedestrian death animations or physical ragdoll simulation.  When enabled, pedestrians have less realistic death animation but ensures determinism.  When disabled, pedestrians are simulated as ragdolls with more realistic simulation and collision but no determinsm can be ensured.  
- <a name="carla.WorldSettings.tile_stream_distance"></a>**<font color="#f8805a">tile_stream_distance</font>** (_float_)  
Used for large maps only. Configures the maximum distance from the hero vehicle to stream tiled maps. Regions of the map within this range will be visible (and capable of simulating physics). Regions outside this region will not be loaded.  
- <a name="carla.WorldSettings.actor_active_distance"></a>**<font color="#f8805a">actor_active_distance</font>** (_float_)  
Used for large maps only. Configures the distance from the hero vehicle to convert actors to dormant. Actors within this range will be active, and actors outside will become dormant.  
- <a name="carla.WorldSettings.spectator_as_ego"></a>**<font color="#f8805a">spectator_as_ego</font>** (_bool_)  
Used for large maps only. Defines the influence of the spectator on tile loading in Large Maps. By default, the spectator will provoke loading of neighboring tiles in the absence of an ego actor. This might be inconvenient for applications that immediately spawn an ego actor.  

### Methods
- <a name="carla.WorldSettings.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**synchronous_mode**=False</font>, <font color="#00a6ed">**no_rendering_mode**=False</font>, <font color="#00a6ed">**fixed_delta_seconds**=0.0</font>, <font color="#00a6ed">**max_culling_distance**=0.0</font>, <font color="#00a6ed">**deterministic_ragdolls**=False</font>, <font color="#00a6ed">**tile_stream_distance**=3000</font>, <font color="#00a6ed">**actor_active_distance**=2000</font>, <font color="#00a6ed">**spectator_as_ego**=True</font>)  
Creates an object containing desired settings that could later be applied through [carla.World](#carla.World) and its method __<font color="#7fb800">apply_settings()</font>__.  
    - **Parameters:**
        - `synchronous_mode` (_bool_) - Set this to true to enable client-server synchrony.  
        - `no_rendering_mode` (_bool_) - Set this to true to completely disable rendering in the simulation.  
        - `fixed_delta_seconds` (_float<small> - seconds</small>_) - Set a fixed time-step in between frames. <code>0.0</code> means variable time-step and it is the default mode.  
        - `max_culling_distance` (_float<small> - meters</small>_) - Configure the max draw distance for each mesh of the level.  
        - `deterministic_ragdolls` (_bool_) - Defines wether to use deterministic physics or ragdoll simulation for pedestrian deaths.  
        - `tile_stream_distance` (_float<small> - meters</small>_) - Used for large maps only. Configures the maximum distance from the hero vehicle to stream tiled maps.  
        - `actor_active_distance` (_float<small> - meters</small>_) - Used for large maps only. Configures the distance from the hero vehicle to convert actors to dormant.  
        - `spectator_as_ego` (_bool_) - Used for large maps only. Defines the influence of the spectator on tile loading in Large Maps.  

##### Dunder methods
- <a name="carla.WorldSettings.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSettings](#carla.WorldSettings)</font>)  
Returns <b>True</b> if both objects' variables are the same.  
    - **Return:** _bool_  
- <a name="carla.WorldSettings.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSettings](#carla.WorldSettings)</font>)  
Returns <b>True</b> if both objects' variables are different.  
    - **Return:** _bool_  
- <a name="carla.WorldSettings.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the established settings to a string and shows them in command line.  
    - **Return:** _str_  

---

## carla.WorldSnapshot<a name="carla.WorldSnapshot"></a>
This snapshot comprises all the information for every actor on scene at a certain moment of time. It creates and gives acces to a data structure containing a series of [carla.ActorSnapshot](#carla.ActorSnapshot). The client recieves a new snapshot on every tick that cannot be stored.  

### Instance Variables
- <a name="carla.WorldSnapshot.id"></a>**<font color="#f8805a">id</font>** (_int_)  
A value unique for every snapshot to differentiate them.  
- <a name="carla.WorldSnapshot.frame"></a>**<font color="#f8805a">frame</font>** (_int_)  
Simulation frame in which the snapshot was taken.  
- <a name="carla.WorldSnapshot.timestamp"></a>**<font color="#f8805a">timestamp</font>** (_[carla.Timestamp](#carla.Timestamp)<small> - seconds</small>_)  
Precise moment in time when snapshot was taken. This class works in seconds as given by the operative system.  

### Methods
- <a name="carla.WorldSnapshot.find"></a>**<font color="#7fb800">find</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_id**</font>)  
Given a certain actor ID, returns its corresponding snapshot or <b>None</b> if it is not found.  
    - **Parameters:**
        - `actor_id` (_int_)  
    - **Return:** _[carla.ActorSnapshot](#carla.ActorSnapshot)_  
- <a name="carla.WorldSnapshot.has_actor"></a>**<font color="#7fb800">has_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_id**</font>)  
Given a certain actor ID, checks if there is a snapshot corresponding it and so, if the actor was present at that moment.  
    - **Parameters:**
        - `actor_id` (_int_)  
    - **Return:** _bool_  

##### Dunder methods
- <a name="carla.WorldSnapshot.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSnapshot](#carla.WorldSnapshot)</font>)  
Returns __True__ if both **<font color="#f8805a">timestamp</font>** are the same.  
    - **Return:** _bool_  
- <a name="carla.WorldSnapshot.__iter__"></a>**<font color="#7fb800">\__iter__</font>**(<font color="#00a6ed">**self**</font>)  
Iterate over the [carla.ActorSnapshot](#carla.ActorSnapshot) stored in the snapshot.  
- <a name="carla.WorldSnapshot.__len__"></a>**<font color="#7fb800">\__len__</font>**(<font color="#00a6ed">**self**</font>)  
Returns the amount of [carla.ActorSnapshot](#carla.ActorSnapshot) present in this snapshot.  
    - **Return:** _int_  
- <a name="carla.WorldSnapshot.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSnapshot](#carla.WorldSnapshot)</font>)  
Returns <b>True</b> if both **<font color="#f8805a">timestamp</font>** are different.  
    - **Return:** _bool_  

---

## command.ApplyAngularImpulse<a name="command.ApplyAngularImpulse"></a>
Command adaptation of __<font color="#7fb800">add_angular_impulse()</font>__ in [carla.Actor](#carla.Actor). Applies an angular impulse to an actor.  

### Instance Variables
- <a name="command.ApplyAngularImpulse.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyAngularImpulse.impulse"></a>**<font color="#f8805a">impulse</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - degrees*s</small>_)  
Angular impulse applied to the actor.  

### Methods
- <a name="command.ApplyAngularImpulse.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**impulse**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> - degrees*s</small>_)  

---

## command.ApplyForce<a name="command.ApplyForce"></a>
Command adaptation of __<font color="#7fb800">add_force()</font>__ in [carla.Actor](#carla.Actor). Applies a force to an actor.  

### Instance Variables
- <a name="command.ApplyForce.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyForce.force"></a>**<font color="#f8805a">force</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - N</small>_)  
Force applied to the actor over time.  

### Methods
- <a name="command.ApplyForce.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**force**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `force` (_[carla.Vector3D](#carla.Vector3D)<small> - N</small>_)  

---

## command.ApplyImpulse<a name="command.ApplyImpulse"></a>
Command adaptation of __<font color="#7fb800">add_impulse()</font>__ in [carla.Actor](#carla.Actor). Applies an impulse to an actor.  

### Instance Variables
- <a name="command.ApplyImpulse.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyImpulse.impulse"></a>**<font color="#f8805a">impulse</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - N*s</small>_)  
Impulse applied to the actor.  

### Methods
- <a name="command.ApplyImpulse.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**impulse**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> - N*s</small>_)  

---

## command.ApplyTargetAngularVelocity<a name="command.ApplyTargetAngularVelocity"></a>
Command adaptation of __<font color="#7fb800">set_target_angular_velocity()</font>__ in [carla.Actor](#carla.Actor). Sets the actor's angular velocity vector.  

### Instance Variables
- <a name="command.ApplyTargetAngularVelocity.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTargetAngularVelocity.angular_velocity"></a>**<font color="#f8805a">angular_velocity</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - deg/s</small>_)  
The 3D angular velocity that will be applied to the actor.  

### Methods
- <a name="command.ApplyTargetAngularVelocity.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**angular_velocity**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `angular_velocity` (_[carla.Vector3D](#carla.Vector3D)<small> - deg/s</small>_) - Angular velocity vector applied to the actor.  

---

## command.ApplyTargetVelocity<a name="command.ApplyTargetVelocity"></a>
Command adaptation of __<font color="#7fb800">set_target_velocity()</font>__ in [carla.Actor](#carla.Actor).  

### Instance Variables
- <a name="command.ApplyTargetVelocity.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTargetVelocity.velocity"></a>**<font color="#f8805a">velocity</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - m/s</small>_)  
The 3D velocity applied to the actor.  

### Methods
- <a name="command.ApplyTargetVelocity.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**velocity**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)<small> - m/s</small>_) - Velocity vector applied to the actor.  

---

## command.ApplyTorque<a name="command.ApplyTorque"></a>
Command adaptation of __<font color="#7fb800">add_torque()</font>__ in [carla.Actor](#carla.Actor). Applies a torque to an actor.  

### Instance Variables
- <a name="command.ApplyTorque.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTorque.torque"></a>**<font color="#f8805a">torque</font>** (_[carla.Vector3D](#carla.Vector3D)<small> - degrees</small>_)  
Torque applied to the actor over time.  

### Methods
- <a name="command.ApplyTorque.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**torque**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `torque` (_[carla.Vector3D](#carla.Vector3D)<small> - degrees</small>_)  

---

## command.ApplyTransform<a name="command.ApplyTransform"></a>
Command adaptation of __<font color="#7fb800">set_transform()</font>__ in [carla.Actor](#carla.Actor). Sets a new transform to an actor.  

### Instance Variables
- <a name="command.ApplyTransform.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTransform.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transformation to be applied.  

### Methods
- <a name="command.ApplyTransform.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**transform**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `transform` (_[carla.Transform](#carla.Transform)_)  

---

## command.ApplyVehicleAckermannControl<a name="command.ApplyVehicleAckermannControl"></a>
Command adaptation of __<font color="#7fb800">apply_ackermann_control()</font>__ in [carla.Vehicle](#carla.Vehicle). Applies a certain akermann control to a vehicle.  

### Instance Variables
- <a name="command.ApplyVehicleAckermannControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Vehicle actor affected by the command.  
- <a name="command.ApplyVehicleAckermannControl.control"></a>**<font color="#f8805a">control</font>** (_[carla.AckermannVehicleControl](#carla.AckermannVehicleControl)_)  
Vehicle ackermann control to be applied.  

### Methods
- <a name="command.ApplyVehicleAckermannControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `control` (_[carla.AckermannVehicleControl](#carla.AckermannVehicleControl)_)  

---

## command.ApplyVehicleControl<a name="command.ApplyVehicleControl"></a>
Command adaptation of __<font color="#7fb800">apply_control()</font>__ in [carla.Vehicle](#carla.Vehicle). Applies a certain control to a vehicle.  

### Instance Variables
- <a name="command.ApplyVehicleControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Vehicle actor affected by the command.  
- <a name="command.ApplyVehicleControl.control"></a>**<font color="#f8805a">control</font>** (_[carla.VehicleControl](#carla.VehicleControl)_)  
Vehicle control to be applied.  

### Methods
- <a name="command.ApplyVehicleControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `control` (_[carla.VehicleControl](#carla.VehicleControl)_)  

---

## command.ApplyVehiclePhysicsControl<a name="command.ApplyVehiclePhysicsControl"></a>
Command adaptation of __<font color="#7fb800">apply_physics_control()</font>__ in [carla.Vehicle](#carla.Vehicle). Applies a new physics control to a vehicle, modifying its physical parameters.  

### Instance Variables
- <a name="command.ApplyVehiclePhysicsControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Vehicle actor affected by the command.  
- <a name="command.ApplyVehiclePhysicsControl.physics_control"></a>**<font color="#f8805a">physics_control</font>** (_[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_)  
Physics control to be applied.  

### Methods
- <a name="command.ApplyVehiclePhysicsControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**physics_control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `physics_control` (_[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_)  

---

## command.ApplyWalkerControl<a name="command.ApplyWalkerControl"></a>
Command adaptation of __<font color="#7fb800">apply_control()</font>__ in [carla.Walker](#carla.Walker). Applies a control to a walker.  

### Instance Variables
- <a name="command.ApplyWalkerControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Walker actor affected by the command.  
- <a name="command.ApplyWalkerControl.control"></a>**<font color="#f8805a">control</font>** (_[carla.WalkerControl](#carla.WalkerControl)_)  
Walker control to be applied.  

### Methods
- <a name="command.ApplyWalkerControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `control` (_[carla.WalkerControl](#carla.WalkerControl)_)  

---

## command.ApplyWalkerState<a name="command.ApplyWalkerState"></a>
Apply a state to the walker actor. Specially useful to initialize an actor them with a specific location, orientation and speed.  

### Instance Variables
- <a name="command.ApplyWalkerState.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Walker actor affected by the command.  
- <a name="command.ApplyWalkerState.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transform to be applied.  
- <a name="command.ApplyWalkerState.speed"></a>**<font color="#f8805a">speed</font>** (_float<small> - m/s</small>_)  
Speed to be applied.  

### Methods
- <a name="command.ApplyWalkerState.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**speed**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
        - `speed` (_float<small> - m/s</small>_)  

---

## command.DestroyActor<a name="command.DestroyActor"></a>
Command adaptation of __<font color="#7fb800">destroy()</font>__ in [carla.Actor](#carla.Actor) that tells the simulator to destroy this actor. It has no effect if the actor was already destroyed. When executed with __<font color="#7fb800">apply_batch_sync()</font>__ in [carla.Client](#carla.Client) there will be a <b>command.Response</b> that will return a boolean stating whether the actor was successfully destroyed.  

### Instance Variables
- <a name="command.DestroyActor.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  

### Methods
- <a name="command.DestroyActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  

---

## command.Response<a name="command.Response"></a>
States the result of executing a command as either the ID of the actor to whom the command was applied to (when succeeded) or an error string (when failed).  actor ID, depending on whether or not the command succeeded. The method __<font color="#7fb800">apply_batch_sync()</font>__ in [carla.Client](#carla.Client) returns a list of these to summarize the execution of a batch.  

### Instance Variables
- <a name="command.Response.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor to whom the command was applied to. States that the command was successful.  
- <a name="command.Response.error"></a>**<font color="#f8805a">error</font>** (_str_)  
A string stating the command has failed.  

### Methods
- <a name="command.Response.has_error"></a>**<font color="#7fb800">has_error</font>**(<font color="#00a6ed">**self**</font>)  
Returns <b>True</b> if the command execution fails, and <b>False</b> if it was successful.  
    - **Return:** _bool_  

---

## command.SetAutopilot<a name="command.SetAutopilot"></a>
Command adaptation of __<font color="#7fb800">set_autopilot()</font>__ in [carla.Vehicle](#carla.Vehicle). Turns on/off the vehicle's autopilot mode.  

### Instance Variables
- <a name="command.SetAutopilot.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor that is affected by the command.  
- <a name="command.SetAutopilot.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  
If autopilot should be activated or not.  
- <a name="command.SetAutopilot.port"></a>**<font color="#f8805a">port</font>** (_uint16_)  
Port of the Traffic Manager where the vehicle is to be registered or unlisted.  

### Methods
- <a name="command.SetAutopilot.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>, <font color="#00a6ed">**port**=8000</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `enabled` (_bool_)  
        - `port` (_uint16_) - The Traffic Manager port where the vehicle is to be registered or unlisted. If __None__ is passed, it will consider a TM at default port `8000`.  

---

## command.SetEnableGravity<a name="command.SetEnableGravity"></a>
Command adaptation of __<font color="#7fb800">set_enable_gravity()</font>__ in [carla.Actor](#carla.Actor). Enables or disables gravity on an actor.  

### Instance Variables
- <a name="command.SetEnableGravity.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_[carla.Actor](#carla.Actor) or int_)  
Actor that is affected by the command.  
- <a name="command.SetEnableGravity.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  

### Methods
- <a name="command.SetEnableGravity.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or Actor ID to which the command will be applied to.  
        - `enabled` (_bool_)  

---

## command.SetSimulatePhysics<a name="command.SetSimulatePhysics"></a>
Command adaptation of __<font color="#7fb800">set_simulate_physics()</font>__ in [carla.Actor](#carla.Actor). Determines whether an actor will be affected by physics or not.  

### Instance Variables
- <a name="command.SetSimulatePhysics.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.SetSimulatePhysics.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  
If physics should be activated or not.  

### Methods
- <a name="command.SetSimulatePhysics.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `enabled` (_bool_)  

---

## command.SetVehicleLightState<a name="command.SetVehicleLightState"></a>
Command adaptation of __<font color="#7fb800">set_light_state()</font>__ in [carla.Vehicle](#carla.Vehicle). Sets the light state of a vehicle.  

### Instance Variables
- <a name="command.SetVehicleLightState.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor that is affected by the command.  
- <a name="command.SetVehicleLightState.light_state"></a>**<font color="#f8805a">light_state</font>** (_[carla.VehicleLightState](#carla.VehicleLightState)_)  
Defines the light state of a vehicle.  

### Methods
- <a name="command.SetVehicleLightState.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**light_state**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or its ID to whom the command will be applied to.  
        - `light_state` (_[carla.VehicleLightState](#carla.VehicleLightState)_) - Recaps the state of the lights of a vehicle, these can be used as a flags.  

---

## command.ShowDebugTelemetry<a name="command.ShowDebugTelemetry"></a>
Command adaptation of __<font color="#7fb800">show_debug_telemetry()</font>__ in [carla.Actor](#carla.Actor). Displays vehicle control telemetry data.  

### Instance Variables
- <a name="command.ShowDebugTelemetry.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_[carla.Actor](#carla.Actor) or int_)  
Actor that is affected by the command.  
- <a name="command.ShowDebugTelemetry.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  

### Methods
- <a name="command.ShowDebugTelemetry.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) - Actor or Actor ID to which the command will be applied to.  
        - `enabled` (_bool_)  

---

## command.SpawnActor<a name="command.SpawnActor"></a>
Command adaptation of __<font color="#7fb800">spawn_actor()</font>__ in [carla.World](#carla.World). Spawns an actor into the world based on the blueprint provided and the transform. If a parent is provided, the actor is attached to it.  

### Instance Variables
- <a name="command.SpawnActor.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transform to be applied.  
- <a name="command.SpawnActor.parent_id"></a>**<font color="#f8805a">parent_id</font>** (_int_)  
Identificator of the parent actor.  

### Methods
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>)  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_)  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**parent**</font>)  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_)  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
        - `parent` (_[carla.Actor](#carla.Actor) or int_)  
- <a name="command.SpawnActor.then"></a>**<font color="#7fb800">then</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**command**</font>)  
Links another command to be executed right after. It allows to ease very common flows such as spawning a set of vehicles by command and then using this method to set them to autopilot automatically.  
    - **Parameters:**
        - `command` (_any carla Command_) - a Carla command.  

---
[comment]: <> (=========================)
[comment]: <> (PYTHON API SCRIPT SNIPETS)
[comment]: <> (=========================)
<div id="snipets-container" class="Container" onmouseover='this.style["overflowX"]="scroll";' onmouseout='this.style["overflowX"]="visible";'></div>
<script>
function CopyToClipboard(containerid) {
if (document.selection) {
var range = document.body.createTextRange();
range.moveToElementText(document.getElementById(containerid));
range.select().createTextRange();
document.execCommand("copy");
} 
else if (window.getSelection) {
var range = document.createRange();
range.selectNode(document.getElementById(containerid));
window.getSelection().addRange(range);
document.execCommand("copy");
}
}
</script>
<script>
function CloseSnipet() {
document.getElementById("snipets-container").innerHTML = null;
}
</script>
  
<div id ="carla.World.enable_environment_objects-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.World.enable_environment_objects
</p>
<div id="carla.World.enable_environment_objects-code" class="SnipetContent">

```py
  
# This recipe turn visibility off and on for two specifc buildings on the map

# Get the buildings in the world
world = client.get_world()
env_objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

# Access individual building IDs and save in a set
building_01 = env_objs[0]
building_02 = env_objs[1]
objects_to_toggle = {building_01.id, building_02.id}

# Toggle buildings off
world.enable_environment_objects(objects_to_toggle, False)
# Toggle buildings on
world.enable_environment_objects(objects_to_toggle, True)
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.World.enable_environment_objects-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.DebugHelper.draw_string-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.DebugHelper.draw_string
</p>
<div id="carla.DebugHelper.draw_string-code" class="SnipetContent">

```py
  

# This recipe is a modification of lane_explorer.py example.
# It draws the path of an actor through the world, printing information at each waypoint.

# ...
current_w = map.get_waypoint(vehicle.get_location())
while True:

    next_w = map.get_waypoint(vehicle.get_location(), lane_type=carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk )
    # Check if the vehicle is moving
    if next_w.id != current_w.id:
        vector = vehicle.get_velocity()
        # Check if the vehicle is on a sidewalk
        if current_w.lane_type == carla.LaneType.Sidewalk:
            draw_waypoint_union(debug, current_w, next_w, cyan if current_w.is_junction else red, 60)
        else:
            draw_waypoint_union(debug, current_w, next_w, cyan if current_w.is_junction else green, 60)
        debug.draw_string(current_w.transform.location, str('%15.0f km/h' % (3.6 * math.sqrt(vector.x**2 + vector.y**2 + vector.z**2))), False, orange, 60)
        draw_transform(debug, current_w.transform, white, 60)

    # Update the current waypoint and sleep for some time
    current_w = next_w
    time.sleep(args.tick_time)
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.DebugHelper.draw_string-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.World.unload_map_layer-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.World.unload_map_layer
</p>
<div id="carla.World.unload_map_layer-code" class="SnipetContent">

```py
  
# This recipe toggles off several layers in our "_Opt" maps

# Load town one with minimum layout (roads, sidewalks, traffic lights and traffic signs)
# as well as buildings and parked vehicles
world = client.load_world('Town01_Opt', carla.MapLayer.Buildings | carla.MapLayer.ParkedVehicles) 

# Toggle all buildings off
world.unload_map_layer(carla.MapLayer.Buildings)

# Toggle all parked vehicles off
world.unload_map_layer(carla.MapLayer.ParkedVehicles)
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.World.unload_map_layer-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.Vehicle.set_wheel_steer_direction-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.Vehicle.set_wheel_steer_direction
</p>
<div id="carla.Vehicle.set_wheel_steer_direction-code" class="SnipetContent">

```py
  
# Sets the appearance of the vehicles front wheels to 40°. Vehicle physics will not be affected.

vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FR_Wheel, 40.0)
vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FL_Wheel, 40.0)
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.Vehicle.set_wheel_steer_direction-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.Client.__init__-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.Client.__init__
</p>
<div id="carla.Client.__init__-code" class="SnipetContent">

```py
  

# This recipe shows in every script provided in PythonAPI/Examples 
# and it is used to parse the client creation arguments when running the script. 

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = 0.1 / speed_factor

    client = carla.Client(args.host, args.port)

  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.Client.__init__-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.Map.get_waypoint-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.Map.get_waypoint
</p>
<div id="carla.Map.get_waypoint-code" class="SnipetContent">

```py
  

# This recipe shows the current traffic rules affecting the vehicle. 
# Shows the current lane type and if a lane change can be done in the actual lane or the surrounding ones.

# ...
waypoint = world.get_map().get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder | carla.LaneType.Sidewalk))
print("Current lane type: " + str(waypoint.lane_type))
# Check current lane change allowed
print("Current Lane change:  " + str(waypoint.lane_change))
# Left and Right lane markings
print("L lane marking type: " + str(waypoint.left_lane_marking.type))
print("L lane marking change: " + str(waypoint.left_lane_marking.lane_change))
print("R lane marking type: " + str(waypoint.right_lane_marking.type))
print("R lane marking change: " + str(waypoint.right_lane_marking.lane_change))
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.Map.get_waypoint-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  

<img src="/img/snipets_images/carla.Map.get_waypoint.jpg">
  
</div>
  
<div id ="carla.World.spawn_actor-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.World.spawn_actor
</p>
<div id="carla.World.spawn_actor-code" class="SnipetContent">

```py
  

# This recipe attaches different camera / sensors to a vehicle with different attachments.

# ...
camera = world.spawn_actor(rgb_camera_bp, transform, attach_to=vehicle, attachment_type=Attachment.Rigid)
# Default attachment:  Attachment.Rigid
gnss_sensor = world.spawn_actor(sensor_gnss_bp, transform, attach_to=vehicle)
collision_sensor = world.spawn_actor(sensor_collision_bp, transform, attach_to=vehicle)
lane_invasion_sensor = world.spawn_actor(sensor_lane_invasion_bp, transform, attach_to=vehicle)
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.World.spawn_actor-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.WalkerAIController.stop-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.WalkerAIController.stop
</p>
<div id="carla.WalkerAIController.stop-code" class="SnipetContent">

```py
  

#To destroy the pedestrians, stop them from the navigation, and then destroy the objects (actor and controller).

# stop pedestrians (list is [controller, actor, controller, actor ...])
for i in range(0, len(all_id), 2):
    all_actors[i].stop()

# destroy pedestrian (actor and controller)
client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.WalkerAIController.stop-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.DebugHelper.draw_box-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.DebugHelper.draw_box
</p>
<div id="carla.DebugHelper.draw_box-code" class="SnipetContent">

```py
  

# This recipe shows how to draw traffic light actor bounding boxes from a world snapshot.

# ....
debug = world.debug
world_snapshot = world.get_snapshot()

for actor_snapshot in world_snapshot:
    actual_actor = world.get_actor(actor_snapshot.id)
    if actual_actor.type_id == 'traffic.traffic_light':
        debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(0.5,0.5,2)),actor_snapshot.get_transform().rotation, 0.05, carla.Color(255,0,0,0),0)
# ...

  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.DebugHelper.draw_box-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  

<img src="/img/snipets_images/carla.DebugHelper.draw_box.jpg">
  
</div>
  
<div id ="carla.World.get_spectator-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.World.get_spectator
</p>
<div id="carla.World.get_spectator-code" class="SnipetContent">

```py
  

# This recipe spawns an actor and the spectator camera at the actor's location.

# ...
world = client.get_world()
spectator = world.get_spectator()

vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle.bmw.*'))
transform = random.choice(world.get_map().get_spawn_points())
vehicle = world.try_spawn_actor(vehicle_bp, transform)

# Wait for world to get the vehicle actor
world.tick()

world_snapshot = world.wait_for_tick()
actor_snapshot = world_snapshot.find(vehicle.id)

# Set spectator at given transform (vehicle transform)
spectator.set_transform(actor_snapshot.get_transform())
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.World.get_spectator-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.Sensor.listen-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.Sensor.listen
</p>
<div id="carla.Sensor.listen-code" class="SnipetContent">

```py
  

# This recipe applies a color conversion to the image taken by a camera sensor,
# so it is converted to a semantic segmentation image.

# ...
camera_bp = world.get_blueprint_library().filter('sensor.camera.semantic_segmentation')
# ...
cc = carla.ColorConverter.CityScapesPalette
camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame, cc))
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.Sensor.listen-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.TrafficLight.set_state-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.TrafficLight.set_state
</p>
<div id="carla.TrafficLight.set_state-code" class="SnipetContent">

```py
  

# This recipe changes from red to green the traffic light that affects the vehicle. 
# This is done by detecting if the vehicle actor is at a traffic light.

# ...
world = client.get_world()
spectator = world.get_spectator()

vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle.bmw.*'))
transform = random.choice(world.get_map().get_spawn_points())
vehicle = world.try_spawn_actor(vehicle_bp, transform)

# Wait for world to get the vehicle actor
world.tick()

world_snapshot = world.wait_for_tick()
actor_snapshot = world_snapshot.find(vehicle.id)

# Set spectator at given transform (vehicle transform)
spectator.set_transform(actor_snapshot.get_transform())
# ...# ...
if vehicle_actor.is_at_traffic_light():
    traffic_light = vehicle_actor.get_traffic_light()
    if traffic_light.get_state() == carla.TrafficLightState.Red:
       # world.hud.notification("Traffic light changed! Good to go!")
        traffic_light.set_state(carla.TrafficLightState.Green)
# ...

  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.TrafficLight.set_state-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  

<img src="/img/snipets_images/carla.TrafficLight.set_state.gif">
  
</div>
  
<div id ="carla.ActorBlueprint.set_attribute-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.ActorBlueprint.set_attribute
</p>
<div id="carla.ActorBlueprint.set_attribute-code" class="SnipetContent">

```py
  

# This recipe changes attributes of different type of blueprint actors.

# ...
walker_bp = world.get_blueprint_library().filter('walker.pedestrian.0002')
walker_bp.set_attribute('is_invincible', True)

# ...
# Changes attribute randomly by the recommended value
vehicle_bp = wolrd.get_blueprint_library().filter('vehicle.bmw.*')
color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
vehicle_bp.set_attribute('color', color)

# ...

camera_bp = world.get_blueprint_library().filter('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', 600)
camera_bp.set_attribute('image_size_y', 600)
# ...
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.ActorBlueprint.set_attribute-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.Client.apply_batch_sync-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.Client.apply_batch_sync
</p>
<div id="carla.Client.apply_batch_sync-code" class="SnipetContent">

```py
  
# 0. Choose a blueprint fo the walkers
world = client.get_world()
blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
walker_bp = random.choice(blueprintsWalkers)

# 1. Take all the random locations to spawn
spawn_points = []
for i in range(50):
    spawn_point = carla.Transform()
    spawn_point.location = world.get_random_location_from_navigation()
    if (spawn_point.location != None):
        spawn_points.append(spawn_point)

# 2. Build the batch of commands to spawn the pedestrians
batch = []
for spawn_point in spawn_points:
    walker_bp = random.choice(blueprintsWalkers)
    batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

# 2.1 apply the batch
results = client.apply_batch_sync(batch, True)
for i in range(len(results)):
    if results[i].error:
        logging.error(results[i].error)
    else:
        walkers_list.append({"id": results[i].actor_id})

# 3. Spawn walker AI controllers for each walker
batch = []
walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
for i in range(len(walkers_list)):
    batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

# 3.1 apply the batch
results = client.apply_batch_sync(batch, True)
for i in range(len(results)):
    if results[i].error:
        logging.error(results[i].error)
    else:
        walkers_list[i]["con"] = results[i].actor_id

# 4. Put altogether the walker and controller ids
for i in range(len(walkers_list)):
    all_id.append(walkers_list[i]["con"])
    all_id.append(walkers_list[i]["id"])
all_actors = world.get_actors(all_id)

# wait for a tick to ensure client receives the last transform of the walkers we have just created
world.wait_for_tick()

# 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
for i in range(0, len(all_actors), 2):
    # start walker
    all_actors[i].start()
    # set walk to random point
    all_actors[i].go_to_location(world.get_random_location_from_navigation())
    # random max speed
    all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.Client.apply_batch_sync-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  
<div id ="carla.World.load_map_layer-snipet" style="display: none;">
<p class="SnipetFont">
Snippet for carla.World.load_map_layer
</p>
<div id="carla.World.load_map_layer-code" class="SnipetContent">

```py
  
# This recipe toggles on several layers in our "_Opt" maps

# Load town one with only minimum layout (roads, sidewalks, traffic lights and traffic signs)
world = client.load_world('Town01_Opt', carla.MapLayer.None)

# Toggle all buildings on
world.load_map_layer(carla.MapLayer.Buildings)

# Toggle all foliage on
world.load_map_layer(carla.MapLayer.Foliage)

# Toggle all parked vehicles on
world.load_map_layer(carla.MapLayer.ParkedVehicles)
  

```
<button id="button1" class="CopyScript" onclick="CopyToClipboard('carla.World.load_map_layer-code')">Copy snippet</button>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<button id="button1" class="CloseSnipet" onclick="CloseSnipet()">Close snippet</button><br><br>
  
</div>
  

</div>

<script>
function ButtonAction(container_name){
if(window_big){
snipet_name = container_name.replace('-snipet_button','-snipet');
document.getElementById("snipets-container").innerHTML = document.getElementById(snipet_name).innerHTML;
}
else{
document.getElementById("snipets-container").innerHTML = null;code_name = container_name.replace('-snipet_button','-code');
var range = document.createRange();
range.selectNode(document.getElementById(code_name));
alert(range);
}
}
function WindowResize(){
if(window.innerWidth > 1200){
window_big = true;
}
else{
window_big = false;
}
}
var window_big;
if(window.innerWidth > 1200){
window_big = true;
}
else{
window_big = false;
}
buttons = document.getElementsByClassName('SnipetButton')
for (let i = 0; i < buttons.length; i++) {
buttons[i].addEventListener("click",function(){ButtonAction(buttons[i].id);},true);
}
window.onresize = WindowResize;
</script>
