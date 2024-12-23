## 车辆目录

### 检查模拟器中的车辆

运行以下脚本通过第一视角进行矿卡驾驶：
```shell
python manual_control.py --filter vehicle.miningtruck.miningtruck
```


要从目录中检查车辆，请使用以下代码，从下面的车辆详细信息中检索蓝图 ID，并将其粘贴到行 `bp_lib.find('blueprint.id.goes_here') ` 中 ：
```python
client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spectator = world.get_spectator()

# 设置车辆变换
vehicle_loc = carla.Location(x=-46.9, y=20.0, z=0.2)
vehicle_rot = carla.Rotation(pitch=0.0, yaw=142.0, roll=0.0)
vehicle_trans = carla.Transform(vehicle_loc,vehicle_rot)

# 将蓝图ID粘贴到这里：
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz') 

# 设置观看变换：
camera_loc = carla.Location(x=-48.7, y=24.8, z=1.7)
camera_rot = carla.Rotation(pitch=-13.4, yaw=-75.7, roll=0.0)
camera_trans = carla.Transform(camera_loc,camera_rot)

# 生成车辆
vehicle = world.spawn_actor(vehicle_bp, vehicle_trans)

# 移动观察者
spectator.set_transform(camera_trans)
```

在尝试生成另一辆车辆之前，不要忘记先销毁该车辆以避免碰撞：

```python
vehicle.destroy()
```
