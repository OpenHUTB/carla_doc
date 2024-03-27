# 获取所有地图，并根据地图名进行切换
import carla

# 指定端口启动Carla服务：
# CarlaUE4.exe -carla-rpc-port=3000
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
print(client.get_available_maps())  # 获取所有可使用地图名

world = client.get_world()
print(world.get_map())  # 获取当前使用地图名
# ['/Game/Carla/Maps/Town10HD_Opt', '/Game/RoadRunner/Maps/ScenarioBasic']
# 注意：客户端默认加载使用Town10HD_Opt地图，Opt后缀表示可分层(Optional)地图。

# world = client.load_world('Town10HD')  # 高精度地图城市
# world = client.load_world('Town02')
# world = client.load_world('/Game/Carla/Maps/Town02')
# world = client.load_world('/Game/Carla/Maps/roundaboutA/roundaboutA')
print(world.get_map())
# world.unload_map_layer(carla.MapLayer.All) # 隐藏所有层

