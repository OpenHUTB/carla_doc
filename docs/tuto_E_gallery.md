## [基本示例](examples.md)

[交通灯的配置和使用](tuto_G_traffic_light.md)

[流量预测](tuto_G_traffic_prediction.md)


## [Carla 教程 (Python API)](https://github.com/wuhanstudio/carla-tutorial)

请使用 0.9.14 版本或者其他兼容 API 的 Carla 版本。

```
# 开始 Carla 仿真器
./CarlaUE4.sh -quality-level=epic -resx=800 -resy=600

# 如果您没有强大的 GPU
./CarlaUE4.sh -quality-level=low -resx=800 -resy=600

# 运行示例
python 01_get_started.py
python 02_sync_mode.py
python 03_RGB_camera.py
python 04_more_cameras.py
python 05_open3d_library.py
python 06_trafic_manager.py
python 07_3d_bounding_boxes.py
python 08_draw_waypoints.py
python 09_basic_navigation.py
python 10_manual_control.py
```

### Gallery

示例 01: 开始 (异步：车辆会有抖动感）

![](img/tuto_E_gallery/01_get_started.gif)

示例 02: 同步

![](img/tuto_E_gallery/02_sync_mode.gif)

示例 03: RGB 相机

![](img/tuto_E_gallery/03_RGB_camera.gif)

示例 04: 更多相机

![](img/tuto_E_gallery/04_more_cameras.gif)

示例 05: Open3D 激光雷达

![](img/tuto_E_gallery/05_open3d_lidar.gif)

示例 06: 交通管理器

![](img/tuto_E_gallery/06_traffic_manager.gif)

示例 07: 3D 边界框

![](img/tuto_E_gallery/07_3d_bounding_boxes.gif)

示例 08: 绘制航点

![](img/tuto_E_gallery/08_draw_waypoints.gif)

示例 09: 基本导航

![](img/tuto_E_gallery/09_basic_navigation.gif)

示例 10: 手动控制

![](img/tuto_E_gallery/10_manual_control.gif)
