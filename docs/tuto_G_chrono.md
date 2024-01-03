# Chrono 集成

本指南概述了 Chrono 是什么、如何在 Carla 中使用它以及集成中涉及的限制。

- [__Chrono 项目__](#project-chrono)
- [__在 Carla 上使用 Chrono__](#using-chrono-on-carla)
    - [配置服务器](#configuring-the-server)
    - [启用 Chrono 物理](#enabling-chrono-physics)
- [__局限性__](#limitations)

---

## Chrono 项目

[Project Chrono](https://projectchrono.org/) 是一款开源多物理场仿真引擎，它使用基于模板的方法提供高度真实的车辆动力学。Carla 中的集成允许用户在导航地图时利用 Chrono 模板来模拟车辆动力学。

---

## 在 Carla 上使用 Chrono

要使用 Chrono 集成，您必须首先在启动时使用标签配置服务器，然后使用 PythonAPI 在生成的车辆上启用它。请阅读以获得更多详情。

### 配置服务器

仅当 CARLA 服务器使用 Chrono 标签编译时，Chrono 才会工作。

__在从 CARLA 的源代码构建版本中__，运行以下命令来启动服务器：

```sh
make launch ARGS="--chrono"
```

---

### 启用 Chrono 物理

Chrono 物理是通过 [Actor](python_api.md#carlaactor) 类提供的 `enable_chrono_physics` 方法启用的。除了子步骤和子步骤增量时间的值之外，它还需要三个模板文件和一个基本路径来定位这些文件： 

- __`base_path`:__ 包含模板文件的目录路径。这对于确保从模板文件引用的辅助文件具有用于搜索的公共基本路径是必要的。
- __`vehicle_json`:__ 相对于 `base_path`的车辆模板文件 .
- __`tire_json`:__ 相对于 `base_path` 的轮胎模板文件。
- __`powertrain_json`:__ 相对于 `base_path` 动力总成模板文件。

!!! 重要
    仔细检查您的路径。不正确或缺失的路径可能会导致虚幻引擎崩溃。

`Build/chrono-install/share/chrono/data/vehicle` 中提供了适用于不同车辆的各种示例模板文件。阅读 Project Chrono [文档](https://api.projectchrono.org/manual_vehicle.html) ，了解有关其车辆示例以及如何创建模板的更多信息。

请参阅下面的示例，了解如何启用 Chrono 物理：

```python
    # Spawn your vehicle
    vehicle = world.spawn_actor(bp, spawn_point)

    # Set the base path
    base_path = "/path/to/carla/Build/chrono-install/share/chrono/data/vehicle/"

    # Set the template files

    vehicle_json = "sedan/vehicle/Sedan_Vehicle.json"
    powertrain_json = "sedan/powertrain/Sedan_SimpleMapPowertrain.json"
    tire_json = "sedan/tire/Sedan_TMeasyTire.json"

    # Enable Chrono physics

    vehicle.enable_chrono_physics(5000, 0.002, vehicle_json, powertrain_json, tire_json, base_path)
```

您可以使用 `PythonAPI/examples` 中的示例脚本 `manual_control_chrono.py` 尝试 Chrono 物理集成。运行脚本后，按`Ctrl + o`启用 Chrono。

---

### 局限性

此集成不支持碰撞。__当发生碰撞时，车辆将恢复为 CARLA 默认物理状态。__
