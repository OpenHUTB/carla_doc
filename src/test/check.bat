
:: 指定的Python 模块作为脚本运行
:: `-v`（小写v）参数用于启动详细的模式（verbose mode）
:: RuntimeError: time-out of 120000ms while waiting for the simulator, make sure the simulator is ready and connected to localhost:3654
python -m nose2 -v smoke.test_client smoke.test_sync smoke.test_sensor_determinism smoke.test_collision_determinism smoke.test_vehicle_physics smoke.test_props_loading smoke.test_sensor_tick_time smoke.test_map smoke.test_snapshot smoke.test_lidar smoke.test_streamming smoke.test_spawnpoints smoke.test_blueprint smoke.test_collision_sensor smoke.test_world smoke.test_determinism