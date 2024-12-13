# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import unittest  # 导入单元测试框架

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time

TESTING_ADDRESS = ('localhost', 2000)  # 测试服务器的IP地址和端口号，原来是3654端口
# vehicle_vehicles_exclude_from_old_towns：从旧的城镇中排除一些车辆
VEHICLE_VEHICLES_EXCLUDE_FROM_OLD_TOWNS = ['vehicle.mitsubishi.fusorosa', 'vehicle.carlamotors.european_hgv', 'vehicle.carlamotors.firetruck']


# 冒烟测试的主类
class SmokeTest(unittest.TestCase):
    # 测试前的初始化操作
    def setUp(self):
        self.testing_address = TESTING_ADDRESS
        self.client = carla.Client(*TESTING_ADDRESS)
        self.vehicle_vehicles_exclude_from_old_towns = VEHICLE_VEHICLES_EXCLUDE_FROM_OLD_TOWNS
        self.client.set_timeout(120.0)
        self.world = self.client.get_world()

    # 测试后的操作
    def tearDown(self):
        self.client.load_world("Town03")
        # 应变方法：：加载后给 UE4 一些时间来清理内存（旧资产）
        time.sleep(5)
        self.world = None
        self.client = None
    
    def filter_vehicles_for_old_towns(self, blueprint_list):
        new_list = []
        for blueprint in blueprint_list:
            if blueprint.id not in self.vehicle_vehicles_exclude_from_old_towns:
                new_list.append(blueprint)
        return new_list


class SyncSmokeTest(SmokeTest):
    def setUp(self):
        super(SyncSmokeTest, self).setUp()
        self.settings = self.world.get_settings()
        settings = carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=0.05)
        self.world.apply_settings(settings)
        self.world.tick()

    def tearDown(self):
        self.world.apply_settings(self.settings)
        self.world.tick()
        self.settings = None
        super(SyncSmokeTest, self).tearDown()
