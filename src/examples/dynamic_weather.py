#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# 解读：https://bbs.carla.org.cn/info/e11460e2c6444888ae21f28cee1ec811?csr=1
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA 动态天气：

连接到 CARLA 仿真器实例并控制天气。随着时间的推移，太阳的位置会平稳地变化，偶尔会产生风暴。

"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import math


# 限值函数，将输入的参数限制在0到100中间后输出
def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


# 太阳类，用以控制太阳方位角和太阳高度角
class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    # 步进更新，更新太阳高度，方位角
    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds  # 方位角
        self.azimuth %= 360.0  # 每360度数值重新开始
        self.altitude = (70 * math.sin(self._t)) - 20  # 高度

    # 输出当前太阳高度，方位角
    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


# 风暴类用于控制降水，云雾，湿度，积水，风等
class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0  # 降水量
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0  # 道路的湿度百分比，从 0 到 100
        self.puddles = 0.0  # 水坑
        self.wind = 0.0
        self.fog = 0.0

    # # 更新风暴类中的所有参数，降水云雾等
    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)  # 把水坑限制在0到85之间
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        # clouds小于20则wind为5，clouds大于70则wind为90，其他wind为40
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False

    # 输出当前云量，降水和风强度
    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


# 天气类，包含太阳和风暴
class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        # 云量，范围0-100，值为0时无云，值为100时天空完全被云覆盖
        self.weather.cloudiness = self._storm.clouds
        # 降水量，范围0-100，值为0时没有降雨，值为100时为大雨
        self.weather.precipitation = self._storm.rain
        # 积水量，范围0-100，决定了道路上积水水洼的产生，值为0时完全没有积水，为100时道路完全被水覆盖，
        # 注意水洼会在固定的地点生成，没有随机性
        self.weather.precipitation_deposits = self._storm.puddles
        # 风强，范围0-100，控制风的强度，从0-100为无风到强风，风会影响雨和树叶的效果。
        self.weather.wind_intensity = self._storm.wind
        # 雾气浓度，范围0-100，注意此属性只影响RGB相机的输出
        self.weather.fog_density = self._storm.fog
        # 湿度，范围0-100，可能会引起镜头水雾等，注意此属性只影响RGB相机的输出
        self.weather.wetness = self._storm.wetness
        # 太阳方位角，范围0-360，单位为角度，值为0时为UE引擎定义的天空球原点
        self.weather.sun_azimuth_angle = self._sun.azimuth
        # 太阳高度角，范围-90-+90，单位为角度，-90时代表午夜，+90时代表正午
        self.weather.sun_altitude_angle = self._sun.altitude

    # 输出当前天气参数
    def __str__(self):
        return '%s %s' % (self._sun, self._storm)


def main():
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
    client.set_timeout(2.0)
    world = client.get_world()

    weather = Weather(world.get_weather())

    elapsed_time = 0.0

    while True:
        timestamp = world.wait_for_tick(seconds=30.0).timestamp
        elapsed_time += timestamp.delta_seconds
        if elapsed_time > update_freq:
            weather.tick(speed_factor * elapsed_time)
            world.set_weather(weather.weather)
            sys.stdout.write('\r' + str(weather) + 12 * ' ')
            sys.stdout.flush()
            elapsed_time = 0.0


if __name__ == '__main__':

    main()
