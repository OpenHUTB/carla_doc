# 交通拥堵模拟

`generate_traffic.py` 是一个用于模拟交通的脚本。通过多次运行该脚本，可以模拟和分析不同交通流量下的交通拥堵情况，并进行数据统计和优化评估。

## 功能介绍

1. **交通拥堵模拟**：通过多次运行脚本生成大量车辆，模拟不同场景下的交通流量变化。
2. **[统计分析](traffic_indicators.md)**：对多次模拟过程中的拥堵情况进行数据采集和统计分析。
3. **效果评估**：评估不同交通优化措施对缓解交通拥堵的影响。

# generate_traffic.py 使用说明文档

## 概述

`generate_traffic.py` 是一个用于模拟交通拥堵的脚本。通过多次运行该脚本，可以模拟和分析不同交通流量下的交通拥堵情况，并进行数据统计和优化评估。

## 功能介绍

1. **交通拥堵模拟**：通过多次运行脚本生成大量车辆，模拟不同场景下的交通流量变化。
2. **统计分析**：对多次模拟过程中的拥堵情况进行数据采集和统计分析。
3. **效果评估**：评估不同交通优化措施对缓解交通拥堵的影响。

## 使用方法

### 依赖环境

在运行 `generate_traffic.py` 之前，请确保安装了以下依赖包：

- `CARLA`模拟环境的搭建。
- Carla,Numpy等依赖包的安装。

### 脚本运行

使用以下命令运行脚本：

```
python generate_traffic.py
```

## 多次运行模拟交通拥堵

为了模拟不同交通流量下的拥堵情况，可以多次运行 [**generate_traffic.py**](https://github.com/OpenHUTB/doc/blob/master/src/examples/generate_traffic.py) 脚本。

使用[**traffic_indicators.py**](https://github.com/OpenHUTB/doc/blob/master/src/course/traffic_indicators.py) 脚本对路口交通流量、路口车均延误、路口饱和、排队长度四种路口真实性评价指标分析。