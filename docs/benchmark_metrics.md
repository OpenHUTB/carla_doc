# 计算的基准性能指标

本页介绍了性能指标模块。此模块用于根据代理在基准测试期间执行的操作计算结果摘要。


## 提供性能指标

驾驶基准性能指标模块提供以下性能指标：

- **成功百分比**：代理成功完成的情节（任务姿势）的百分比。

- **平均完成度**：代理能够到达目标的平均距离。

- **越野路口**：代理驶出道路的次数。只有当车辆在道路外的面积大于阈值时，才会计算路口。

- **其他车道交叉口**：代理进入其他车道的次数。只有当其他车道上的车辆面积大于阈值时，才会计算交叉口。

- **车辆碰撞**：影响量大于阈值的车辆碰撞次数。

- **行人碰撞**：与行人发生碰撞且影响大于阈值的次数。

- **一般碰撞**：与所有其他物体发生碰撞且影响大于阈值的次数。

## 执行并设置参数
指标作为基准测试的最后一步进行计算，并将结果摘要存储在 json 文件中。内部执行如下：

```python
metrics_object = Metrics(metrics_parameters)
summary_dictionary = metrics_object.compute(path_to_execution_log)
```

Metric 的计算函数接收执行日志的完整路径。Metric 类应该使用一些参数进行实例化。这些参数包括：

- **阈值**：指标使用的阈值。
- **帧重新计数**：犯规后，设置代理需要继续犯规的帧数，才会将其计为另一次犯规。
- **跳过帧数**：与碰撞或交叉开始后跳过的帧数有关。

这些参数被定义为 *实验套件* 基类的属性，可以在您的 [自定义实验套件](https://carla.readthedocs.io/en/stable/benchmark_creating/#defining-the-experiment-suite) 中重新定义。

默认参数为：

```python
@property
    def metrics_parameters(self):
    """
    Property to return the parameters for the metrics module
    Could be redefined depending on the needs of the user.
    """
    return {

        'intersection_offroad': {'frames_skip': 10,
                                 'frames_recount': 20,
                                 'threshold': 0.3
                                 },
        'intersection_otherlane': {'frames_skip': 10,
                                   'frames_recount': 20,
                                   'threshold': 0.4
                                   },
        'collision_other': {'frames_skip': 10,
                            'frames_recount': 20,
                            'threshold': 400
                            },
        'collision_vehicles': {'frames_skip': 10,
                               'frames_recount': 30,
                               'threshold': 400
                               },
        'collision_pedestrians': {'frames_skip': 5,
                                  'frames_recount': 100,
                                  'threshold': 300
                                  },

          }
```
