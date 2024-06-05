# 驾驶基准
_驾驶基准模块_ 用于评估驾驶控制器（智能体）并获取有关其性能的指标。

该模块主要设计用于：

- 致力于开发自动驾驶代理并希望了解其在 CARLA 中的表现的用户。

在本节中您将了解。

- 如何快速入门并立即对简单智能体进行基准测试。
- 了解驾驶基准测试模块的总体实现架构。
- 了解如何设置您的代理并创建您自己的一组实验。
- 了解所使用的性能指标。


# 入门
为了让您熟悉系统，我们提供了一个在一组小型实验中执行的简单智能体（基本）。要执行它，只需运行：
```shell
./driving_benchmark_example.py
```

请记住，要运行上述命令，您需要在本地主机和端口 2000 上运行 CARLA 模拟器。

我们已经提供了与 [CoRL 2017](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf) 论文中使用的相同基准。CoRL 2017 实验套件可以在一个简单的代理中运行，只需运行：
```shell
./driving_benchmark_example.py --corl-2017
```

此基准测试示例可以进一步配置。运行 help 命令查看可用的选项。
```shell
./driving_benchmark_example.py --help
```

可用的选项之一是能够从之前的基准测试执行继续。例如，要继续 CoRL2017 中日志名称为“driving_benchmark_test”的实验，请运行：
```shell
./driving_benchmark_example.py --continue-experiment -n driving_benchmark_test --corl-2017
```

!!! 笔记
    如果日志名称已经存在并且您没有将其设置为继续，它将以不同的名称创建另一个日志。

当运行基本配置的驾驶基准测试时，你应该 [期待这些结果](https://carla.readthedocs.io/en/stable/benchmark_creating/#expected-results) 。




