# CARLA–PyTorch JIT 轮子动力学推理模块

> 该文档详细说明了代码中各模块的功能与调用流程，方便开发者快速理解与使用。

------

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 依赖与头文件](#2-依赖与头文件)
- [3. 全局辅助函数](#3-全局辅助函数)
- [4. carla::learning 命名空间](#4-carlalearning-命名空间)
- [5. NeuralModelImpl 结构体](#5-NeuralModelImpl-结构体)
- [6. NeuralModel 类接口](f3#6-NeuralModel-类接口)
- [7. 整体调用流程](#7-整体调用流程)

------

## 1. 项目概述

本模块在 **CARLA 仿真环境** 中实现了 **轮子动力学计算** 与 **PyTorch JIT 模型推理** 之间的链接。开发者可通过该模块将 CARLA 中的轮子动力学数据转换为适合深度学习模型处理的 PyTorch 张量，并通过加载的模型执行高效推理。该技术集成帮助开发者快速实现仿真数据驱动的自动驾驶算法测试与优化。

![该模块的各个函数关系图](../img/carla_learning_pytorch_structure.svg)

本模块提供以下功能特性：

1. **数据高效转换**：原地转换仿真数据为PyTorch张量，无额外内存开销。
2. **灵活推理方式**：支持CPU、动态推理和CUDA硬件加速多种模式。
3. **结果精确提取**：快速从推理输出提取轮子力、扭矩等动力学数据。
4. **扩展性良好**：方便开发者自定义扩展功能。

------

## 2. 依赖与头文件

- **源码参考**：[pytorch.h 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html)
- **torch/torch.h 与 torch/script.h**：核心PyTorch C++ API及JIT模型接口
- **torchscatter & torchcluster**：用于CUDA扩展功能验证
- **tensorexpr_fuser**：JIT优化控制，平衡性能与兼容性
- **CUDACachingAllocator**：手动控制CUDA缓存，优化内存使用

------

## 3. 全局辅助函数

### 3.1 add_mark

- **用途**：代码执行位置标记，便于调试
- **参数说明**：`text`（标记文本，无实际功能）
- **源码参考**：[pytorch.cpp](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.htmll#L20)

------

## 4. carla::learning 命名空间

封装轮子动力学与深度模型推理相关函数。- 源码参考：[pytorch.cpp](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.htmll#L25)

### 4.1 test_learning

- **作用**：打印CUDA环境信息，验证torchscatter和torchcluster。
- **示例输出**：`cuda version X.Y`

### 4.2 GetWheelTensorInputs

- **作用**：转换轮子动力学输入为张量
- **输入数据**：WheelInput结构，包含粒子位置、速度、轮子位姿与速度等
- **输出**：适用于模型推理的张量组

### 4.3 GetWheelTensorOutput

- **作用**：解析模型输出到WheelOutput
- **输入**：粒子与轮子的输出力、扭矩张量
- **输出**：填充WheelOutput结构

### 4.4 GetWheelTensorOutputDynamic

- **作用**：动态推理专用输出解析，与GetWheelTensorOutput逻辑一致

------

## 5. NeuralModelImpl 结构体

- **内部实现核心结构体**- 源码参考：[pytorch.cpp](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.htmll#L135)
- **成员变量**：
  - module：TorchScript模型对象
  - 缓存张量队列（用于高效推理，后续可扩展）

### 5.1 GetWheelTensorInputsCUDA

- **作用**：CUDA模式下的数据转换，附加设备标识
- **用途**：并行多轮子推理

------

## 6. NeuralModel 类接口

对外提供完整模型加载、设置输入、执行推理、获取输出的流程控制。- 源码参考：[pytorch.cpp](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.htmll#L187)

### 6.1 构造与析构

- 自动管理模型生命周期

### 6.2 LoadModel

- 加载TorchScript模型，支持GPU设备指定
- 异常安全，捕获并报告模型加载问题

### 6.3 SetInputs

- 存储完整输入数据（四轮状态、驾驶指令、地形等）

### 6.4 Forward (标准推理)

- 执行模型推理，CPU环境适用

### 6.5 ForwardDynamic (动态推理)

- 执行动态推理，推理后清理CUDA缓存

### 6.6 ForwardCUDATensors (CUDA推理)

- 全GPU模式，加速高效

### 6.7 GetOutputs

- 获取推理结果，包含所有轮子状态信息

------

## 7. 整体调用流程

模块典型使用流程：

![该模块的各个函数关系图](../img/neural_model_flow.svg)

**Step-by-Step示例：**

1. **模型加载**

```cpp
NeuralModel model;
model.LoadModel("mymodel.pt", /*device=*/0);
```

1. **准备并设置输入**

```cpp
Inputs inp;
// 构建 WheelInput 和驾驶指令
model.SetInputs(inp);
```

1. **执行推理**

```cpp
model.Forward();            // CPU模式
// 或
model.ForwardDynamic();     // 动态推理，适用于频繁模型切换
// 或
model.ForwardCUDATensors(); // GPU模式，加速推理
```

1. **提取结果**

```cpp
auto &outs = model.GetOutputs();
// 使用 outs.wheel0 至 outs.wheel3 的结果
```

通过以上流程，开发者可高效地在CARLA仿真中集成深度学习推理，提升自动驾驶仿真效果与性能。