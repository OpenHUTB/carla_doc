# CARLA–PyTorch JIT 轮子动力学推理模块

> 该文档详细说明了代码中各模块的功能与调用流程，方便开发者快速理解与使用。

------

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 依赖与头文件](#2-依赖与头文件)
- [3. 全局辅助函数](#3-全局辅助函数)
- [4. carla::learning 命名空间](#4-carlalearning-命名空间)
- [5. NeuralModelImpl 结构体](#5-NeuralModelImpl-结构体)
- [6. NeuralModel 类接口](#6-NeuralModel-类接口)
- [7. 整体调用流程](#7-整体调用流程)


## 1. 项目概述

本模块为 CARLA 仿真中 **轮子动力学计算** 与 **PyTorch JIT 模型推理** 搭建了桥梁。
 主要功能：

1. 将 C++ 原始数据（WheelInput）封装成 PyTorch 张量
2. 调用已加载的 TorchScript 模型执行前向推理
3. 从输出张量中读取粒子力和轮子力，填写 WheelOutput
4. 支持多种推理模式（CPU、动态、CUDA）

------

## 2. 依赖与头文件
- **源码参考** [pytorch.h 源码](https://openhutb.github.io/carla_cpp/db/dd0/pytorch_8h_source.html)
- **torch/torch.h / torch/script.h**：PyTorch C++ API 与 JIT 接口
- **torchscatter & torchcluster**：验证扩展库的 CUDA 支持
- **tensorexpr_fuser**：控制 JIT Fuse 优化
- **CUDACachingAllocator**：用于显式清理 CUDA 缓存

------

## 3. 全局辅助函数

### 3.1 add_mark
- **作用**：空壳函数，用于在代码中插入标记，便于日志或调试
- **参数**：`text` — 标记文本，不作实际处理
- **源码链接**：更多细节请参 [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L20)

------

## 4. carla::learning 命名空间

该命名空间封装了与轮子动力学及模型推理相关的核心函数。

### 4.1 test_learning
- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L27)
- **功能**：打印 torchcluster 和 torchscatter 的 CUDA 版本，验证环境

- **示例输出**：

  ```
  pgsql复制编辑cuda version X.Y
  cuda version X.Y
  ```

### 4.2 GetWheelTensorInputs 

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L37)
- **输入**：WheelInput 结构体，包含：
  - particles_positions (float*)
  - particles_velocities (float*)
  - wheel_positions (float[3])
  - wheel_orientation (float[4])
  - wheel_linear_velocity (float[3])
  - wheel_angular_velocity (float[3])
  - num_particles (int)
- **处理**：
  1. 使用 `torch::from_blob` 将原始指针包装成 at::Tensor（不复制数据）
  2. 构造 std::vector[torch::jit::IValue]()，按顺序存放所有张量
- **输出**：Value 元组，用于 Module::forward()

### 4.3 GetWheelTensorOutput  

- **源码参考**:[pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L74)
- **输入**：
  - particle_forces：形状 [num_particles, 3] 张量
  - wheel_forces：长度 6 张量（力 xyz + 扭矩 xyz）
- **处理**：
  1. 从 `wheel_forces.data_ptr<float>()` 读取 6 个值，赋给 WheelOutput 对应字段
  2. 遍历 `particle_forces.data_ptr<float>()`，按 xyz 顺序填充 WheelOutput::_particle_forces 向量
- **输出**：填充后的 WheelOutput 结构体

### 4.4 GetWheelTensorOutputDynamic 

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L107)
- **功能**：与 GetWheelTensorOutput 相同，用于“动态”推理后的后处理

------

## 5. NeuralModelImpl 结构体

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L133)
- **成员变量**：
  - module：已加载的 TorchScript 模型
  - particles_*_tensors：可选的缓存张量队列（当前未深入使用）
- **成员函数**：
  - GetWheelTensorInputsCUDA：CUDA 版本的 GetWheelTensorInputs

### 5.1 GetWheelTensorInputsCUDA

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L150)
- **功能**：与 GetWheelTensorInputs 相同，但创建后立即 `.cuda()` 并附加 wheel_idx 信息
- **用途**：并行推理时区分不同车轮

------

## 6. NeuralModel 类接口

该类对外暴露模型加载、输入设置、推理和结果获取接口。

**源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L187)

### 6.1 构造与析构

```
cpp复制编辑NeuralModel::NeuralModel()
NeuralModel::~NeuralModel()
```

- **功能**：使用 `std::make_unique<NeuralModelImpl>()` 初始化内部实现；自动释放

### 6.2 LoadModel

```
cpp


复制编辑
void NeuralModel::LoadModel(char* filename, int device)
```

- **功能**：
  1. `torch::jit::setTensorExprFuserEnabled(false)` 关闭 TensorExpr Fuse
  2. `Model->module = torch::jit::load(filename)` 加载模型
  3. （可选）将模型转移到 `cuda:device`
- **异常处理**：捕获并打印 c10::Error

### 6.3 SetInputs

```
cpp


复制编辑
void NeuralModel::SetInputs(Inputs input)
```

- **功能**：保存传入的 Inputs（包含四个 WheelInput、操作指令、地形类型、verbose 标志）

### 6.4 推理：Forward

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L219)
- **流程**：
  1. 调用 GetWheelTensorInputs 构建四组车轮张量
  2. 驾驶命令（steering, throttle, braking）及 optional terrain_type、verbose
  3. 执行 `module.forward(TorchInputs)`
  4. 拆解输出元组为 8 个张量，调用 GetWheelTensorOutput 填充 _output

### 6.5 动态推理：ForwardDynamic

- **区别**：使用 GetWheelTensorOutputDynamic，并在末尾调用 `CUDACachingAllocator::emptyCache()` 清理 CUDA 缓存

### 6.6 CUDA 推理：ForwardCUDATensors

- **区别**：
  - 使用 GetWheelTensorInputsCUDA 将所有张量 `.cuda()`
  - 驾驶指令等也 `.cuda()`
  - 其余逻辑同 Forward

### 6.7 结果获取：GetOutputs

- **源码参考**: [pytorch.cpp 源码](https://openhutb.github.io/carla_cpp/dd/d8c/pytorch_8cpp_source.html#L246)
- **功能**：返回内部 _output，包含四个 WheelOutput

------

## 7. 整体流程

1. **初始化与加载**

   ```
   cpp
   NeuralModel model;
   model.LoadModel("mymodel.pt", /*device=*/0);
   ```

2. **设置输入**

   ```
   cpp
   Inputs inp = /* 构造 WheelInput + 操作指令 */;
   model.SetInputs(inp);
   ```

3. **选择推理模式**

   ```
   cpp
   model.Forward();                // CPU 模式
   // 或
   model.ForwardDynamic();         // 动态 + 缓存清理
   // 或
   model.ForwardCUDATensors();     // 全 CUDA 模式
   ```

4. **获取输出**

   ```
   cpp
   auto &outs = model.GetOutputs();
   // outs.wheel0 ... outs.wheel3
   ```

