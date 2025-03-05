# NeuralModel 说明文档

## 1. 概述

该代码由巴塞罗那自治大学 (Universitat Autonoma de Barcelona, UAB) 计算机视觉中心 (Computer Vision Center, CVC) 开发，并采用 MIT 许可证授权。该代码定义了 `carla::learning` 命名空间下的一组数据结构和类，用于基于 PyTorch 进行神经网络模型的推理，主要应用于自动驾驶仿真场景。

## 2. 主要组件

### 2.1 `test_learning` 函数

- 该函数用于测试学习功能，但具体实现未包含在该代码文件中。

### 2.2 `NeuralModelImpl` 结构体

- 该结构体未在代码中定义，仅作为 `NeuralModel` 的私有实现细节。

### 2.3 `WheelInput` 结构体

- 该结构体用于表示单个车轮的输入数据。

- 包含以下成员变量：
  
  - `num_particles`: 粒子数量。
  
  - `particles_positions`: 指向粒子位置数组的指针。
  
  - `particles_velocities`: 指向粒子速度数组的指针。
  
  - `wheel_positions`: 指向车轮位置数组的指针。
  
  - `wheel_oritentation`: 指向车轮方向数组的指针。
  
  - `wheel_linear_velocity`: 指向车轮线速度数组的指针。
  
  - `wheel_angular_velocity`: 指向车轮角速度数组的指针。

### 2.4 `Inputs` 结构体

- 该结构体用于存储四个车轮的输入数据以及驾驶控制信息。

- 包含以下成员变量：
  
  - `wheel0, wheel1, wheel2, wheel3`: 四个 `WheelInput` 结构体，分别表示四个车轮的输入数据。
  
  - `steering`: 车辆的转向角度。
  
  - `throttle`: 车辆的油门输入。
  
  - `braking`: 车辆的刹车输入。
  
  - `terrain_type`: 当前地形类型。
  
  - `verbose`: 是否启用详细输出。

### 2.5 `WheelOutput` 结构体

- 该结构体用于存储单个车轮的输出数据。

- 包含以下成员变量：
  
  - `wheel_forces_x, wheel_forces_y, wheel_forces_z`: 车轮受力（x, y, z 轴）。
  
  - `wheel_torque_x, wheel_torque_y, wheel_torque_z`: 车轮扭矩（x, y, z 轴）。
  
  - `_particle_forces`: 存储粒子受力的数组。

### 2.6 `Outputs` 结构体

- 该结构体用于存储四个车轮的输出数据。

- 包含以下成员变量：
  
  - `wheel0, wheel1, wheel2, wheel3`: 四个 `WheelOutput` 结构体，分别表示四个车轮的输出数据。

### 2.7 `NeuralModel` 类

该类提供了一个基于 PyTorch 进行推理的神经网络模型。

#### **成员函数**

- `NeuralModel()`: 构造函数。

- `void LoadModel(char* filename, int device)`: 加载神经网络模型。

- `void SetInputs(Inputs input)`: 设置输入数据。

- `void Forward()`: 进行前向传播。

- `void ForwardDynamic()`: 进行动态前向传播。

- `void ForwardCUDATensors()`: 使用 CUDA 进行前向传播。

- `Outputs& GetOutputs()`: 获取模型的输出数据。

- `~NeuralModel()`: 析构函数。

#### **私有成员变量**

- `std::unique_ptr<NeuralModelImpl> Model`: 存储模型的私有实现。

- `Inputs _input`: 存储当前输入数据。

- `Outputs _output`: 存储当前输出数据。

## 3. 代码依赖

该代码依赖以下 C++ 标准库和 PyTorch 组件：

- `<memory>`: 用于智能指针管理。

- `<vector>`: 用于存储数组数据。

- PyTorch C++ API: 需要正确安装 LibTorch，并支持 CUDA（如果使用 `ForwardCUDATensors`）。

## 4. 适用场景

该代码适用于以下场景：

- 在自动驾驶仿真中使用深度学习模型进行车辆动力学预测。

- 基于 PyTorch 进行 GPU 加速的神经网络推理。

- 处理车辆行驶过程中的力学数据和控制信号。

## 5. 许可证

本代码遵循 MIT 许可证，更多信息请参考 [MIT License](https://opensource.org/licenses/MIT)。


