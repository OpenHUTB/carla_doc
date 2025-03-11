# DVector.h 文档

**Dvector.h 中定义了FDVector** 结构体，用于表示三维向量，其各分量为双精度浮点数。

## 成员变量

- **<font color="#f8805a">X</font>**：向量的 X 分量，初始值为 0.0
- **<font color="#f8805a">Y</font>**：向量的 Y 分量，初始值为 0.0
- **<font color="#f8805a">Z</font>**：向量的 Z 分量，初始值为 0.0

## 构造函数

提供多种构造方式：

- **默认构造函数**：将所有分量初始化为 0.0
- **构造函数（float/double 参数）**：根据传入值初始化各分量
- **从 FVector/FIntVector 构造**：便于与虚幻引擎内的向量类型转换

## 方法

### **<font color="#7fb800">Size</font>**

计算向量的模长，公式为：
$$
\text{Size} = \sqrt{\mathcal{X}^2 + \mathcal{Y}^2 + \mathcal{Z^2}}
$$

### **<font color="#7fb800">SizeSquared</font>**

返回向量模长的平方：
$$
\text{SizeSquared} = \mathcal{X}^2 + \mathcal{Y}^2 + \mathcal{Z}^2
$$

### 静态方法

- **<font color="#7fb800">Dist</font>**：计算两个向量间距离  
  $$\text{Dist} = \sqrt{\text{DistSquared}(\mathcal{V_1},\mathcal{V_2})}$$
- **<font color="#7fb800">DistSquared</font>**：计算两个向量间距离的平方

### 转换方法

- **<font color="#7fb800">ToFVector</font>**：转换为 FVector 类型
- **<font color="#7fb800">ToFIntVector</font>**：转换为 FIntVector 类型（通过类型转换）

## 运算符重载

支持常用的向量运算：

- **<font color="#7fb800">operator=</font>**：赋值操作
- **<font color="#7fb800">operator==</font>**：判断向量是否相等
- **<font color="#7fb800">operator+</font>**、**<font color="#7fb800">operator-</font>**：实现向量加减
- **<font color="#7fb800">operator*</font>**、**<font color="#7fb800">operator/</font>**：标量乘除运算
- 以及对应的复合赋值运算符（如 **<font color="#7fb800">operator+=</font>**、**<font color="#7fb800">operator*=</font>** 等）

## 使用示例

```cpp
// 默认构造，所有分量为 0.0
FDVector vec1;

// 指定分量构造
FDVector vec2(1.0, 2.0, 3.0);

// 计算模长
double len = vec2.Size();

// 向量加法
FDVector vec3 = vec1 + vec2;
```
