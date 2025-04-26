
# Math 类说明文档

## 概述
`carla::geom::Math` 提供了一系列数学常量与常用数学运算函数，主要用于三维/二维向量运算、角度转换、距离计算、线性插值等。

---

## 模板常量

### **<font color="#7fb800">Pi</font>**
```cpp
template <typename T>
static constexpr T Pi();
```
- **模板参数**  
  - <font color="#f8805a">T</font>：必须为浮点类型  
- **返回值**：π（3.141592653589793…），类型为 <font color="#f8805a">T</font>

### **<font color="#7fb800">Pi2</font>**
```cpp
template <typename T>
static constexpr T Pi2();
```
- **模板参数**  
  - <font color="#f8805a">T</font>：必须为浮点类型  
- **返回值**：2π，即 2 × **<font color="#7fb800">Pi</font>**，类型为 <font color="#f8805a">T</font>

---

## 角度转换

### **<font color="#7fb800">ToDegrees</font>**
```cpp
template <typename T>
static constexpr T ToDegrees(T rad);
```
- **参数**  
  - <font color="#f8805a">rad</font>：弧度值  
- **返回值**：度数，等于 <font color="#f8805a">rad</font> × (180 / π)

### **<font color="#7fb800">ToRadians</font>**
```cpp
template <typename T>
static constexpr T ToRadians(T deg);
```
- **参数**  
  - <font color="#f8805a">deg</font>：度数值  
- **返回值**：弧度，等于 <font color="#f8805a">deg</font> × (π / 180)

---

## 范围与平方

### **<font color="#7fb800">Clamp</font>**
```cpp
template <typename T>
static T Clamp(T a, T min = T(0), T max = T(1));
```
- **参数**  
  - <font color="#f8805a">a</font>：待限制的值  
  - <font color="#f8805a">min</font>：下界，默认 0  
  - <font color="#f8805a">max</font>：上界，默认 1  
- **返回值**：将 <font color="#f8805a">a</font> 限制在 [<font color="#f8805a">min</font>,<font color="#f8805a">max</font>] 范围内的结果

### **<font color="#7fb800">Square</font>**
```cpp
template <typename T>
static T Square(const T &a);
```
- **参数**  
  - <font color="#f8805a">a</font>：输入值  
- **返回值**：<font color="#f8805a">a</font>²

---

## 向量运算

### **<font color="#7fb800">Cross</font>**
```cpp
static auto Cross(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：三维向量  
- **返回值**：叉积向量

### **<font color="#7fb800">Dot</font>**
```cpp
static auto Dot(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：三维向量  
- **返回值**：点积（标量）

### **<font color="#7fb800">Dot2D</font>**
```cpp
static auto Dot2D(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：仅取 x,y 分量  
- **返回值**：二维点积

---

## 距离计算

### **<font color="#7fb800">DistanceSquared</font>**
```cpp
static auto DistanceSquared(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>：起点向量  
  - <font color="#f8805a">b</font>：终点向量  
- **返回值**：两点间三维距离的平方

### **<font color="#7fb800">DistanceSquared2D</font>**
```cpp
static auto DistanceSquared2D(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>：起点向量  
  - <font color="#f8805a">b</font>：终点向量  
- **返回值**：两点间二维距离的平方

### **<font color="#7fb800">Distance</font>**
```cpp
static auto Distance(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>：起点向量  
  - <font color="#f8805a">b</font>：终点向量  
- **返回值**：两点间三维距离

### **<font color="#7fb800">Distance2D</font>**
```cpp
static auto Distance2D(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>：起点向量  
  - <font color="#f8805a">b</font>：终点向量  
- **返回值**：两点间二维距离

---

## 插值

### **<font color="#7fb800">LinearLerp</font>**
```cpp
static float LinearLerp(float a, float b, float f);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：端点值  
  - <font color="#f8805a">f</font>：插值因子 (0…1)  
- **返回值**：线性插值结果

---

## 复杂几何运算

### **<font color="#7fb800">GetVectorAngle</font>**
```cpp
static double GetVectorAngle(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：三维向量  
- **返回值**：向量 <font color="#f8805a">a</font> 与 <font color="#f8805a">b</font> 之间的夹角（弧度）

### **<font color="#7fb800">DistanceSegmentToPoint</font>**
```cpp
static std::pair<float, float> DistanceSegmentToPoint(
    const Vector3D &p,
    const Vector3D &v,
    const Vector3D &w);
```
- **参数**  
  - <font color="#f8805a">p</font>：要计算距离的点  
  - <font color="#f8805a">v</font>：线段起点  
  - <font color="#f8805a">w</font>：线段终点  
- **返回值**：`(沿线段的弧长距离, 点到线段的欧氏距离)`

### **<font color="#7fb800">DistanceArcToPoint</font>**
```cpp
static std::pair<float, float> DistanceArcToPoint(
    Vector3D p,
    Vector3D start_pos,
    float length,
    float heading,
    float curvature);
```
- **参数**  
  - <font color="#f8805a">p</font>：要计算距离的点  
  - <font color="#f8805a">start_pos</font>：圆弧起始位置  
  - <font color="#f8805a">length</font>：圆弧长度  
  - <font color="#f8805a">heading</font>：起始航向（弧度）  
  - <font color="#f8805a">curvature</font>：曲率  
- **返回值**：`(沿圆弧的弧长距离, 点到圆弧的最近距离)`

### **<font color="#7fb800">RotatePointOnOrigin2D</font>**
```cpp
static Vector3D RotatePointOnOrigin2D(Vector3D p, float angle);
```
- **参数**  
  - <font color="#f8805a">p</font>：二维点  
  - <font color="#f8805a">angle</font>：旋转角度（弧度）  
- **返回值**：绕原点旋转后的二维点

### **<font color="#7fb800">GetForwardVector</font>**
```cpp
static Vector3D GetForwardVector(const Rotation &rotation);
```
- **参数**  
  - <font color="#f8805a">rotation</font>：旋转信息  
- **返回值**：前向单位向量

### **<font color="#7fb800">GetRightVector</font>**
```cpp
static Vector3D GetRightVector(const Rotation &rotation);
```
- **参数**  
  - <font color="#f8805a">rotation</font>：旋转信息  
- **返回值**：右向单位向量

### **<font color="#7fb800">GetUpVector</font>**
```cpp
static Vector3D GetUpVector(const Rotation &rotation);
```
- **参数**  
  - <font color="#f8805a">rotation</font>：旋转信息  
- **返回值**：上向单位向量

---

## 范围生成

### **<font color="#7fb800">GenerateRange</font>**
```cpp
static std::vector<int> GenerateRange(int a, int b);
```
- **参数**  
  - <font color="#f8805a">a</font>：起始整数  
  - <font color="#f8805a">b</font>：结束整数  
- **返回值**：包含从 <font color="#f8805a">a</font> 到 <font color="#f8805a">b</font>（含）的所有整数的向量

# FDVector 头文件文档

该文档详细介绍 **<font color="#f8805a">FDVector</font>** 头文件中所有变量、构造函数、方法以及操作符重载。该结构在 [Carla Simulator](https://github.com/carla-simulator/carla) 项目中主要用于高效执行向量运算及类型转换，避免直接使用 Unreal Engine 的向量类型带来的性能损耗。

## 概述

**<font color="#f8805a">FDVector</font>** 是一个轻量级的三维向量结构，提供了向量的基本数学运算、类型转换以及便于调试的字符串输出功能。

## 构造函数

- **<font color="#7fb800">FDVector()</font>**  
  默认构造函数，将 **<font color="#f8805a">X</font>**、**<font color="#f8805a">Y</font>**、**<font color="#f8805a">Z</font>** 均初始化为 0.0.  
  **示例：**  
~~~cpp
FDVector vec1;
// vec1.X = 0.0, vec1.Y = 0.0, vec1.Z = 0.0
~~~
- **<font color="#7fb800">FDVector(float InX, float InY, float InZ)</font>**
使用单精度浮点数初始化向量。
**示例：**
~~~cpp
FDVector vec2(1.0f, 2.0f, 3.0f);
// vec2.X = 1.0, vec2.Y = 2.0, vec2.Z = 3.0
~~~
- **<font color="#7fb800">FDVector(double InX, double InY, double InZ)</font>**
使用双精度浮点数初始化向量。
**示例：**
~~~cpp
FDVector vec3(1.0, 2.0, 3.0);
// vec3.X = 1.0, vec3.Y = 2.0, vec3.Z = 3.0
~~~
- **<font color="#7fb800">FDVector(const FVector& V)</font>**
从 Unreal Engine 的 FVector 类型构造 <font color="#f8805a">FDVector</font>。
**示例：**
~~~cpp
FIntVector intVec(7, 8, 9);
FDVector vec5(intVec);
// vec5.X = 7.0, vec5.Y = 8.0, vec5.Z = 9.0
~~~


## 成员变量

- **<font color="#f8805a">X</font>**：表示向量在 X 轴的坐标。
- **<font color="#f8805a">Y</font>**：表示向量在 Y 轴的坐标。
- **<font color="#f8805a">Z</font>**：表示向量在 Z 轴的坐标。

**示例：**
~~~cpp
FDVector vec;
vec.X = 10.0;
vec.Y = 20.0;
vec.Z = 30.0;
~~~

## 主要方法

- **<font color="#7fb800">Size()</font>**
返回向量的长度（欧几里得范数）。
**示例：**
~~~cpp
FDVector vec(3.0, 4.0, 0.0);
double length = vec.Size();  // length = 5.0
~~~
**计算公式如下：**
$$
\|\mathbf{v}\|=\sqrt{x^2+y^2+z^2}
$$
- **<font color="#7fb800">SizeSquared()</font>**
返回向量长度的平方，避免了计算平方根的开销。
**示例：**
~~~cpp
FDVector vec(3.0, 4.0, 0.0);
double lengthSq = vec.SizeSquared();  // lengthSq = 25.0
~~~
- **<font color="#7fb800">Dist(const FDVector &V1, const FDVector &V2)</font>**
静态方法，计算两个向量之间的距离。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b(4.0, 6.0, 3.0);
double distance = FDVector::Dist(a, b);  // distance = 5.0
~~~
**计算公式如下：**
$$
d(\mathbf{v_1},\mathbf{v_2})=\sqrt{(x_2-x_1)^2+(y_2-y_1)^2+(z_2-z_1)^2}
$$
- **<font color="#7fb800">DistSquared(const FDVector &V1, const FDVector &V2)</font>**
静态方法，计算两个向量之间距离的平方。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b(4.0, 6.0, 3.0);
double distanceSq = FDVector::DistSquared(a, b);  // distanceSq = 25.0
~~~
- **<font color="#7fb800">ToFVector()</font>**
将 <font color="#f8805a">FDVector</font> 转换为 Unreal Engine 的 FVector 类型。
**示例：**
~~~cpp
FDVector vec(1.0, 2.0, 3.0);
FVector ueVec = vec.ToFVector();
// ueVec 与 vec 数值一致
~~~
- **<font color="#7fb800">ToString()</font>**
将向量格式化为字符串，输出格式为 "X=xx.xx Y=yy.yy Z=zz.zz"，方便调试。
**示例：**
~~~cpp
FDVector vec(1.2345, 2.3456, 3.4567);
FString str = vec.ToString();
// str 为 "X=1.23 Y=2.35 Z=3.46"
~~~
- **<font color="#7fb800">ToFIntVector()</font>**
将 <font color="#f8805a">FDVector</font> 转换为 Unreal Engine 的 FIntVector 类型，采用强制类型转换。
**示例：**
~~~cpp
FDVector vec(1.9, 2.9, 3.9);
FIntVector intVec = vec.ToFIntVector();
// intVec.X = 1, intVec.Y = 2, intVec.Z = 3
~~~

## 操作符重载

为了支持直观的向量运算，**<font color="#f8805a">FDVector</font>** 重载了以下操作符：
- **<font color="#7fb800">operator=</font>**
赋值操作符，将另一个 <font color="#f8805a">FDVector</font> 的值复制给当前对象。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b;
b = a;  // b 的值变为 (1.0, 2.0, 3.0)
~~~
- **<font color="#7fb800">operator==</font>**
比较两个 <font color="#f8805a">FDVector</font> 是否相等（各分量相等）。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b(1.0, 2.0, 3.0);
bool equal = (a == b);  // equal 为 true
~~~
- **<font color="#7fb800">operator+</font>**
实现向量加法，返回两个向量分量相加的结果。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b(4.0, 5.0, 6.0);
FDVector c = a + b;  // c 的值为 (5.0, 7.0, 9.0)
~~~
- **<font color="#7fb800">operator+=(float Scalar)</font>**
将标量 <font color="#00a6ed">Scalar</font> 累加到每个分量上。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
a += 2.0f;
// a 的值变为 (3.0, 4.0, 5.0)
~~~
- **<font color="#7fb800">operator-</font>**
实现向量减法，返回两个向量分量相减的结果。
**示例：**
~~~cpp
FDVector a(5.0, 7.0, 9.0);
FDVector b(1.0, 2.0, 3.0);
FDVector c = a - b;  // c 的值为 (4.0, 5.0, 6.0)
~~~
- <font color="#7fb800">operator-=(const FIntVector& V)</font>
  提供两种重载：

  1. 返回新的向量而不修改原向量。
  **示例：**
~~~cpp
FDVector a(5.0, 7.0, 9.0);
FIntVector intVec(1, 2, 3);
FDVector b = a -= intVec;  
// b 的值为 (4.0, 5.0, 6.0)，a 保持不变
~~~
  2. 修改当前向量，直接更新自身。
**示例：**
~~~cpp
FDVector a(5.0, 7.0, 9.0);
FIntVector intVec(1, 2, 3);
a -= intVec;
// a 的值更新为 (4.0, 5.0, 6.0)
~~~
- **<font color="#7fb800">operator/</font>**
实现向量除以标量，返回每个分量均除以 <font color="#00a6ed">Scale</font> 后的结果。
**示例：**
~~~cpp
FDVector a(4.0, 8.0, 12.0);
FDVector b = a / 2.0f;  
// b 的值为 (2.0, 4.0, 6.0)
~~~
- **<font color="#7fb800">operator</font>**
实现向量乘以标量，返回每个分量均乘以 <font color="#00a6ed">Scale</font> 后的结果。
**示例：**
~~~cpp
FDVector a(1.0, 2.0, 3.0);
FDVector b = a * 3.0f;  
// b 的值为 (3.0, 6.0, 9.0)
~~~
- **<font color="#7fb800">operator=(float Scale)</font>**
将当前向量的每个分量乘以标量 <font color="#00a6ed">Scale</font> 并更新自身。
**示例：**
~~~cpp
FDVector a(2.0, 3.0, 4.0);
a *= 2.0f;
// a 的值更新为 (4.0, 6.0, 8.0)
~~~
- **<font color="#7fb800">operator=(double Scale)</font>**
同样功能，支持双精度标量。
**示例：**
~~~cpp
FDVector a(2.0, 3.0, 4.0);
a *= 2.0;
// a 的值更新为 (4.0, 6.0, 8.0)
~~~
## 在 Carla 项目中的应用
在  [Carla Simulator](https://github.com/carla-simulator/carla) 项目中，<font color="#f8805a">FDVector</font> 的主要应用包括：

- 高效计算：用于执行频繁的向量运算（如位置计算、距离判定、碰撞检测等），减少直接使用 Unreal Engine 内置向量类可能带来的性能损耗。
- 类型转换：便于与 Unreal Engine 的 **FVector** 和 **FIntVector** 类型之间的互转，保持各模块间数据传递的一致性和高效性。

