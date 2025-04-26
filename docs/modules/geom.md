# Math 

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

# Vector3D.h

## 概述
`Vector3D` 类用于表示三维空间中的向量，包含坐标分量与丰富的向量运算接口，也支持与 UE4 的类型互转及 MsgPack 序列化。

---

## 公开数据成员

- <font color="#f8805a">x</font>：x 轴分量，默认为 0.0f  
- <font color="#f8805a">y</font>：y 轴分量，默认为 0.0f  
- <font color="#f8805a">z</font>：z 轴分量，默认为 0.0f  

---

## 构造函数

### **默认构造**  
```cpp
Vector3D();
```
- **说明**：将 <font color="#f8805a">x</font>, <font color="#f8805a">y</font>, <font color="#f8805a">z</font> 初始化为 0.0f。  
- **示例**：  
  ```cpp
  Vector3D v; // v = (0,0,0)
  ```

### **带参数构造**  
```cpp
Vector3D(float ix, float iy, float iz);
```
- **参数**  
  - <font color="#f8805a">ix</font>：x 分量  
  - <font color="#f8805a">iy</font>：y 分量  
  - <font color="#f8805a">iz</font>：z 分量  
- **示例**：  
  ```cpp
  Vector3D v(1.0f, 2.0f, -3.0f); // v = (1,2,-3)
  ```

---

## 基本方法

### **<font color="#7fb800">SquaredLength</font>font**  
```cpp
float SquaredLength() const;
```
- **返回值**：向量长度的平方，即 x² + y² + z²。  
- **示例**：  
  ```cpp
  Vector3D v(1,2,2);
  float sq = v.SquaredLength(); // sq == 9.0f
  ```

### **<font color="#7fb800">Length</font>**  
```cpp
float Length() const;
```
- **返回值**：向量的欧氏长度，等于 sqrt(SquaredLength())。  
- **示例**：  
  ```cpp
  Vector3D v(1,2,2);
  float len = v.Length(); // len == 3.0f
  ```

### **<font color="#7fb800">SquaredLength2D</font>**  
```cpp
float SquaredLength2D() const;
```
- **返回值**：仅 x,y 分量的长度平方。  
- **示例**：  
  ```cpp
  Vector3D v(3,4,5);
  float sq2 = v.SquaredLength2D(); // sq2 == 25.0f
  ```

### **<font color="#7fb800">Length2D</font>**  
```cpp
float Length2D() const;
```
- **返回值**：仅 x,y 分量的二维长度。  
- **示例**：  
  ```cpp
  Vector3D v(3,4,5);
  float len2 = v.Length2D(); // len2 == 5.0f
  ```

### **<font color="#7fb800">Abs</font>**  
```cpp
Vector3D Abs() const;
```
- **返回值**：各分量取绝对值后的新向量。  
- **示例**：  
  ```cpp
  Vector3D v(-1,-2,3);
  Vector3D a = v.Abs(); // a = (1,2,3)
  ```

### **<font color="#7fb800">MakeUnitVector</font>**  
```cpp
Vector3D MakeUnitVector() const;
```
- **说明**：将向量标准化为单位向量，长度必须大于机器 epsilon。  
- **示例**：  
  ```cpp
  Vector3D v(0,3,4);
  Vector3D u = v.MakeUnitVector(); // u ≈ (0,0.6,0.8)
  ```

### **<font color="#7fb800">MakeSafeUnitVector</font>**  
```cpp
Vector3D MakeSafeUnitVector(const float epsilon) const;
```
- **说明**：当长度小于 epsilon 时不归一化，否则归一化。  
- **示例**：  
  ```cpp
  Vector3D v(0.0001f,0,0);
  Vector3D s = v.MakeSafeUnitVector(1e-3f); // s == v （不变）
  ```

---

## 算术运算符

### **operator+= / operator+**  
```cpp
Vector3D &operator+=(const Vector3D &rhs);
friend Vector3D operator+(Vector3D lhs, const Vector3D &rhs);
```
- **示例**：  
  ```cpp
  Vector3D a(1,2,3), b(4,5,6);
  Vector3D c = a + b; // c = (5,7,9)
  a += b;             // a = (5,7,9)
  ```

### **operator-= / operator-**  
```cpp
Vector3D &operator-=(const Vector3D &rhs);
friend Vector3D operator-(Vector3D lhs, const Vector3D &rhs);
```
- **示例**：  
  ```cpp
  Vector3D a(5,5,5), b(1,2,3);
  Vector3D c = a - b; // c = (4,3,2)
  a -= b;             // a = (4,3,2)
  ```

### **operator*= / operator***  
```cpp
Vector3D &operator*=(float rhs);
friend Vector3D operator*(Vector3D lhs, float rhs);
friend Vector3D operator*(float lhs, Vector3D rhs);
```
- **示例**：  
  ```cpp
  Vector3D v(1,2,3);
  Vector3D w = v * 2.0f; // w = (2,4,6)
  v *= 3.0f;             // v = (3,6,9)
  ```

### **operator/= / operator/**  
```cpp
Vector3D &operator/=(float rhs);
friend Vector3D operator/(Vector3D lhs, float rhs);
friend Vector3D operator/(float lhs, Vector3D rhs);
```
- **示例**：  
  ```cpp
  Vector3D v(4,6,8);
  Vector3D u = v / 2.0f; // u = (2,3,4)
  ```

---

## 比较运算符

### **operator== / operator!=**  
```cpp
bool operator==(const Vector3D &rhs) const;
bool operator!=(const Vector3D &rhs) const;
```
- **示例**：  
  ```cpp
  Vector3D a(1,1,1), b(1,1,1), c(2,2,2);
  bool eq = (a == b); // true
  bool ne = (a != c); // true
  ```

---

## UE4 互转（仅在 UE4 模式开启时）

```cpp
Vector3D ToMeters() const;
Vector3D ToCentimeters() const;
FVector ToFVector() const;
```
- **示例**：  
  ```cpp
  Vector3D v(100,200,300);
  FVector f = v.ToFVector();     // f = (100,200,300)
  Vector3D m = v.ToMeters();     // m = (1,2,3)
  Vector3D cm = m.ToCentimeters(); // cm = (100,200,300)
  ```

---

## MsgPack 序列化

```cpp
template <typename Packer>
void msgpack_pack(Packer& pk) const;

void msgpack_unpack(clmdep_msgpack::object const& o);

template <typename MSGPACK_OBJECT>
void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& sneaky_variable_that_shadows_z) const;
```
- **说明**：使用内置宏 `make_define_array(x,y,z)` 实现序列化/反序列化。  
- **示例**：  
  ```cpp
  // pack
  clmdep_msgpack::sbuffer sbuf;
  msgpack::pack(sbuf, Vector3D(1,2,3));
  // unpack
  auto obj = msgpack::unpack(sbuf.data(), sbuf.size()).get();
  Vector3D v;
  v.msgpack_unpack(obj);
  ```

---