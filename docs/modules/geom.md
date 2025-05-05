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

设$a=[x_1,y_1,z_1]$,$b=[x_2,y_2,z_2]$那么它们的叉积$a\times b$为：
$$
a\times b=\begin{pmatrix}y_1z_2-z_1y_2\\z_1x_2-x_1z_2\\x_1y_2-y_1x_2\end{pmatrix}
$$

### **<font color="#7fb800">Dot</font>**

```cpp
static auto Dot(const Vector3D &a, const Vector3D &b);
```
- **参数**  
  - <font color="#f8805a">a</font>, <font color="#f8805a">b</font>：三维向量  
- **返回值**：点积（标量）

向量的点积计算公式为:
$$
a\cdot b=\sum_i^n a_ib_i
$$


其中$n$是向量的维度。

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

设：

- 点 $p$
- 线段起点 $v$
- 线段终点 $w$

线段向量为：$\vec{d} = \vec{w} - \vec{v}$  
向量 $\vec{vp} = \vec{p} - \vec{v}$

**1. 计算投影参数 $t$**

该参数 $t$ 表示点 $p$ 在 $\vec{d}$ 上的投影比例位置：

$$
t = \frac{(\vec{p} - \vec{v}) \cdot (\vec{w} - \vec{v})}{\|\vec{w} - \vec{v}\|^2}
$$

然后将 $t$ 限制在 $[0,1]$ 范围内（因为只在线段上）：

```cpp
t = Clamp(t, 0.0f, 1.0f);
```

**2. 计算投影点坐标**

$$
\vec{proj} = \vec{v} + t \cdot (\vec{w} - \vec{v})
$$

**3. 返回结果**

- 第一项：投影点到线段起点的距离（线段弧长）

  $$
  \text{distance\_along\_segment} = t \cdot \|\vec{w} - \vec{v}\|
  $$

- 第二项：点 $p$ 到投影点的欧氏距离

  $$
  \text{distance\_to\_segment} = \|\vec{p} - \vec{proj}\|
  $$

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

**1. 处理特殊情况**

若曲率为 0，圆弧退化为直线段，调用 `DistanceSegmentToPoint` 即可：
```cpp
if (std::abs(curvature) < 1e-6) return DistanceSegmentToPoint(...);
```

**2. 圆心计算**

设圆弧是逆时针方向（曲率为正），那么：

- 曲率半径 $R = \frac{1}{|curvature|}$
- 圆心在起点的垂直方向上：

$$
center = start\_pos + R \cdot \begin{bmatrix} -\sin(heading) \\ \cos(heading) \end{bmatrix}
\quad \text{（当 curvature > 0）}
$$

**3. 将点 $p$ 映射到极坐标下**

- 向量 $\vec{v} = p - center$
- 距离为 $r = \|\vec{v}\|$
- 角度 $\theta_p = \arctan2(p_y - c_y, p_x - c_x)$

**4. 计算圆弧起始角度 $\theta_0$**

由于起始点位于弧上的起点，其角度为：

$$
\theta_0 = \arctan2(start\_pos_y - c_y, start\_pos_x - c_x)
$$

**5. 弧长范围映射（0 到 total_arc_angle）**

总的圆心角：

$$
\Delta \theta = \frac{length}{R}
$$

点到起点的圆心角（根据方向判断）：

$$
\Delta \theta_p = \text{WrapAngle}(\theta_p - \theta_0)
$$

- 如果 $curvature > 0$，逆时针方向；否则顺时针。

投影点是否在圆弧范围内需判断 $0 \leq \Delta \theta_p \leq \Delta \theta$

**6. 计算投影点坐标**
$$
proj = center + R \cdot \begin{bmatrix} \cos(\theta_p) \\ \sin(\theta_p) \end{bmatrix}
$$

**7. 计算结果**

- 沿弧长距离：

  $$
  s = R \cdot \Delta \theta_p
  $$

- 欧氏距离：

  $$
  d = \|\vec{p} - \vec{proj}\|
  $$

若超出圆弧范围，则与端点距离更近，返回：

```cpp
Distance(p, start_pos) 或 Distance(p, arc_end)
```

---

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

# Vector3D

## 概述
`carla::geom::Vector3D` 类用于表示三维空间中的向量，包含坐标分量与丰富的向量运算接口，也支持与 UE4 的类型互转及 MsgPack 序列化。

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
- **说明**：将向量标准化为单位向量。  
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

# Mesh

## 概述
`carla::geom::Mesh` 类是一个网格数据容器，提供验证、构建、导出以及与 UE4 类型互转的功能。支持顶点、法线、索引、UV 和材质分组的管理。

---

## 类型别名

- **vertex_type**：顶点类型，`Vector3D`  
- **normal_type**：法线类型，`Vector3D`  
- **index_type**：索引类型，`size_t`  
- **uv_type**：纹理坐标类型，`Vector2D`  
- **material_type**：材质类型，`MeshMaterial`

---

## 构造函数

### **Mesh(...)**
```cpp
Mesh(
  const std::vector<vertex_type> &vertices = {},
  const std::vector<normal_type> &normals = {},
  const std::vector<index_type> &indexes = {},
  const std::vector<uv_type> &uvs = {}
);
```
- **说明**：可选传入初始顶点、法线、索引和 UV 列表。  
- **示例**：
  ```cpp
  std::vector<Vector3D> verts = { {0,0,0}, {1,0,0}, {0,1,0} };
  Mesh mesh(verts);
  ```

---

## 验证方法

### **IsValid**
```cpp
bool IsValid() const;
```
- **说明**：  
  1. 顶点列表非空；  
  2. 索引数若非零必须为 3 的倍数；  
  3. 所有开启的材质必须正确关闭。  
- **示例**：
  ```cpp
  if (!mesh.IsValid()) {
    std::cerr << "Invalid mesh!" << std::endl;
  }
  ```

---

## 网格构建方法

### **AddTriangleStrip**
```cpp
void AddTriangleStrip(const std::vector<vertex_type> &vertices);
```
- **说明**：按三角带（Triangle Strip）方式添加顶点，依次生成面。  
- **示例**：
  ```cpp
  mesh.AddTriangleStrip({{0,0,0},{1,0,0},{1,1,0},{0,1,0}});
  ```

### **AddTriangleFan**
```cpp
void AddTriangleFan(const std::vector<vertex_type> &vertices);
```
- **说明**：按三角扇（Triangle Fan）方式添加顶点，第一个顶点为中心。  
- **示例**：
  ```cpp
  mesh.AddTriangleFan({{0,0,0},{1,0,0},{1,1,0},{0,1,0}});
  ```

### **AddVertex / AddVertices**
```cpp
void AddVertex(vertex_type vertex);
void AddVertices(const std::vector<vertex_type> &vertices);
```
- **说明**：将顶点附加到内部列表。  
- **示例**：
  ```cpp
  mesh.AddVertex({0,0,1});
  mesh.AddVertices({{1,0,1},{0,1,1}});
  ```

### **AddNormal**
```cpp
void AddNormal(normal_type normal);
```
- **说明**：将法线附加到内部列表。  

### **AddIndex**
```cpp
void AddIndex(index_type index);
```
- **说明**：将单个面索引附加到内部列表。  

### **AddUV / AddUVs**
```cpp
void AddUV(uv_type uv);
void AddUVs(const std::vector<uv_type> &uvs);
```
- **说明**：添加纹理坐标 (u,v)。  

### **AddMaterial / EndMaterial**
```cpp
void AddMaterial(const std::string &material_name);
void EndMaterial();
```
- **说明**：开始／结束一个材质区间，记录当前索引范围。  
- **示例**：
  ```cpp
  mesh.AddMaterial("matA");
  // ... 添加三角面 ...
  mesh.EndMaterial();
  ```

---

## 导出方法

### **GenerateOBJ**
```cpp
std::string GenerateOBJ() const;
```
- **说明**：导出为标准 OBJ 格式：  
  - 顶点行：`v x y z`  
  - UV 行：`vt u v`  
  - 法线行：`vn x y z`  
  - 面行：`f i j k`  

### **GenerateOBJForRecast**
```cpp
std::string GenerateOBJForRecast() const;
```
- **说明**：为 Recast 库导出 OBJ，交换 y/z 并反转面顶点顺序。

### **GeneratePLY**
```cpp
std::string GeneratePLY() const;
```
- **说明**：导出为 PLY 格式（部分实现）。

---

## 访问方法

### **GetVertices / GetVerticesNum**
```cpp
const std::vector<vertex_type> &GetVertices() const;
size_t GetVerticesNum() const;
```

### **GetNormals**
```cpp
const std::vector<normal_type> &GetNormals() const;
```

### **GetIndexes / GetIndexesNum**
```cpp
const std::vector<index_type> &GetIndexes() const;
size_t GetIndexesNum() const;
```

### **GetUVs**
```cpp
const std::vector<uv_type> &GetUVs() const;
```

### **GetMaterials**
```cpp
const std::vector<material_type> &GetMaterials() const;
```

### **GetLastVertexIndex**
```cpp
size_t GetLastVertexIndex() const;
```
- **说明**：返回当前顶点总数。

---

## 合并网格

### **ConcatMesh**
```cpp
Mesh& ConcatMesh(const Mesh &rhs, int num_vertices_to_link);
```
- **说明**：将 `rhs` 网格合并到当前网格，并在相交处生成桥接三角面。  

### **operator+= / operator+**
```cpp
Mesh& operator+=(const Mesh &rhs);
friend Mesh operator+(const Mesh &lhs, const Mesh &rhs);
```
- **说明**：无桥接直接拼接顶点、法线、索引、UV 和材质区间。

---

## UE4 类型转换（仅在 LIBCARLA_INCLUDED_FROM_UE4 定义时）

### **operator FProceduralCustomMesh() const**
```cpp
operator FProceduralCustomMesh() const;
```
- **说明**：  
  - 顶点从米转换到厘米；  
  - 法线重新计算：  
    \[
      \mathbf{N} = \mathrm{normalize}\bigl((\mathbf{V}_1-\mathbf{V}_0)\times(\mathbf{V}_2-\mathbf{V}_0)\bigr)
    \]
  - UV 坐标保留 (x,y)。  
- **示例**：
  ```cpp
  FProceduralCustomMesh ue_mesh = mesh;
  ```

---

# Simplification

## 概述
`carla::geom::Simplification` 类用于对 `carla::geom::Mesh` 网格模型执行简化操作，通过指定简化比率，将三角形数减少到原始数量的若干百分比，从而降低模型复杂度。

---

## 构造函数

### **<font color="#7fb800">Simplification()</font>**
```cpp
Simplification() = default;
```
- **说明**：默认构造，无需初始化即可使用  
- **示例**：
  ```cpp
  carla::geom::Simplification sim; // 使用默认简化率（未指定）
  ```

### **<font color="#7fb800">Simplification(float simplificationrate)</font>**
```cpp
Simplification(float simplificationrate);
```
- **参数**  
  - **<font color="#f8805a">simplificationrate</font>**：目标简化比率（0.0–1.0），如 0.5 表示保留 50% 三角形  
- **示例**：
  ```cpp
  carla::geom::Simplification sim(0.25f); // 保留 25% 的三角形
  ```

---

## 公开数据成员

- **<font color="#f8805a">simplification_percentage</font>**  
  
  - 存储简化比率（0–1 之间的浮点数）  
  - **示例**：
    
    ```cpp
    float rate = sim.simplification_percentage; // 获取当前简化比率
    ```

---

## 方法

### **<font color="#7fb800">Simplificate</font>**
```cpp
void Simplificate(const std::unique_ptr<geom::Mesh>& pmesh);
```
- **说明**：对传入的 `Mesh` 对象执行网格简化，简化后会修改其顶点与索引数据。  
- **参数**  
  
  - **<font color="#f8805a">pmesh</font>**：指向待简化网格的智能指针引用  
- **示例**：
  ```cpp
  #include "carla/geom/Simplification.h"
  #include "carla/geom/Mesh.h"
  
  // 假设已创建或加载了一个 Mesh 对象
  std::unique_ptr<carla::geom::Mesh> mesh = LoadYourMesh();
  carla::geom::Simplification sim(0.5f);    // 保留 50% 三角形
  sim.Simplificate(mesh);                   // 执行简化
  // 之后 mesh->GetVertices() 和 mesh->GetIndexes() 已被更新为简化后的数据
  ```

---

# RTree

## 简介
R-树（R-Tree）是一种用于空间索引的树形结构，通过将空间中相近对象聚合成最小外接矩形（MBR）构建索引，广泛应用于点云、线段云等三维几何对象的快速邻近查询与碰撞检测。

RTree包含两个模板类：
- `PointCloudRtree<T>`：支持对点云的插入、k-NN 查询、带过滤器查询；
- `SegmentCloudRtree<T>`：支持对线段云的插入、k-NN 查询、交集查询。

---

## 欧几里得距离公式

对任意两个 $n$ 维点 $\mathbf{p}=(p_1,p_2,\dots,p_n)$ 和 $\mathbf{q}=(q_1,q_2,\dots,q_n)$，欧几里得距离定义如下：

$$
d(\mathbf{p}, \mathbf{q}) = \sqrt{ \sum_{i=1}^n (p_i - q_i)^2 }
$$

---

## 类定义：PointCloudRtree

```cpp
template <typename T, size_t Dimension = 3>
class PointCloudRtree;
```

- `T`：元素类型；
- `Dimension`：空间维度，默认值为 `3`。

### 成员类型

```cpp
using BPoint = boost::geometry::model::point<float, Dimension>;
using TreeElement = std::pair<BPoint, T>;
```

### 方法说明

#### InsertElement

```cpp
void InsertElement(const BPoint &point, const T &element);
void InsertElement(const TreeElement &element);
```

- 插入一个点或点-元素对到 R 树中。

#### InsertElements

```cpp
void InsertElements(const std::vector<TreeElement> &elements);
```

- 批量插入元素。

#### GetNearestNeighbours

```cpp
std::vector<TreeElement> GetNearestNeighbours(const BPoint &point, size_t number_neighbours = 1) const;
```

- 返回距离给定点最近的 $k$ 个邻居（默认 $k=1$）。

#### GetNearestNeighboursWithFilter

```cpp
template <typename Filter>
std::vector<TreeElement> GetNearestNeighboursWithFilter(const BPoint &point, Filter filter, size_t number_neighbours = 1) const;
```

- 在返回最近邻的基础上，添加用户自定义过滤条件。

#### GetTreeSize

```cpp
size_t GetTreeSize() const;
```

- 返回当前树中包含的元素数量。

---

## 类定义：`SegmentCloudRtree`

```cpp
template <typename T, size_t Dimension = 3>
class SegmentCloudRtree;
```

### 成员类型

```cpp
using BPoint = boost::geometry::model::point<float, Dimension>;
using BSegment = boost::geometry::model::segment<BPoint>;
using TreeElement = std::pair<BSegment, std::pair<T, T>>;
```

### 方法说明

#### InsertElement

```cpp
void InsertElement(const BSegment &segment, const T &element_start, const T &element_end);
void InsertElement(const TreeElement &element);
```

- 插入一条线段以及其两端关联的数据。

#### InsertElements

```cpp
void InsertElements(const std::vector<TreeElement> &elements);
```

- 批量插入线段。

#### GetNearestNeighbours

```cpp
template <typename Geometry>
std::vector<TreeElement> GetNearestNeighbours(const Geometry &geometry, size_t number_neighbours = 1) const;
```

- 返回与几何体最邻近的线段。

#### GetNearestNeighboursWithFilter

```cpp
template <typename Geometry, typename Filter>
std::vector<TreeElement> GetNearestNeighboursWithFilter(const Geometry &geometry, Filter filter, size_t number_neighbours = 1) const;
```

- 返回最邻近的线段，且满足过滤条件。

#### GetIntersections

```cpp
template <typename Geometry>
std::vector<TreeElement> GetIntersections(const Geometry &geometry) const;
```

- 查询与几何体相交的线段集合。注意：Boost Geometry 暂不支持 3D 线段与线段精确交集判断，结果可能不完全准确。

#### GetTreeSize

```cpp
size_t GetTreeSize() const;
```

- 返回线段树的大小。

---