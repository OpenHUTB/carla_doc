## Windows 下 OpenRadioss 的使用

### 测试实例
测试例子是 [车辆撞墙模拟](https://openradioss.atlassian.net/wiki/spaces/OPENRADIOSS/pages/47546369/HPC+Benchmark+Models#Taurus-10-Million-finite-elements) 的 [模型文件](https://openradioss.atlassian.net/wiki/download/attachments/47546369/Neon1m11_2017.zip?api=v2) 。

### 处理步骤
1. 模型初始化
```shell
starter_win64_sp.exe -i NEON1M11_0000.rad -nt 8
```

2. 引擎求解
```shell
engine_win64_sp.exe -i NEON1M11_0001.rad -nt 8
```
输出文件：NEON1M11A00{1-17}。

3. 将输出转化为可视化模型文件
```shell
anim_to_vtk_win64.exe NEON1M11A017 > view.vtk
```

4. 使用 [paraview](https://www.paraview.org/download/) 可视化模型文件
```shell
paraview view.vtk
```
并点击左下角`Properties`中的`Apply`。


#### 运行结果
![汽车碰撞](figure/crashing.gif)
