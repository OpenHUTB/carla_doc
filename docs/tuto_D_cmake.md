## CMake命令介绍
参考[教程](https://subingwen.cn/cmake/CMake-primer/index.html) 。

### 参数解释
解释在`Util/BuildTools/*`目录下所使用到的各个脚本：

| 参数              | 含义         |
|-----------------|------------|
| -G %GENERATOR% %PLATFORM% | 指定编译器版本、平台 |
| -DCMAKE_BUILD_TYPE=Client | 指定构建类型 |
