## CMake命令介绍
参考[教程](https://subingwen.cn/cmake/CMake-primer/index.html) 。

### 参数解释
解释在`Util/BuildTools/*`目录下所使用到的各个脚本：

| 参数              | 含义                                                          |
|-----------------|-------------------------------------------------------------|
| `-G %GENERATOR% %PLATFORM%` | 指定编译器版本、平台                                                  |
| `-DCMAKE_BUILD_TYPE=Client` | 指定构建类型                                                      |
| `-DCMAKE_CXX_FLAGS_RELEASE="/MD /MP"` | 为`/MD /MP`构建设置C++编译器优化级别                                    |
| `-DCMAKE_INSTALL_PREFIX` | 指定这些文件应该拷贝到哪个根目录（PythonAPI里）                                |
| `--build .` | 对生成好的项目进行编译构建，`.`指的是生成好的build tree的路径                       |
| `--config Release` | 构建后会按install规则安装一份，同时在`-DCMAKE_INSTALL_PREFIX`指定的路径下也会安装一份。 |
| `findstr /V "Up-to-date:"` | 对打印印不包含相符项目的行。                                             |
