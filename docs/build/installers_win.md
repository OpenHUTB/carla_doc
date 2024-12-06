## Windows平台下的依赖安装脚本

这些脚本位于`Util/InstallersWin/`目录下，包括 `install_boost.bat`、`install_chrono.bat`、`install_gtest.bat`、`install_proj.bat`、`install_recast.bat`、`install_rpclib.bat`、`install_sqlite3.bat`、XML文档解析C语言版`install_xercesc.bat`、`install_zlib.bat`。

### 使用方法
脚本传入的命令相同，比如对于 install_boost 库的编译：
```shell
install_boost.bat --build-dir C:\buf --toolset msvc-14.2 --version 1.80.0 -j 4
```
- --build-dir：代码下载和生成安装文件的目录
- --toolset：编译所使用的编译器，msvc-14.2表示vs2019
- --version：表示所下载的软件的版本
- -j：使用多少核心进行编译
