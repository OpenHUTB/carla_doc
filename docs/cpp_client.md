C++ 客户端示例
==================

此示例使用 Carla 的C++API创建一个应用程序，以从C++连接和控制仿真器。

编译和运行
---------------

使用提供的 Makefile（仅限Linux）编译并运行示例。请注意，它希望有一个仿真器在2000端口运行。（[Windows客户端cmake文件](https://github.com/OpenHUTB/carla_doc/tree/master/src/cmake/CMakeLists.txt) ）

```
make run
```

它的工作原理
------------

为了将应用程序链接到 LibCarla，我们需要使用与应用程序相同的编译器和配置来编译 LibCarla。为此，我们生成一个 CMake 工具链文件，指定所需的编译器和标志

```cmake
# ToolChain.cmake 示例
set(CMAKE_C_COMPILER /usr/bin/clang-8)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -DNDEBUG" CACHE STRING "" FORCE)
```

编译LibCarla.client时，我们将此文件传递给CMake

```sh
cd /path/to/carla-root-folder

make setup

cd /path/to/build-folder

cmake \
    -G "Ninja" \
    -DCMAKE_BUILD_TYPE=Client \
    -DLIBCARLA_BUILD_RELEASE=ON \
    -DLIBCARLA_BUILD_DEBUG=OFF \
    -DLIBCARLA_BUILD_TEST=OFF \
    -DCMAKE_TOOLCHAIN_FILE=/path/to/ToolChain.cmake \
    -DCMAKE_INSTALL_PREFIX=/path/to/install-folder \
    /path/to/carla-root-folder

ninja
ninja install
```

这将在提供的安装路径上生成以下结构

```
libcarla-install
├── include
│ ├── carla
│ ├── cephes
│ ├── pugixml
|   ├── ...
│ └── system
│     ├── boost
│     ├── recast
│     └── rpc
└── lib
    ├── libcarla_client.a
    ├── librpc.a
    ├── libboost_filesystem.a
    └── ...
```

我们的应用程序至少需要链接到`libcarla_client.a`，`librpc.a`、`libRecast.a`和`libDetour*.a`。如果我们利用IO我们需要链接的功能和/或图像处理`boost_filesystem`、`png`、`tiff`和/或`jpeg`。

有关更多详细信息，请查看提供的 Makefile。
