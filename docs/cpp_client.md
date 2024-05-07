C++ 客户端示例
==================

此示例使用CARLA的C++API创建一个应用程序，以从C++连接和控制仿真器。

编译和运行
---------------

使用提供的 Makefile（仅限Linux）编译并运行示例。请注意，它希望有一个仿真器在2000端口运行。

```
make run
```

它的工作原理
------------

为了将应用程序链接到 LibCarla，我们需要使用与应用程序相同的编译器和配置来编译LibCarla。为此，我们生成一个 CMake 工具链文件，指定所需的编译器和标志

```cmake
# Example ToolChain.cmake
set(CMAKE_C_COMPILER /usr/bin/clang-8)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -DNDEBUG" CACHE STRING "" FORCE)
```

We pass this file to CMake when compiling LibCarla.client

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

This will generate the following structure at the provided install path

```
libcarla-install
├── include
│   ├── carla
│   ├── cephes
│   ├── pugixml
|   ├── ...
│   └── system
│       ├── boost
│       ├── recast
│       └── rpc
└── lib
    ├── libcarla_client.a
    ├── librpc.a
    ├── libboost_filesystem.a
    └── ...
```

Our application needs to be linked at minimum against `libcarla_client.a`,
`librpc.a`, `libRecast.a`, and `libDetour*.a`. If we make use of IO
functionality and/or image processing we would need to link against
`boost_filesystem`, `png`, `tiff`, and/or `jpeg`.

For more details take a look at the Makefile provided.
