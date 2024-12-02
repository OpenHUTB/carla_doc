## C++

在源代码的根目录下运行`doxygen`将在`Doxygen/html`目录下生成html文件，[文档主页](https://openhutb.github.io/carla_cpp/) 位于。

[所有类的继承关系](https://openhutb.github.io/carla_cpp/inherits.html) 。

!!! 注意
    C++设计的哲学：Programmer needs to be trusted.（程序员应该被信任），是一种能够直接被编译成机器码的语言。

### LibCarla

### 客户端

客户端连接不使用 Nagle 算法，将Linux上的同步模式速度提高了约3倍。
```cpp
_socket.set_option(boost::asio::ip::tcp::no_delay(true));
```

!!! 笔记
    Nagle算法是一种用于提高TCP/IP网络性能的算法，它通过减少数据包的发送量来减少网络拥堵。Nagle算法的主要目的是合并一定数量的输出数据后一次性提交，以避免网络中充斥着许多小数据块。当TCP报文承载的数据量非常小时，例如几个字节，整个网络的效率会很低，因为每个TCP报文中都会包含大量的头部信息，而有效数据占有的比重就会非常低。Nagle算法要求，当一个TCP连接中有在传数据（已经发出但还未确认的数据）时，小于MSS（最大分段大小）的报文段就不能被发送，直到所有的在传数据都收到了ACK。同时收到ACK后，TCP还不会马上就发送数据，会收集小包合并一起发送。然而，Nagle算法并不适用于所有情况，在一些要求低延迟的应用程序中（例如即时通讯应用），Nagle算法的规定是不易被接受的，因此需要设置 `TCP_NODELAY` 或者 `TCP_CORK` 关闭Nagle算法。

[C++客户端连接示例](cpp_client.md) 只支持在 Linux 上运行。


## 概念

### DDS
DDS（Data Distribution Service），即 [数据分发服务](https://www.cnblogs.com/tomato-haha/p/17696510.html) ，是OMG（Object Management Group）对象管理组织发布的分布式通信规范，采用订阅发布模型，以中间件的形式提供通信服务，并提供QoS（Quality of Service）策略，保障数据实时、高效、灵活的分发。


## 问题
* VS 提示无法启动程序“ \DEBUG\ALL-BUILD”,系统找不到指定的文件

解决方案是：项目 —> 设置为启动项目。


* 中文乱码

解决：需要将其他文件的编码方式转换为UTF-8编码，vs2019参考 [链接](https://www.cnblogs.com/blossomwei/p/17866621.html) 进行转换。

* vs2019中调式或者运行按钮是灰色的

解决：需要在工具栏中选中当前的运行的文件。

## 参考
* [CMake 中文教程](https://cmake-doc.readthedocs.io/zh-cn/latest/guide/tutorial/index.html) 
* [vs2019 CMake 教程](https://learn.microsoft.com/zh-cn/cpp/build/cmake-projects-in-visual-studio?view=msvc-160) ，下载 CMake 3.26 的示例代码。
* [C++ API文档](https://cplusplus.com/reference/string/stod/) 
* [C++ 教程](https://zh.cppreference.com/w/cpp)
* [CMake 链接权限](https://zhuanlan.zhihu.com/p/663367171) ，区分PRIVATE、PUBLIC与INTERFACE。