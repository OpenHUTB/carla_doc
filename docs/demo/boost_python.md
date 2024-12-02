## [VS2019调用boost.Python](https://developer.aliyun.com/article/1260043)

### 生成可执行文件
1. 新建 Cmake 工程；
2. 输入以下代码：
```cpp
#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/to_python_converter.hpp>
#include <iostream>
using namespace std;
void dummyFunc() {
	cout << "call OK!" << endl;
}
int main()
{
	dummyFunc();
	return 0;
}
```

3. 在`CMakeLists.txt`中添加头文件目录和库文件目录
```text
# 添加头文件目录
include_directories("D:/work/workspace/carla/Build/boost-1.80.0-install/include")
include_directories("C:/Users/Administrator/AppData/Local/Programs/Python/Python37/include")
# 添加库文件目录
link_directories("C:/Users/Administrator/AppData/Local/Programs/Python/Python37/libs")
# 还是缺少 libboost_python37-vc142-mt-gd-x64-1_80.lib
link_directories("D:/work/workspace/carla/Build/boost-1.80.0-install/lib_no_prefix")
```

4. 到boost安装目录下，进入 lib文件夹下，找到前缀是`libboost_python37-vc142-mt-x64-1_80.lib`文件（前缀有lib），将其前缀的lib删除即可（即重命名为`boost_python37-vc142-mt-x64-1_80.lib`）。

5. 生成解决方案，会生成exe文件，如果不可以在vs中直接执行，将工具栏中的`当前文档`改为`*.exe`。


### 生成Python调用的库

1. 新建`hello_exe.cpp`，注意文件名必须为模块名：
```cpp
// 参考：https://developer.aliyun.com/article/1260043

// 当引入 #include <boost/python/xxx> 时，Boost 会默认链接 boost_python 动态链接库，
// 如果我们想要链接静态链接库，就需要在 include 之前加上 #define BOOST_PYTHON_STATIC_LIB
// 指定链接 boost_python 的静态库，而不是默认的动态库。
#define BOOST_PYTHON_STATIC_LIB

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/to_python_converter.hpp>
#include <iostream>

using namespace std;

char const* greet()
{
	return "hello, world";
}

BOOST_PYTHON_MODULE(hello_ext)
{
	using namespace boost::python;
	def("greet", greet);
}
```

2. 新建`CMakeLists.txt`：
```shell
cmake_minimum_required (VERSION 3.8)

# 添加头文件目录
include_directories("D:/work/workspace/carla/Build/boost-1.80.0-install/include")
include_directories("C:/Users/Administrator/AppData/Local/Programs/Python/Python37/include")
# 添加库文件目录
link_directories("C:/Users/Administrator/AppData/Local/Programs/Python/Python37/libs")
# 还是缺少 libboost_python37-vc142-mt-gd-x64-1_80.lib
link_directories("D:/work/workspace/carla/Build/boost-1.80.0-install/lib_no_prefix")

# 将源代码添加到此项目的可执行文件。
# add_executable (boost_demo "boost_demo.cpp" "boost_demo.h")

# # 生成动态库(共享库)共享库则只有一个副本
add_library(hello_ext SHARED hello_ext.cpp)
#(boost_demo 是库的名字)，这条命令告诉cmake，我们想把 boost_demo.cpp编译成一个叫作“boost_demo”的库。
set_target_properties(hello_ext PROPERTIES SUFFIX ".pyd")
```
构建后会生成`hello_ext.pyd`。


3. 在生成的`.pyd`目录下新建`hello.py`
```python
import hello_ext

print(hello_ext.greet())
```

### 从Python中调试*.cpp
1. 在原始.py脚本中添加打印进程号的代码，并使调试停止于cpp函数调用之前。
```python
import os

# 获取当前进程ID（便于调试）
process_id = os.getpid()
print("Current process id is: ", process_id)

import hello_ext

print(hello_ext.greet())
```

2. 在visual studio 中添加断点，并附着到第1步的进程中。

3. 在`*.py`中继续执行，会在vs中暂停。


### whl安装包的调式
参考[链接](https://blog.csdn.net/Attitude93/article/details/135656910) ，转至目录`carla\PythonAPI\carla`，运行`pip install -e .`，在`Python37\Lib\site-packages\easy-install.pth`文件中生成指向源代码的路径。

`-e` 或 `--editable`：这个选项表示以“可编辑”模式安装包。在这种模式下，包的源代码可以在本地进行修改，并且这些修改会立即反映到系统中该包的行为上，无需重新安装。这对于开发和调试非常有用。
在当前目录下安装一个包，并创建一个软连接引用该包（而不是将包复制到 site-packages
目录下）。这个软连接是一个指向包代码的符号链接，它可以使包的修改直接反映到当前目录下的项目中，从而方便开发和调试。

## 解释

### [setup.py](https://www.cnblogs.com/xueweihan/p/12030457.html) 
| 参数          | 说明                                                                                                                                                                                              |
|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ext_modules` | 用于构建 C++ 扩展扩展包，其是 Extension 实例的列表，每一个 Extension 实例描述了一个独立的扩展模块，扩展模块可以设置扩展包名、源文件`source`、头文件`include_dirs`、链接库及其路径`library_dirs`、编译参数`extra_compile_args`、链接参数`extra_compile_args`、依赖`depends`等。 |
| `package_dir={'': 'source'}` | “root package” 中的模块都在 source 目录中                                                                                                                                                                |
| `packages=['carla']` | 需要处理的包目录（包含了一些.cpp文件）                                                                                                                                                                           |

### GLI
[全局解释器锁](https://www.jb51.net/article/148353.htm) （GLI,Global Interpreter Lock）：在调用外部代码（如C、C++扩展函数）的时候，GIL将会被锁定，直到这个函数结束为止（由于期间没有python的字节码运行，所以不会做线程切换）。
