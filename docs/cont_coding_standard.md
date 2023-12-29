# 编码标准

*   [__一般__](#general)  
*   [__Python__](#python)  
*   [__C++__](#c++)  

---
## 一般

  * 使用空格，而不是制表符。
  * 避免添加尾随空格，因为它会在差异中产生噪音。

---
## Python

  * 注释不应超过 80 列，代码不应超过 120 列。
  * 所有代码必须与 Python 2.7 和 3.7 兼容。
  * [Pylint][pylintlink] 不应给出任何错误或警告（很少有例外适用于外部类，如 `numpy` 和 `pygame`，请参阅我们的 `.pylintrc`）。
  * Python 代码遵循 [PEP8 风格指南][pep8link]（`autopep8`尽可能使用）。

[pylintlink]: https://www.pylint.org/
[pep8link]: https://www.python.org/dev/peps/pep-0008/

---
## C++

  * 注释不应超过 80 列，如果可以使代码更清晰，在极少数情况下代码可能会稍微超出此限制。
  * 编译不应给出任何错误或警告
    (`clang++-8 -Wall -Wextra -std=C++14 -Wno-missing-braces`).
  * 禁止使用 `throw`，请改为使用 `carla::throw_exception`。
  * 虚幻 C++ 代码（CarlaUE4 和 Carla 插件）遵循 [虚幻引擎的编码标准][ue4link] ，但使用空格而不是制表符。
  * LibCarla 使用 [谷歌风格指南][googlelink] 的变体。
  * 如果代码在服务器端使用，则应将 `try-catch` 块的使用 `#ifndef LIBCARLA_NO_EXCEPTIONS` 括起来 。

[ue4link]: https://docs.unrealengine.com/latest/INT/Programming/Development/CodingStandard/
[googlelink]: https://google.github.io/styleguide/cppguide.html
