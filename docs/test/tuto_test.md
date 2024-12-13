## 测试框架

Carla的测试框架目前只支持Ubuntu平台，执行命令`make smoke_tests`进行测试。


### Windows平台
脚本`src\test>check.bat`用于启动windows平台下的测试，运行的第一个测试用例为：
```shell
python -m nose2 -v smoke.test_client
```
整个脚本会依次运行 [smoke_test_list.txt](https://github.com/OpenHUTB/carla_doc/blob/master/src/test/smoke_test_list.txt) 文件中的所有测试用例。


