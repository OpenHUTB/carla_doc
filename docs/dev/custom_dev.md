
# 自定义开发

## 同步 [ue4_dev 分支](https://github.com/carla-simulator/carla/commits/ue4-dev/)

1.在需要合并 [提交](https://github.com/carla-simulator/carla/commit/cf17eb576e866404eabac0c93af3558da0d91be3) 的地址栏中页面地址后添加`.patch`，下载对应提交的补丁。

2.检查patch是否能正常打入:
```shell
git apply --check 【path/to/xxx.patch】
```

3.如果检查能够正常打入（第2步没有任何输出），则执行：
```shell
git apply 【path/to/xxx.patch】
```
如果有冲突则按行进行合并。


## 冲突解决

强制打补丁
```shell
git apply --reject xxx.patch
```
加入这个选项能够将能打上的补丁先打上，冲突的文件生成*.rej文件



## 参考

[git 生成补丁文件及打补丁](https://blog.csdn.net/xiewenhao12/article/details/117923288)



