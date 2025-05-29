# 持续集成


## [Jenkins安装](https://sdpro.top/blog/html/article/1051.html)

1.安装 [JDK21](https://java.com/java/technologies/downloads/#jdk21-windows)

2.[安装Jenkins](https://blog.csdn.net/qq_36746815/article/details/127393076)

作为本地服务；端口 8080


3.打开 [http://172.21.108.56:8080](http://172.21.108.56:8080)

b00acffa035946e18bb20924c6f77181

```text
用户名：cc
密码：a5300066
```

重启服务：
在地址栏中输入：`http://172.21.108.56:8080/restart/`


配置所使用的节点：`http://172.21.108.56:8080/manage/computer/`

需要配置jenkins代理，否则会出现各种连接不到github的错误。
Manager Jenkins -> System Configuration -> System -> 
Http Proxy Configuration -> 
```shell
# 服务器
127.0.0.1
# 端口
7890
```

缺少资产下载操作


### 安装github插件

在 Dashboard 中选择 “管理 Jenkins” -> ``


## Jenkins配置

这个是因为我使用的本地的git用来测试，所以需要配置一下，在 jenkins安装目录（默认是C:\Program Files\Jenkins）找到 jenkins.xml

1.在配置页面的`流水线`下的`定义`下拉菜单中选择`Pipeline script from SCM` (Software Configuration Management) 。
仓库配置为`C:\ProgramData\Jenkins\.jenkins\workspace`，并制定分支。


2.开始构建

默认克隆在`C:\ProgramData\Jenkins\.jenkins\workspace`目录下。




## 参考

* [社区的持续集成地址](http://158.109.8.172:8080)


