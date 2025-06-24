# 持续集成


## [Jenkins安装](https://sdpro.top/blog/html/article/1051.html)

1.安装 [JDK21](https://java.com/java/technologies/downloads/#jdk21-windows)

2.[安装 Jenkins 2.492.3 ](https://blog.csdn.net/qq_36746815/article/details/127393076)

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
Http Proxy Configuration：
```shell
# 服务器
127.0.0.1
# 端口
7890
```

安装 [亚马逊云的命令行工具](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html) ，
[创建访问密钥](https://us-east-1.console.aws.amazon.com/iam/home?region=ap-southeast-2#/security_credentials) ：
```shell
# 配置ID和访问密钥
aws configure
# 查看配置（可选）
aws configure list
# 下载文件
aws s3 cp s3://hutb/hutb.png .
# 上传文件
aws s3 cp Jenkinsfile s3://hutb/
```

（可能非必需）安装 AWS Credentials 插件，然后在设置中添加AWS Credentials。

在脚本中直接指定key_id和secret_access_key：
```shell
aws configure set aws_access_key_id XXX
aws configure set aws_secret_access_key XXX
```


## Jenkins配置

这个是因为我使用的本地的git用来测试，所以需要配置一下，在 jenkins安装目录（默认是C:\Program Files\Jenkins）找到 jenkins.xml

1.在配置页面的`流水线`下的`定义`下拉菜单中选择`Pipeline script from SCM` (Software Configuration Management) 。
仓库配置为`C:\ProgramData\Jenkins\.jenkins\workspace`，并制定分支。


2.开始构建

默认克隆在`C:\ProgramData\Jenkins\.jenkins\workspace`目录下。

## 高级配置

##### [配置构建失败时发送邮件](https://juejin.cn/post/6844904119707123719)

下载两个插件：Email Extension, Email Extension Template， 这两个插件可以帮助我们进行邮件的编写以及格式化。

在Jenkins的“设置->Account”中设置邮件地址，如“123456@qq.com”

在Jenkins的“Manage Jenkins -> System” 中的 `Extended E-mail Notification`：

问题: `org.eclipse.angus.mail.smtp.SMTPSenderFailedException: 501 Mail from address must be same as authorization user.`

> 需要将`Manage Jenkins —> System` 中的 `系统管理员邮件地址` 要和`邮件通知`中的地址一致（`123456@qq.com`）。

问题：`HTTP ERROR 403 No valid crumb was included in the request`

> 将`Manage Jenkins -> Security`中的`跨站请求伪造保护`中的`启用代理兼容`勾选。

其中的`Test e-mail recipient`为接收方邮箱地址（QQ邮箱为发送方）。


## 参考

* [社区的持续集成地址](http://158.109.8.172:8080)
* [Jenkins之Email配置与任务邮件发送实践与踩坑](https://juejin.cn/post/6844904119707123719)
* [QQ邮箱：什么是授权码，它又是如何设置？]()


