# [DeepSeek R1 本地部署（Ollama+OpenWebUI）](https://blog.csdn.net/youcans/article/details/145414871)

!!! 注意
     校园内网也可以直接访问链接 [链接](http://10.0.254.164:8080/auth) 进行使用（用户名：2929@hutb.edu.cn，密码：hutb）。

## 1.DeepSeek R1 安装
下载并安装 [Ollama](https://ollama.com/download) 后，打开链接 [http://localhost:11434/](http://localhost:11434/) ，验证安装Ollama成功。

运行以下命令安装7b模型（目前支持1.5b、7b、14b、32b、70b、671b）：
```shell
ollama run deepseek-r1:7b
```
下载完成即可通过命令行进行对话。

## 2.安装 WebUI
2.1 以管理员身份运行 PowerShell，执行以下命令启动Hyper-V：
```shell
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V -All
```

2.2 安装 [Docker for Desktop](https://www.docker.com/products/docker-desktop/)


2.3 安装和启动 open-webui
```shell
docker run -d -p 3000:8080 --add-host=host.docker.internal:host-gateway -v open-webui:/app/backend/data --name open-webui --restart always ghcr.io/open-webui/open-webui:main
```

2.4 打开 [http://127.0.0.1:3000](http://127.0.0.1:3000) 开始网页使用。

## 参考

* [全流程安装DeepSeek开源模型](https://blog.csdn.net/AnNanDu/article/details/145462409)
* [Docker 安装](https://www.runoob.com/docker/windows-docker-install.html)
* [文档和代码](https://github.com/OpenHUTB/carla_doc)