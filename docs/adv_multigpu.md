# Carla 多 GPU 功能

Carla 中的多 GPU 意味着用户可以启动多个服务器（称为辅助服务器），这些服务器将使用系统中的专用 GPU 为主服务器（称为主服务器）执行渲染工作。主服务器将用户创建的传感器分发到不同的可用辅助服务器。

## 主服务器

步骤是: 首先，启动没有任何渲染功能的主服务器。我们可以使用的参数有：

* `-nullrhi`
* `-carla-primary-port`

从命令行运行：

```sh
./CarlaUE4.sh -nullrhi
```

The primary server will use, by default, the port 2002 to listen for secondary servers. If you need to listen on another port, then you can change it with the port flag:

```sh
./CarlaUE4.sh -nullrhi -carla-primary-port=3002
```

## Secondary servers

We may then start as many servers as we need, but the ideal is to have as many secondary servers as the number of GPUs installed in the computer. Through parameters we need to specify the GPU we want the server use and also the host/port where the primary server is listening, with the flags:

  * `-carla-rpc-port`
  * `-carla-primary-host`
  * `-carla-primary-port`
  * `-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter`

For example, if the primary server is executing in the same computer than the secondary servers and with the default port, we can use this command:

```sh
./CarlaUE4.sh -carla-rpc-port=3000 -carla-primary-host=127.0.0.1 -ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=0
```

Here, the secondary server will use port 3000 as the RPC server to avoid conflicts with other ports (but it will never be used), and will connect to the primary server located at IP 127.0.0.1 (localhost) in the default port (2002), and also this server will use the GPU device 0.

If we want to start another secondary server using another GPU, we could use the following command:

```sh
./CarlaUE4.sh -carla-rpc-port=4000 -carla-primary-host=127.0.0.1 -carla-primary-port=2002 -ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=1
```

This secondary server will use port 4000 as the RPC server to avoid conflicts with other ports and will connect to the primary server located at IP 127.0.0.1 in port 2002, and also this server will use the GPU device 1.

## Pipeline

After the first secondary server connects to the primary server, it will set up the synchronous mode automatically, with the default values of 1/20 delta seconds.

