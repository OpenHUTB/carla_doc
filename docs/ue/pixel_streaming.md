## 像素流推送


!!! 注意
    当启动信令服务后访问不了像素流，尝试关闭系统代理然后再访问。


### 优化
量化参数（Quantization Parameter, QP）值用于控制每帧视频中每一个宏块的压缩量，和比特率成反比，QP值越小画质越高。

### 参考

- [交通大模型实现的像素流推送](https://github.com/OpenHUTB/gpt/tree/main/sim/WindowsNoEditor/Engine/Source/Programs/PixelStreaming/WebServers/SignallingWebServer)
- [像素流送入门](https://dev.epicgames.com/documentation/zh-cn/unreal-engine/getting-started-with-pixel-streaming?application_version=4.27)
- [流送优化指南](https://dev.epicgames.com/documentation/zh-cn/unreal-engine/stream-tuning-guide)
