# 参考：https://www.cnblogs.com/qkshhan/p/16202931.html
# 无限循环播放：https://blog.csdn.net/I_fole_you/article/details/131050549
from PIL import ImageSequence, Image
import imageio, os

location = "./"  # 文件目录
gif_file = os.path.join(location, "origin.gif")  # 输入GIF文件名
out_file = os.path.join(location, "output.gif")  # 输出GIF文件名

# 这里如果想使用灰度图，可以使用`.convert("L")`代替，参考https://pillow-cn.readthedocs.io/zh_CN/latest/handbook/concepts.html#mode
frames = [frame.copy().convert("RGBA") for frame in ImageSequence.Iterator(Image.open(gif_file))]
imageio.mimsave(out_file, frames, 'GIF', duration=0.5, loop=0)  # duration表示两帧之间播放的间隔时间，也可不设，使用默认（0.5表示加快）；loop=0表示无限循环播放
# frames[0].save("out.gif", save_all=True, append_images=frames[1:])
