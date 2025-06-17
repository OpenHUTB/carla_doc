# 参考：https://www.cnblogs.com/qkshhan/p/16202931.html
# 无限循环播放：https://blog.csdn.net/I_fole_you/article/details/131050549
# 使用everything搜索大于28MB的文件（cloudflare不能部署大于30M的文件）：size:>28mb D:\work\workspace\doc\docs\img
from PIL import ImageSequence, Image
import imageio, os

location = "./"  # 文件目录
gif_file = os.path.join(location, "origin.gif")  # 输入GIF文件名
out_file = os.path.join(location, "output.gif")  # 输出GIF文件名

# 未压缩版本
# 这里如果想使用灰度图，可以使用`.convert("L")`代替，参考https://pillow-cn.readthedocs.io/zh_CN/latest/handbook/concepts.html#mode
# frames = [frame.copy().convert("RGBA") for frame in ImageSequence.Iterator(Image.open(gif_file))]
# imageio.mimsave(out_file, frames, 'GIF', duration=0.5, loop=0)  # duration表示两帧之间播放的间隔时间，也可不设，使用默认（0.5表示加快）；loop=0表示无限循环播放
# frames[0].save("out.gif", save_all=True, append_images=frames[1:])


# 压缩版本，参考： https://developer.aliyun.com/article/1172904
# 设置压缩尺寸，值越大动图越清晰，文件也越大
rp = 800

img_list = []

# 读取原gif动图
img = Image.open(gif_file)

# 对原动图进行压缩，并存入img_list
for i in ImageSequence.Iterator(img):
    i = i.convert('RGB')
    if max(i.size[0], i.size[1]) > rp:
        i.thumbnail((rp, rp))
    img_list.append(i)

# 计算帧的频率
# durt = (img.info)['duration'] / 0.5

# 读取img_list合成新的gif
imageio.mimsave(out_file, img_list, loop=0)  # duration=0.8
