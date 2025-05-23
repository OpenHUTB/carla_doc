import carla
from PIL import Image

# 客户端连接
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

world = client.get_world()

# 加载已修改的纹理
image = Image.open('cloth.tga')  # 任意一个.tga文件即可，不需从虚幻编辑器中导出。
height = image.size[1]
width = image.size[0]

# 实例化 carla.TextureColor 对象并使用修改后的图像中的数据填充像素
texture = carla.TextureColor(width ,height)
for x in range(0,width):
    for y in range(0,height):
        color = image.getpixel((x,y))
        r = int(color[0])
        g = int(color[1])
        b = int(color[2])
        a = 255
        texture.set(x, y, carla.Color(r,g,b,a))

# 现在将纹理应用到建筑资产
world.apply_color_texture_to_object('BP_Apartment04_v05_Opt_2', carla.MaterialParameter.Diffuse, texture)
