import platform
import subprocess
import threading
import time

from generate_traffic import *
from manual_control import *


def launch_carla():
    launch_info = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'WindowsNoEditor_9.15', 'CarlaUE4.exe')
    # 等待调用的程序执行结束后再运行
    os.popen(launch_info)
    print("launch carla")


# class Manual_Control:
#     def manual_control_method(self):
#         while(True):
#             print("Manual control")
#             manual_control()


if __name__ == '__main__':

    launch_carla()
    time.sleep(20)

    # 使用独立的现成启动交通管理器
    if platform.system().lower() == 'windows':
        gen_traffic_cmd = 'start ' + sys.executable + ' ' + os.path.join(os.path.split(os.path.realpath(__file__))[0], 'generate_traffic.py')
    elif platform.system().lower() == 'linux':
        gen_traffic_cmd = sys.executable + ' ' + os.path.join(os.path.split(os.path.realpath(__file__))[0], 'generate_traffic.py') + ' &'

    # creationflags 设置为 0x08000000，是隐藏cmd窗口的标识（无效）
    subprocess.Popen(gen_traffic_cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1, creationflags=0x08000000)
    # os.system(gen_traffic_cmd)

    manual_control()

