import subprocess
import threading
import time

from generate_traffic import *
from manual_control import *


def launch_carla():
    # , '-system=PRD', '-client=110', '-l=ZH',
    #                   '-user=用户名', '-pw=密码'
    launch_info = 'D:\\project\\carla\\WindowsNoEditor_9.15\\CarlaUE4.exe'
    # 等待调用的程序执行结束后再运行
    # subprocess.check_call(launch_info)
    # p = subprocess.Popen(launch_info)
    os.popen(launch_info)
    # return_code = p.wait()  # 等待子进程结束，并返回状态码；
    # print(return_code)
    print("launch carla")



# class Generate_Traffic:
#
#     def generate_traffic(self):
#         while(True):
#             print("Generating traffic")
#             generate_traffic()


class Manual_Control:
    def manual_control_method(self):
        while(True):
            print("Manual control")
            manual_control()


if __name__ == '__main__':

    launch_carla()
    time.sleep(20)

    # m_c = Manual_Control()
    # thread_manual_control = threading.Thread(target=m_c.manual_control_method())
    # thread_manual_control.start()

    # 使用独立的现成启动交通管理器
    gen_traffic_cmd = 'start ' + sys.executable + ' ' + os.path.split(os.path.realpath(__file__))[0] + '\\generate_traffic.py'
    os.system(gen_traffic_cmd)
    # gen_traffic = Generate_Traffic()
    # thread_gen_traffic = threading.Thread(target=gen_traffic.generate_traffic())
    # thread_gen_traffic.setDaemon(True)  # 将线程设置为分离状态
    # thread_gen_traffic.start()
    # print("Generated Traffic")

    # m_c = Manual_Control()
    # thread_manual_control = threading.Thread(target=m_c.manual_control_method())
    # thread_manual_control.start()

    manual_control()

