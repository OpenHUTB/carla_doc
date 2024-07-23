import sys
import carla
sys.path.append('../logidrivepy')
from logidrivepy import LogitechController
import time


class SimpleController:
    def __init__(self, k):
        # k 是比例增益，用于调整控制器的响应速度
        self.k = k

    def control(self, aim, now):
        # 计算位置差值
        error = now - aim

        # 计算输出力
        force = self.k * error

        return force

class ForceFeedback:
    def __init__(self):
        self.position = None
        self.torque = None

class Node():
    def __init__(self, client):
        self.client = client
        self.world = client.get_world()
        self.actor = self.get_hero()

    def get_hero(self):
        for actor in self.world.get_actors():
            if actor.attributes.get("role_name") in ["hero", "ego_vehicle"]:
                return actor

    def timer_cb(self):
        out_msg = ForceFeedback()

        steering_angle = self.actor.get_control().steer
        out_msg.position = steering_angle
        out_msg.torque = 0.8

    def get_position(self):
        steering_angle = self.actor.get_control().steer
        position = steering_angle
        return position


def main(args=None):
    controller = LogitechController()
    controller.steering_initialize()

    print("\n---Logitech Spin Test---")

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)

    carla_car = Node(client)
    # 创建一个PID控制器
    sc = SimpleController(k=1)

    # 在每个时间步中，更新控制器并使用新的力
    while True:
        aim = carla_car.get_position() * 150
        now = (controller.LogiGetStateENGINES(0).contents.lX) / 327.50
        if (abs(aim)<1):
            controller.LogiPlaySoftstopForce(0,0)
            controller.logi_update()
            time.sleep(0.1)
            controller.LogiStopSoftstopForce(0)
            controller.logi_update()
            print("Target: {}, Current State: {}".format(aim, now))
        else:
            force = sc.control(now, aim)
            controller.LogiPlayConstantForce(0, -int(force)*2)
            controller.logi_update()
            print("Force: {}, Target: {}, Current State: {}".format(force, aim, now))
            time.sleep(0.1)
    controller.steering_shutdown()

if __name__ == "__main__":
    main()