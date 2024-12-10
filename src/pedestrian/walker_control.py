# 在 0.9.13、0.9.15 中进行了测试
# 参考： https://github.com/PJLab-ADG/PCSim/blob/0a0ea957f489ef14c4db338f0c035bd41f0544f1/ReSimAD/src/walker_control.py
import random
import time

import carla


class BaseWalkerPose:
    def control(self, walker, left_roll=None, right_roll=None):
        bones = walker.get_bones().bone_transforms  # 获得行人的骨架（大小为66BoneTransformDataOut的数组）
        # relative：基于骨骼父级进行变换
        TR = bones[56].relative  # crl_thigh__R 右大腿。
        TL = bones[61].relative  # crl_thigh__L 左大腿
        AR = bones[5].relative  # crl_arm__L 左臂
        AL = bones[33].relative  # crl_arm__R 右臂
        if left_roll is None or right_roll is None:
            total = random.randint(0, 40)
            left_roll = random.randint(0, total)
            right_roll = total + total - left_roll

        # 每个都是(骨骼名称，基于骨骼父级进行变换)
        thigh_r = ('crl_thigh__R', carla.Transform(location=TR.location,
                                                   rotation=carla.Rotation(roll=TR.rotation.roll + right_roll,
                                                                           yaw=TR.rotation.yaw,
                                                                           pitch=TR.rotation.pitch)))
        thigh_l = ('crl_thigh__L', carla.Transform(location=TL.location,
                                                   rotation=carla.Rotation(roll=TL.rotation.roll - left_roll,
                                                                           yaw=TL.rotation.yaw,
                                                                           pitch=TL.rotation.pitch)))
        arm_r = ('crl_arm__R', carla.Transform(location=AR.location,
                                               rotation=carla.Rotation(
                                                   roll=AR.rotation.roll,
                                                   yaw=AR.rotation.yaw,
                                                   pitch=AR.rotation.pitch + left_roll * 0.8)))
        arm_l = ('crl_arm__L', carla.Transform(location=AL.location,
                                               rotation=carla.Rotation(
                                                   roll=AL.rotation.roll,
                                                   yaw=AL.rotation.yaw,
                                                   pitch=AL.rotation.pitch + right_roll * 0.8)))
        control = carla.WalkerBoneControlIn([thigh_r, thigh_l, arm_l, arm_r])  # 为行人提供骨骼的特定操作
        control.bone_transforms = [thigh_r, thigh_l, arm_l, arm_r]
        walker.blend_pose(1)  # 设置自定义姿势与动画的混合值。1: 仅显示自定义姿势（由用户使用 set_bones() 设置）
        walker.set_bones(control)  # 设置参与者的骨骼。对于我们想要设置的每个骨骼，我们使用相对变换。
        walker.show_pose()

    def demo(self):
        client = carla.Client('127.0.0.1', 2000)
        world = client.get_world()
        blueprint_list = world.get_blueprint_library().filter('walker.pedestrian.*')
        blueprint = blueprint_list[1]  # 固定行人，用于测试
        # 不是参数原因，而是在同一个位置生成行人，可能会生成行人失败
        spawn_point = carla.Transform(carla.Location(0, 5, 1), carla.Rotation(0, 0, 0))
        try:
            walker = world.try_spawn_actor(blueprint, spawn_point)
            # 禁用此参与者上的物理模拟
            if walker is not None:
                walker.set_simulate_physics(False)

                # 将相机转向行人的生成点
                trans = walker.get_transform()
                trans.location.x = -2
                trans.location.y = 2
                trans.location.z = 2
                trans.rotation.pitch = -10
                trans.rotation.yaw = 60
                spectator = world.get_spectator()

                spectator.set_transform(trans)  # 将参与者传送到给定的变换（位置和旋转）。

                time.sleep(3)  # 不连续
                self.control(walker, 35, 35)

                # 暂停3秒后销毁（用户调试）
                time.sleep(10)
            else:
                print("Spawn pedestrian failed.")
        finally:
            # 停止并销毁所有controller
            for controller in world.get_actors().filter('*controller*'):
                controller.stop()
            # 销毁所有车辆
            for vehicle in world.get_actors().filter('*vehicle*'):
                vehicle.destroy()
            # 销毁所有行人
            for walker in world.get_actors().filter('*walker*'):
                walker.destroy()


if __name__ == '__main__':
    wp = BaseWalkerPose()
    wp.demo()
