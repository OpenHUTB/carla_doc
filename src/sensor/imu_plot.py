#!/usr/bin/env python

# Copyright (c) 2019 Marc G Puig. All rights reserved.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import matplotlib.pyplot as plt
from matplotlib import animation

import argparse
import math
import time
import weakref


def length(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)

def normalize_vector(vec):
    l = length(vec)
    if l:
        k = 1.0 / length(vec)
        vec.x *= k
        vec.y *= k
        vec.z *= k
        return vec
    return carla.Vector3D()

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    # Time in seconds
    time_plot = []
    # [[x], [y], [z]] values rad/s
    gyros_plot = [[], [], []]
    # [[x], [y], [z]] values m/s
    acc_plot = [[], [], []]

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()
        debug = world.debug

        bp_lb = world.get_blueprint_library()
        imu_bp = bp_lb.filter('sensor.other.imu')[0]
        gnss_bp = bp_lb.filter('sensor.other.gnss')[0]
        mustang_bp = bp_lb.filter('vehicle.ford.mustang')[0]

        imu_bp.set_attribute('noise_seed', '10')

        imu_bp.set_attribute('noise_accel_stddev_x', '0.1')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.1')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.1')

        #imu_bp.set_attribute('noise_lat_stddev', '0.1')

        imu_bp.set_attribute('noise_gyro_stddev_x', '0.1')
        imu_bp.set_attribute('noise_gyro_stddev_z', '0.1')
        imu_bp.set_attribute('noise_gyro_stddev_y', '0.1')

        start_location = carla.Transform(carla.Location(-47.7, -83.9, 5.5))
        # start_location = world.get_map().get_spawn_points()[0].location

        print(world.get_map())
        # print(world.get_map().get_spawn_points())

        mustang = world.spawn_actor(
            mustang_bp,
            start_location)

        imu = world.spawn_actor(
            imu_bp,
            carla.Transform(
                location=carla.Location(y=1.5, z=1.5),
                rotation=carla.Rotation()),
                # rotation=carla.Rotation(pitch=45.0)),
                # rotation=carla.Rotation(pitch=90.0)),
            mustang)

        gnss = world.spawn_actor(
            gnss_bp,
            carla.Transform(),
            mustang)

        mustang.apply_control(carla.VehicleControl(throttle=0.3, steer=-0.0))

        vec_size = 10.0

        init_time = world.get_snapshot().timestamp.elapsed_seconds

        def imu_callback(weak_imu, sensor):
            self = weak_imu()
            if not self:
                return
            # print(sensor)
            imu_tr = imu.get_transform()
            imu_pos = imu_tr.location
            fw_vec = imu_tr.get_forward_vector()

            normalized_gy = normalize_vector(sensor.gyroscope)

            # debug.draw_line(
            #     imu_pos,
            #     imu_pos + (normalized_gy * (vec_size * length(sensor.gyroscope))),
            #     thickness=0.05,
            #     color=carla.Color(255, 0, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # Plotting #######################################
            curr_time = sensor.timestamp - init_time

            sys.stdout.write(f"\rElapsed time: {curr_time:4.1f}/{time_to_run} s")
            sys.stdout.flush()

            time_plot.append(curr_time)

            # gyros_plot[0].append(sensor.gyroscope.x)
            # total = 0.0
            # for i in gyros_plot[0]:
            #     total += i
            # total /= len(gyros_plot[0])
            # gyros_plot[1].append(total)
            gyros_plot[0].append(sensor.gyroscope.x)
            gyros_plot[1].append(sensor.gyroscope.y)
            gyros_plot[2].append(sensor.gyroscope.z)

            acc_plot[0].append(sensor.accelerometer.x)
            acc_plot[1].append(sensor.accelerometer.y)
            acc_plot[2].append(sensor.accelerometer.z)


            # debug.draw_line(
            #     imu_pos,
            #     imu_pos + (fw_vec * vec_size),
            #     thickness=0.05,
            #     color=carla.Color(255, 0, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # angular velocity #######################################
            # nnormal = sensor.gyroscope / (2.0 * math.pi)
            # rotated_fw_vec_x = carla.Location(x=vec_size * nnormal.x)
            # imu_tr.transform(rotated_fw_vec_x)
            # debug.draw_line(
            #     imu_pos,
            #     rotated_fw_vec_x,
            #     thickness=0.05,
            #     color=carla.Color(255, 0, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # rotated_fw_vec_y = carla.Location(y=vec_size * nnormal.y)
            # imu_tr.transform(rotated_fw_vec_y)
            # debug.draw_line(
            #     imu_pos,
            #     rotated_fw_vec_y,
            #     thickness=0.05,
            #     color=carla.Color(0, 255, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # rotated_fw_vec_z = carla.Location(z=vec_size * nnormal.z)
            # imu_tr.transform(rotated_fw_vec_z)
            # debug.draw_line(
            #     imu_pos,
            #     rotated_fw_vec_z,
            #     thickness=0.05,
            #     color=carla.Color(0, 0, 255),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # compass #######################################
            # north_vec = imu_tr.get_forward_vector()
            # carla.Transform(rotation=carla.Rotation(
            #     yaw=math.degrees(sensor.compass))).transform(north_vec)
            # north_vec.z = 0.0
            # north_vec = normalize_vector(north_vec)

            # debug.draw_line(
            #     imu_pos,
            #     imu_pos + (north_vec * vec_size),
            #     thickness=0.05,
            #     color=carla.Color(255, 165, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

            # accelerometer #######################################
            # imu_tr = imu.get_transform()
            # sens_tr = sensor.transform

            # carla.Transform(rotation=carla.Rotation(
            #     yaw=math.degrees(sensor.compass))).transform(north_vec)
            # north_vec = normalize_vector(north_vec)

            # debug.draw_line(
            #     imu_pos,
            #     imu_pos + (north_vec * vec_size),
            #     thickness=0.05,
            #     color=carla.Color(255, 165, 0),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)

        weak_imu = weakref.ref(imu)
        imu.listen(lambda sensor: imu_callback(weak_imu, sensor))

        time_to_run = 20.0 # in seconds

        close_time = time.time() + time_to_run

        rot = 0.0

        while time.time() < close_time:
        # while time.time() > 0:
            snapshot = world.wait_for_tick(2.0)
            # imu.set_transform(carla.Transform(
            #     carla.Location(),
            #     carla.Rotation(yaw=rot)))
            # rot += 5
            # imu_pos = mustang.get_location()
            # debug.draw_line(
            #     imu_pos,
            #     imu_pos + carla.Location(z=10),
            #     life_time=snapshot.timestamp.delta_seconds,
            #     persistent_lines=False)
            # print(f'frame: {snapshot.frame}')
            # print(f'timestamp: {snapshot.timestamp}')

    finally:
        print('')
        #imu.destroy()
        #mustang.destroy()

        # plt.xkcd() # lol

        ############################################

        # plt.title("Simple Plot")
        # plt.figure()

        # plt.subplot(211, title='gyroscope') # ------------------------------------------

        # plt.tick_params(labelbottom=False)
        # plt.ylabel('angular velocity (rad)')

        # plt.grid(True)

        # plt.plot(time_plot[2:], gyros_plot[0][2:], color='red', label='X')
        # plt.plot(time_plot[2:], gyros_plot[1][2:], color='green', label='Y')
        # plt.plot(time_plot[2:], gyros_plot[2][2:], color='blue', label='Z')

        # plt.legend()

        # plt.subplot(212, title='accelerometer') # ------------------------------------------

        # plt.ylabel('accelerometer (m/s)')
        # plt.xlabel('time (s)')

        # plt.grid(True)

        # plt.plot(time_plot[2:], acc_plot[0][2:], color='red', label='X')
        # plt.plot(time_plot[2:], acc_plot[1][2:], color='green', label='Y')
        # plt.plot(time_plot[2:], acc_plot[2][2:], color='blue', label='Z')

        # plt.legend()

        # plt.show()

        ############################################

        # plt.subplot(131)
        # plt.hist(acc_plot[0], bins=50)
        # plt.title('X')

        # plt.subplot(132)
        # plt.hist(acc_plot[1], bins=50)
        # plt.title('Y')

        # plt.subplot(133)
        # plt.hist(acc_plot[2], bins=50)
        # plt.title('Z')

        # plt.show()

        ############################################

        # Gyroscope Figure
        fig_g, ax_g = plt.subplots(figsize=(10, 4))
        ax_g.set(xlim=(0, 20), ylim=(-5, 5))
        ax_g.set_xlabel('Time (s)')
        ax_g.set_ylabel('Gyroscope (rad/s)')
        ax_g.grid(True)

        plot_gyros_x, = plt.plot([], [], color='red', label='X')
        plot_gyros_y, = plt.plot([], [], color='green', label='Y')
        plot_gyros_z, = plt.plot([], [], color='blue', label='Z')

        ax_g.legend()

        def gyros_init():
            plot_gyros_x.set_data(time_plot[2], gyros_plot[0][2])
            plot_gyros_y.set_data(time_plot[2], gyros_plot[1][2])
            plot_gyros_z.set_data(time_plot[2], gyros_plot[2][2])
            return plot_gyros_x, plot_gyros_y, plot_gyros_z,

        def gyros_animate(i):
            ax_g.set_title(f"Frame {i}")
            ax_g.set(xlim=(time_plot[i]-5.0, time_plot[i]))

            plot_gyros_x.set_data(time_plot[:i], gyros_plot[0][:i])
            plot_gyros_y.set_data(time_plot[:i], gyros_plot[1][:i])
            plot_gyros_z.set_data(time_plot[:i], gyros_plot[2][:i])
            return plot_gyros_x, plot_gyros_y, plot_gyros_z,

        gyros_anim = animation.FuncAnimation(
            fig_g,
            gyros_animate,
            init_func=gyros_init,
            frames=len(time_plot),
            interval=16.6,
            blit=True,
            repeat=True)

        # Acceleration Figure
        fig_a, ax_a = plt.subplots(figsize=(10, 4))
        ax_a.set(xlim=(0, 20), ylim=(-15, 20))
        ax_a.set_xlabel('Time (s)')
        ax_a.set_ylabel('Accelerometer (m/s^2)')
        ax_a.grid(True)

        plot_acc_x, = ax_a.plot([], [], color='red', label='X')
        plot_acc_y, = ax_a.plot([], [], color='green', label='Y')
        plot_acc_z, = ax_a.plot([], [], color='blue', label='Z')

        ax_a.legend()

        def acc_init():
            plot_acc_x.set_data(time_plot[2], acc_plot[0][2])
            plot_acc_y.set_data(time_plot[2], acc_plot[1][2])
            plot_acc_z.set_data(time_plot[2], acc_plot[2][2])
            return plot_acc_x, plot_acc_y, plot_acc_z,

        def acc_animate(i):
            ax_a.set_title(f"Frame {i}")
            ax_a.set(xlim=(time_plot[i]-5.0, time_plot[i]))

            plot_acc_x.set_data(time_plot[:i], acc_plot[0][:i])
            plot_acc_y.set_data(time_plot[:i], acc_plot[1][:i])
            plot_acc_z.set_data(time_plot[:i], acc_plot[2][:i])
            return plot_acc_x, plot_acc_y, plot_acc_z,

        acc_anim = animation.FuncAnimation(
            fig_a,
            acc_animate,
            init_func=acc_init,
            frames=len(time_plot),
            interval=16.6,
            blit=True,
            repeat=True)

        # gyros_anim.save('../../../Videos/gyros_plot.mp4', fps=60, extra_args=['-vcodec', 'libx264'])
        # acc_anim.save('../../../Videos/acc_plot.mp4', fps=60, extra_args=['-vcodec', 'libx264'])

        plt.show()


if __name__ == '__main__':

    main()
