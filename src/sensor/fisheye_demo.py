#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Sensor synchronization example for CARLA
在当前脚本的目录的fisheye_data文件夹下生成鱼眼相机的图片

The communication model for the syncronous mode in CARLA sends the snapshot
of the world and the sensors streams in parallel.
We provide this script as an example of how to syncrononize the sensor
data gathering in the client.
To to this, we create a queue that is being filled by every sensor when the
client receives its data and the main loop is blocked until all the sensors
have received its data.
This suppose that all the sensors gather information at every tick. It this is
not the case, the clients needs to take in account at each frame how many
sensors are going to tick at each frame.

modified by J.Z. 22.09.09

"""

import glob
import os
import sys
from queue import Queue
from queue import Empty

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import logging
import random

# Sensor callback.
# This is where you receive the sensor data and
# process it as you liked and the important part is that,
# at the end, it should include an element into the sensor queue.
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    if sensor_data.frame%3==0:
        if sensor_name=="cam1":
            sensor_data.save_to_disk('./fisheye_data/1_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam2":
            sensor_data.save_to_disk('./fisheye_data/2_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam3":
            sensor_data.save_to_disk('./fisheye_data/3_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam4":
            sensor_data.save_to_disk('./fisheye_data/4_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam5":
            sensor_data.save_to_disk('./fisheye_data/5_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam6":
            sensor_data.save_to_disk('./fisheye_data/6_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam7":
            sensor_data.save_to_disk('./fisheye_data/7_%.6f.jpg' % sensor_data.timestamp)
        if sensor_name=="cam8":
            sensor_data.save_to_disk('./fisheye_data/8_%.6f.jpg' % sensor_data.timestamp)
        sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    # We start creating the client
    host_ip = '127.0.0.1'
    host_port = 2000
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(host_ip, host_port)
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        ego_vehicle = None
        ego_cam1 = None
        ego_cam2 = None
        ego_cam3 = None
        ego_cam4 = None
        ego_cam5 = None
        ego_cam6 = None
        ego_cam7 = None
        ego_cam8 = None
        
        # We need to save the settings to be able to recover them at the end
        # of the script to leave the server in the same state that we found it.
        original_settings = world.get_settings()
        settings = world.get_settings()

        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.01
        settings.synchronous_mode = True
        world.apply_settings(settings)

        client.reload_world(False) # reload map keeping the world settings

        # --------------
        # Spawn ego vehicle
        # --------------
        
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            ego_vehicle.set_autopilot(True)  # 车辆进入自动驾驶模型进行数据收集
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not found any spawn points')

        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()

        # We create all the sensors and keep them in a list for convenience.
        sensor_list = []

        # --------------
        # Add a RGB camera sensor to ego vehicle. 
        # --------------
        
        cam1_bp = None
        cam1_bp = world.get_blueprint_library().find('sensor.camera.fisheye')
        cam1_bp.set_attribute('x_size', str(1280))
        cam1_bp.set_attribute('y_size', str(720))
        cam1_bp.set_attribute('max_angle', str(210))
        cam1_bp.set_attribute('d_1', str(0.08309221636708493))
        cam1_bp.set_attribute('d_2', str(0.01112126630599195))
        cam1_bp.set_attribute('d_3', str(-0.008587261043925865))
        cam1_bp.set_attribute('d_4', str(0.0008542188930970716))
        cam1_bp.set_attribute('f_x', str(320))
        cam1_bp.set_attribute('f_y', str(320))
        cam1_bp.set_attribute('c_x', str(640))
        cam1_bp.set_attribute('c_y', str(480))

        # cam1_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam1_bp.set_attribute("image_size_x",str(960))
        #cam1_bp.set_attribute("image_size_y",str(480))
        #cam1_bp.set_attribute("fov",str(105))
        # cam1_bp.set_attribute("sensor_tick", str(0.033))
        cam1_location = carla.Location(2,0,1)
        cam1_rotation = carla.Rotation(0,0,0)
        cam1_transform = carla.Transform(cam1_location,cam1_rotation)
        ego_cam1 = world.spawn_actor(cam1_bp,cam1_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        ego_cam1.listen(lambda image: sensor_callback(image, sensor_queue, "cam1"))
        sensor_list.append(ego_cam1)

        # cam2_bp = None
        # cam2_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam2_bp.set_attribute("image_size_x",str(3840))
        # cam2_bp.set_attribute("image_size_y",str(2160))
        # cam2_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam2_location = carla.Location(2,0,1)
        # cam2_rotation = carla.Rotation(0,10,0)
        # cam2_transform = carla.Transform(cam2_location,cam2_rotation)
        # ego_cam2 = world.spawn_actor(cam2_bp,cam2_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam2.listen(lambda image: sensor_callback(image, sensor_queue, "cam2"))
        # sensor_list.append(ego_cam2)

        # cam3_bp = None
        # cam3_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam3_bp.set_attribute("image_size_x",str(3840))
        # cam3_bp.set_attribute("image_size_y",str(2160))
        # cam3_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam3_location = carla.Location(2,0,1)
        # cam3_rotation = carla.Rotation(0,-10,0)
        # cam3_transform = carla.Transform(cam3_location,cam3_rotation)
        # ego_cam3 = world.spawn_actor(cam3_bp,cam3_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam3.listen(lambda image: sensor_callback(image, sensor_queue, "cam3"))
        # sensor_list.append(ego_cam3)

        # cam4_bp = None
        # cam4_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam4_bp.set_attribute("image_size_x",str(3840))
        # cam4_bp.set_attribute("image_size_y",str(2160))
        # cam4_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam4_location = carla.Location(2,0,1)
        # cam4_rotation = carla.Rotation(0,20,0)
        # cam4_transform = carla.Transform(cam4_location,cam4_rotation)
        # ego_cam4 = world.spawn_actor(cam4_bp,cam4_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam4.listen(lambda image: sensor_callback(image, sensor_queue, "cam4"))
        # sensor_list.append(ego_cam4)

        # cam5_bp = None
        # cam5_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam5_bp.set_attribute("image_size_x",str(3840))
        # cam5_bp.set_attribute("image_size_y",str(2160))
        # cam5_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam5_location = carla.Location(2,0,1)
        # cam5_rotation = carla.Rotation(0,-20,0)
        # cam5_transform = carla.Transform(cam5_location,cam5_rotation)
        # ego_cam5 = world.spawn_actor(cam5_bp,cam5_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam5.listen(lambda image: sensor_callback(image, sensor_queue, "cam5"))
        # sensor_list.append(ego_cam5)

        # cam6_bp = None
        # cam6_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam6_bp.set_attribute("image_size_x",str(3840))
        # cam6_bp.set_attribute("image_size_y",str(2160))
        # cam6_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam6_location = carla.Location(2,0,1)
        # cam6_rotation = carla.Rotation(0,30,0)
        # cam6_transform = carla.Transform(cam6_location,cam6_rotation)
        # ego_cam6 = world.spawn_actor(cam6_bp,cam6_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam6.listen(lambda image: sensor_callback(image, sensor_queue, "cam6"))
        # sensor_list.append(ego_cam6)

        # cam7_bp = None
        # cam7_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam7_bp.set_attribute("image_size_x",str(3840))
        # cam7_bp.set_attribute("image_size_y",str(2160))
        # cam7_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam7_location = carla.Location(2,0,1)
        # cam7_rotation = carla.Rotation(0,-30,0)
        # cam7_transform = carla.Transform(cam7_location,cam7_rotation)
        # ego_cam7 = world.spawn_actor(cam7_bp,cam7_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam7.listen(lambda image: sensor_callback(image, sensor_queue, "cam7"))
        # sensor_list.append(ego_cam7)

        # cam8_bp = None
        # cam8_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # cam8_bp.set_attribute("image_size_x",str(3840))
        # cam8_bp.set_attribute("image_size_y",str(2160))
        # cam8_bp.set_attribute("fov",str(105))
        # # cam2_bp.set_attribute("sensor_tick", str(0.033))
        # cam8_location = carla.Location(2,0,1)
        # cam8_rotation = carla.Rotation(0,45,0)
        # cam8_transform = carla.Transform(cam8_location,cam8_rotation)
        # ego_cam8 = world.spawn_actor(cam8_bp,cam8_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # ego_cam8.listen(lambda image: sensor_callback(image, sensor_queue, "cam8"))
        # sensor_list.append(ego_cam8)


        # Main loop
        while True:
            # Tick the server
            world.tick()
            w_frame = world.get_snapshot().frame
            print("\nWorld's frame: %d" % w_frame)

            # Now, we wait to the sensors data to be received.
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            # We include a timeout of 1.0 s (in the get method) and if some information is
            # not received in this time we continue.
            if w_frame%3==0:
                try:
                    for _ in range(len(sensor_list)):
                        s_frame = sensor_queue.get(True, 2.0)
                        print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

                except Empty:
                    print("    Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        for sensor in sensor_list:
            sensor.destroy()
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
