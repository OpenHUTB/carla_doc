#!/usr/bin/env python
# 参考：https://github.com/marcgpuig/carla_py_clients/blob/master/map_plot.py

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

import argparse

def main():
    argparser = argparse.ArgumentParser()
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

    # Approximate distance between the waypoints
    WAYPOINT_DISTANCE = 5.0 # in meters

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        world = client.get_world()
        carla_map = world.get_map()

        import matplotlib.pyplot as plt

        plt.subplot(211)
        # Invert the y axis since we follow UE4 coordinates
        plt.gca().invert_yaxis()
        plt.margins(x=0.7, y=0)

        # GET WAYPOINTS IN THE MAP ##########################################
        # Returns a list of waypoints positioned on the center of the lanes
        # all over the map with an approximate distance between them.
        waypoint_list = carla_map.generate_waypoints(WAYPOINT_DISTANCE)

        plt.plot(
            [wp.transform.location.x for wp in waypoint_list],
            [wp.transform.location.y for wp in waypoint_list],
            linestyle='', markersize=3, color='blue', marker='o')
        #####################################################################

        plt.subplot(212)
        # Invert the y axis since we follow UE4 coordinates
        plt.gca().invert_yaxis()
        plt.margins(x=0.7, y=0)

        # GET WAYPOINTS IN THE MAP ##########################################
        # It provides a minimal graph of the topology of the current OpenDRIVE file.
        # It is constituted by a list of pairs of waypoints, where the first waypoint
        # is the origin and the second one is the destination.
        # It can be loaded into NetworkX.
        # A valid output could be: [ (w0, w1), (w0, w2), (w1, w3), (w2, w3), (w0, w4) ]
        topology = carla_map.get_topology()
        road_list = []

        for wp_pair in topology:
            current_wp = wp_pair[0]
            # Check if there is a road with no previus road, this can happen
            # in opendrive. Then just continue.
            if current_wp is None:
                continue
            # First waypoint on the road that goes from wp_pair[0] to wp_pair[1].
            current_road_id = current_wp.road_id
            wps_in_single_road = [current_wp]
            # While current_wp has the same road_id (has not arrived to next road).
            while current_wp.road_id == current_road_id:
                # Check for next waypoints in aprox distance.
                available_next_wps = current_wp.next(WAYPOINT_DISTANCE)
                # If there is next waypoint/s?
                if available_next_wps:
                    # We must take the first ([0]) element because next(dist) can
                    # return multiple waypoints in intersections.
                    current_wp = available_next_wps[0]
                    wps_in_single_road.append(current_wp)
                else: # If there is no more waypoints we can stop searching for more.
                    break
            road_list.append(wps_in_single_road)

        # Plot each road (on a different color by default)
        for road in road_list:
            plt.plot(
                [wp.transform.location.x for wp in road],
                [wp.transform.location.y for wp in road])
        #####################################################################

        plt.show()

    finally:
        pass


if __name__ == '__main__':
    try:
        main()
    finally:
        print('Done.')
