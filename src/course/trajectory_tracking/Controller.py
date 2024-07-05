#!/usr/bin/env python3

import CUtils
import numpy as np
import math
import os
import queue
import time

class Controller(object):
    def __init__(self, waypoints, lateral_controller, longitudinal_controller):
        self.vars                = CUtils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self._latency            = 0
        ## LONGITUDINAL CONTROLLER PARAMETERS
        self._longitudinal_controller = longitudinal_controller
        # PID
        self._kP_a               = 1.1
        self._kI_a               = 0.35
        self._kD_a               = 0.27
        self._err_hist_a         = queue.Queue(500) # Limited error buffer
        # ALC
        self._k_t                = 0.5 # Agressiveness coefficient
        self._vel_limit          = 69.4444 # Speed limit (250 kmph)
        self._steer_limit        = 1.22 # Steering limit (70 deg)
        ## LATERAL CONTROLLER PARAMETERS
        self._lateral_controller = lateral_controller
        self._cte_ref_dist       = 1.5 # Distance from vehicle centre to front axle (m) (2 for Bang-Bang, 1.5 for PID and Stanley)
        self._eps_lookahead      = 10e-3 # Epsilon distance approximation threshold (m)
        self._closest_distance   = 0 # Distance to closest waypoint (m)
        # PID
        self._kP_s               = 0.12
        self._kI_s               = 0.01
        self._kD_s               = 0.48
        self._err_hist_s         = queue.Queue(500) # Limited error buffer
        # Pure-Pursuit
        self._Kpp                = 0.9
        self._min_lookahead_dis  = 10
        self._wheelbase          = 3.0
        # Stanley
        self._Kcte               = 1.5
        self._Ksoft              = 1e-5
        self._Kvel               = 1.3
        # POP
        self._min_lookahead_dist = 6
        self._Kv                 = 0.2
        self._steering_list      = np.linspace(-3, 3, 21, endpoint = True)

    def update_values(self, x, y, yaw, speed, timestamp, frame, closest_distance):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        self._closest_distance  = closest_distance
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad
        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def get_shifted_coordinate(self, x, y, yaw, length):
        x_shifted = x + length*np.cos(yaw)
        y_shifted = y + length*np.sin(yaw)
        return x_shifted, y_shifted

    def get_lookahead_dis(self, v):
        return max(self._min_lookahead_dis, self._Kpp*v)

    def get_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_lookahead_point_index(self, x, y, waypoints, lookahead_dis):
        for i in range(len(waypoints)):
            dis = self.get_distance(x, y, waypoints[i][0], waypoints[i][1])
            if abs(dis - lookahead_dis) <= self._eps_lookahead:
                return i
        return len(waypoints)-1

    def get_alpha(self, v1, v2, lookahead_dis):
        inner_prod = v1[0]*v2[0] + v1[1]*v2[1]
        return np.arccos(inner_prod/lookahead_dis)

    def get_steering_direction(self, v1, v2):
        corss_prod = v1[0]*v2[1] - v1[1]*v2[0]
        if corss_prod >= 0:
            return -1
        return 1

    def get_heading_error(self, waypoints, current_yaw):
        waypoint_delta_x = waypoints[1][0] - waypoints[0][0]
        waypoint_delta_y = waypoints[1][1] - waypoints[0][1]
        waypoint_heading = np.arctan(waypoint_delta_y/waypoint_delta_x)
        heading_error_mod = divmod((waypoint_heading - current_yaw), np.pi)[1]
        if heading_error_mod > np.pi/2 and heading_error_mod < np.pi:
            heading_error_mod -= np.pi
        return heading_error_mod

    def get_crosstrack_error(self, x, y, waypoints):
        P = np.asarray([x, y])
        i = self.get_lookahead_point_index(x, y, waypoints, self._cte_ref_dist) # Get current waypoint index
        if i == 0:
            A = np.asarray([waypoints[i][0], waypoints[i][1]])
            B = np.asarray([waypoints[i+1][0], waypoints[i+1][1]])
        else:
            A = np.asarray([waypoints[i-1][0], waypoints[i-1][1]])
            B = np.asarray([waypoints[i][0], waypoints[i][1]])
        n = B-A
        m = P-A
        dirxn = self.get_steering_direction(n, m)
        crosstrack_error = dirxn*(np.abs(((B[0]-A[0])*(A[1]-P[1]))-((A[0]-P[0])*(B[1]-A[1])))/np.sqrt((B[0]-A[0])**2+(B[1]-A[1])**2))
        return crosstrack_error

    def get_predicted_vehicle_location(self, x, y, steering_angle, yaw, v):
        wheel_heading = yaw + steering_angle
        wheel_traveled_dis = v * (self._current_timestamp - self.vars.t_previous)
        return [x + wheel_traveled_dis * np.cos(wheel_heading), y + wheel_traveled_dis * np.sin(wheel_heading)]

    def calculate_acceleration(self, t, v, v_desired, steer_output):
        if self._longitudinal_controller == 'PID':
            time_step = t - self.vars.t_previous
            current_speed_error = v_desired - v
            self._err_hist_a.put(current_speed_error)
            accumulated_speed_error = (self.vars.accumulated_speed_error + current_speed_error)
            if self._err_hist_a.full():
                accumulated_speed_error -= self._err_hist_a.get()
            speed_error_change = (current_speed_error - self.vars.previous_speed_error)
            p_term = self._kP_a * current_speed_error
            i_term = self._kI_a * accumulated_speed_error * time_step
            d_term = self._kD_a * speed_error_change / time_step
            acceleration = p_term + i_term + d_term
            self.vars.accumulated_speed_error = accumulated_speed_error
            self.vars.previous_speed_error = current_speed_error
            return acceleration
        elif self._longitudinal_controller == 'ALC':
            return self._k_t+((((self._vel_limit - v)/self._vel_limit) - (abs(steer_output)/self._steer_limit))*(1-self._k_t))
        else:
            return 0

    def calculate_steering(self, t, x, y, yaw, waypoints, v):
        if  self._lateral_controller == 'BangBang':
            time_stamp=time.time()
            crosstrack_error = self.get_crosstrack_error(x, y, waypoints)
            if crosstrack_error > 0:
                steering = 1.22*0.1
            elif crosstrack_error < 0:
                steering = -1.22*0.1
            else:
                steering = 0
            self._latency = (time.time()-time_stamp)*1000
            #print('Latency: ' + str(self._latency) + ' ms')
            return steering
        elif  self._lateral_controller == 'PID':
            time_stamp=time.time()
            time_step = t - self.vars.t_previous
            current_crosstrack_error = self.get_crosstrack_error(x, y, waypoints)
            self._err_hist_s.put(current_crosstrack_error)
            accumulated_crosstrack_error = (self.vars.accumulated_crosstrack_error + current_crosstrack_error)
            if self._err_hist_s.full():
                accumulated_crosstrack_error -= self._err_hist_s.get()
            crosstrack_error_change = (current_crosstrack_error - self.vars.previous_crosstrack_error)
            p_term = self._kP_s * current_crosstrack_error
            i_term = self._kI_s * accumulated_crosstrack_error * time_step
            d_term = self._kD_s * crosstrack_error_change / time_step
            steering = p_term + i_term + d_term
            self.vars.accumulated_crosstrack_error = accumulated_crosstrack_error
            self.vars.previous_crosstrack_error = current_crosstrack_error
            self._latency = (time.time()-time_stamp)*1000
            #print('Latency: ' + str(self._latency) + ' ms')
            return steering
        elif self._lateral_controller == 'PurePursuit':
            time_stamp=time.time()
            lookahead_dis = self.get_lookahead_dis(v)
            idx = self.get_lookahead_point_index(x, y, waypoints, lookahead_dis)
            v1 = [waypoints[idx][0] - x, waypoints[idx][1] - y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            alpha = self.get_alpha(v1, v2, lookahead_dis)
            if math.isnan(alpha):
                alpha = self.vars.alpha_previous
            if not math.isnan(alpha):
                self.vars.alpha_previous = alpha
            steering = self.get_steering_direction(v1, v2)*np.arctan((2*self._wheelbase*np.sin(alpha))/(lookahead_dis)) # Pure pursuit law
            if math.isnan(steering):
                steering = self.vars.steering_previous
            if not math.isnan(steering):
                self.vars.steering_previous = steering
            self._latency = (time.time()-time_stamp)*1000
            #print('Latency: ' + str(self._latency) + ' ms')
            return steering
        elif self._lateral_controller == 'Stanley':
            time_stamp=time.time()
            v1 = [waypoints[0][0] - x, waypoints[0][1] - y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            heading_error = self.get_heading_error(waypoints, yaw)
            cte_term = self._Kcte * self.get_crosstrack_error(x, y, waypoints)
            cte_term = np.arctan(cte_term/(self._Ksoft+self._Kvel*v))
            cte_term = divmod(cte_term, np.pi)[1]
            if cte_term > np.pi/2 and cte_term < np.pi:
                cte_term -= np.pi
            steering =  (heading_error + cte_term) # Stanley control law
            self._latency = (time.time()-time_stamp)*1000
            #print('Latency: ' + str(self._latency) + ' ms')
            return steering
        elif self._lateral_controller == 'POP':
            time_stamp=time.time()
            steering_list = self.vars.steering_previous + self._steering_list * self._pi/180 # List of steering angles in the neighbourhood (left and right) of previous steering angle
            lookahead_dist = self._min_lookahead_dist + self._Kv*v
            lp_idx = self.get_lookahead_point_index(x, y, waypoints, lookahead_dist)
            lookahead_point = [waypoints[lp_idx][0], waypoints[lp_idx][1]] # Select a lookahead point from the dynamically updated list of proximal waypoints
            min_dist = float("inf") # Initialize minimum distance value to infinity
            steering = self.vars.steering_previous # Set steering angle to previous value if the following optimization problem yields no acceptable solution (sanity check)
            for i in range(len(steering_list)):
                predicted_vehicle_location = self.get_predicted_vehicle_location(x, y, steering_list[i], yaw, v) # Get predicted vehicle location based on its current state and control input (i-th steering angle from the list)
                dist_to_lookahead_point = self.get_distance(predicted_vehicle_location[0], predicted_vehicle_location[1], lookahead_point[0], lookahead_point[1]) # Compute the distance between predicted vehicle location and lookahead point
                if dist_to_lookahead_point < min_dist: # Optimization problem (Minimize distance between predicted vehicle location and lookahead point to ensure effective path-tracking)
                    steering = steering_list[i] # Select the steering angle that minimizes distance between predicted vehicle location and lookahead point
                    min_dist = dist_to_lookahead_point # Update the minimum distance value
            self.vars.steering_previous = steering # Update previous steering angle value
            self._latency = (time.time()-time_stamp)*1000
            #print('Latency: ' + str(self._latency) + ' ms')
            return steering
        else:
            return 0

    #############################################################
    # CONTROL LOOP IMPLEMENTATION
    #############################################################

    def update_controls(self):
        #############################################################
        # RETRIEVE SIMULATOR FEEDBACK AND INITIALIZE CONTROL SIGNALS
        #############################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints

        throttle_output = 0
        brake_output    = 0
        steer_output    = 0

        ######################################################
        # DECLARE USAGE VARIABLES HERE
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('t_previous',0.0)
        self.vars.create_var('accumulated_speed_error',0.0)
        self.vars.create_var('previous_speed_error',0.0)
        self.vars.create_var('accumulated_crosstrack_error',0.0)
        self.vars.create_var('previous_crosstrack_error',0.0)
        self.vars.create_var('alpha_previous',0.0)
        self.vars.create_var('steering_previous',0.0)

        #############################################################
        # CONTROL LOOP IMPLEMENTATION
        #############################################################
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]:
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)

                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            # IMPLEMENTATION OF LATERAL CONTROLLER
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            # Change the steer output with the lateral controller.
            steer_output = self.calculate_steering(t, x, y, yaw, waypoints, v)

            ######################################################
            # IMPLEMENTATION OF LONGITUDINAL CONTROLLER
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            acc_command = self.calculate_acceleration(t, v, v_desired, steer_output)

            if acc_command > 0:
                throttle_output = acc_command
                brake_output    = 0
            else:
                throttle_output = 0
                brake_output    = -acc_command

            ######################################################
            # SET CONTROL SIGNAL OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in clamped percent (0 to 1)
            self.set_steer(steer_output)        # in radians (-1.22 to 1.22)
            self.set_brake(brake_output)        # in clamped percent (0 to 1)

            #########################################################
            # UPDATE VARIABLES FOR NEXT ITERATION OF CONTROL LOOP
            #########################################################
            """
                Use this block to store old values (for example, we can store the
                current x, y, and yaw values here using persistent variables for use
                in the next iteration)
            """

            self.vars.t_previous = t
