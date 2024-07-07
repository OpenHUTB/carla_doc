import numpy as np
from math import atan, sin, cos, pi
from matplotlib import pyplot as plt
import copy

class motion_primitive():

    def __init__(self, theta_i, theta_f, xi, xf, yi, yf):
        self.Ti = 0
        self.Tf = 1
        self.Vr = 1
        self.l = 2  # car length must be set manually

        #   Theta
        self.theta_i = theta_i
        self.theta_f = theta_f

        #   X Co_Ord
        self.xi = xi
        self.xi_dot = self.Vr*cos(theta_i)
        self.xf = xf
        self.xf_dot = self.Vr*cos(theta_f)

        #   Y Co_Ord
        self.yi = yi
        self.yi_dot = self.Vr*sin(theta_i)
        self.yf = yf
        self.yf_dot = self.Vr*sin(theta_f)

    def cubic_T_Matrix(self):
        self.T_matrix = np.matrix([[1, self.Ti, self.Ti ** 2, self.Ti ** 3],
                                  [0, 1, 2 * self.Ti, 3 * self.Ti ** 2],
                                  [1, self.Tf, self.Tf ** 2, self.Tf ** 3],
                                  [0, 1, 2 * self.Tf, 3 * self.Tf ** 2]])
        self.T_inv = np.linalg.inv(self.T_matrix)

    def cubic_trajectory(self, X_matrix):
        Co_ef = np.dot(self.T_inv, X_matrix)
        return Co_ef

    def trajectory(self):
        self.theta_matrix = np.matrix([[self.theta_i], [0], [self.theta_f], [0]])
        self.theta_Co_ef = self.cubic_trajectory(self.theta_matrix)

        self.x_matrix = np.matrix([[self.xi], [self.xi_dot], [self.xf], [self.xf_dot]])
        self.y_matrix = np.matrix([[self.yi], [self.yi_dot], [self.yf], [self.yf_dot]])

        self.x_Co_ef = self.cubic_trajectory(self.x_matrix)
        self.y_Co_ef = self.cubic_trajectory(self.y_matrix)

    def get_state(self, T):
        T_d1 = np.matrix([1, T, T**2, T**3])
        T_d2 = np.matrix([0, 1, 2*T, 3*T**2])

        x = np.dot(T_d1, self.x_Co_ef)
        x_dot = np.dot(T_d2, self.x_Co_ef)
        y = np.dot(T_d1, self.y_Co_ef)
        y_dot = np.dot(T_d2, self.y_Co_ef)
        theta = np.dot(T_d1, self.theta_Co_ef)
        theta_dot = np.dot(T_d2, self.theta_Co_ef)

        return x, x_dot, y, y_dot, theta, theta_dot

    def get_steering_angle(self, theta_dot, Vr):
        return atan((theta_dot*self.l)/Vr)

    def get_path(self, step):
        i = 0
        pos_x = []
        pos_y = []
        while i < self.Tf - self.Ti:
            val = np.squeeze(np.asarray(self.get_state(i)))
            pos_x.append(val[0])
            pos_y.append(val[2])
            i += step

        return pos_x, pos_y

if __name__ == '__main__':

    primitive = motion_primitive(0, pi/2, 1, 2, 0, 5)
    primitive.cubic_T_Matrix()
    primitive.trajectory()
    pos_x, pos_y = primitive.get_path(0.001)
    plt.scatter(pos_x, pos_y)
    plt.show()