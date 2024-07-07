# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
import math
from math import atan2, pi, tan, sin, cos
import carla
import glob
import os
import sys
import pygame

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    import carla
except IndexError:
    pass


# Class for each tree node
class Node:
    def __init__(self, waypoint):
        self.x = waypoint.transform.location.x  # coordinate
        self.y = waypoint.transform.location.y  # coordinate
        self.waypoint = waypoint
        self.parent = None  # parent node / edge
        self.cost = 0.0  # cost to parent / edge weight



# Class for RRT
class RRT_VO:
    # Constructor
    def __init__(self, world, goal, obstacle):
        self.world = world
        self.obstacle = obstacle   #is a list of lists whose 1st element is the transformation and 2nd is velocity
        self.map = world.map
        self.world.carla_world.tick()
        trans = self.world.ego_vehicle.get_transform()
        x_ego = trans.location.x
        y_ego = trans.location.y
        start = self.map.get_waypoint(carla.Location(x=x_ego, y=y_ego), project_to_road=False)
        self.ego_yaw = trans.rotation.yaw*pi/180
        # print(self.obstacles[0][0].location, self.obstacles[0][0].rotation.yaw)
        # print(goal)
        self.start = Node(start)  # start node
        self.goal = Node(goal)  # goal node
        self.size_x = abs(self.start.x - self.goal.x) + 10  # map size
        self.size_y = abs(self.start.y - self.goal.y) + 10  # map size
        self.velocity_obstacle = None
        self.v_min = 4
        self.v_max = 10
        self.v = 8
        self.collision_radius = 2.7 #for tesla model 3





        self.vertices = []  # list of nodes
        self.waypoints_list = []  # list of waypoints
        self.found = False  # found flag
        self.path = []

        self.update_velocity_obstacle()

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def update_velocity_obstacle(self):

        yaw_obs = self.obstacle[0].rotation.yaw * pi / 180
        x_obs = self.obstacle[0].location.x
        y_obs = self.obstacle[0].location.y
        v_obs = self.obstacle[1]
        # m_obs = tan(yaw_obs)

        yaw_ego = self.ego_yaw
        x_ego = self.start.x
        y_ego = self.start.y
        # m_ego = tan(yaw_ego)

        angle = atan2(y_obs-y_ego, x_obs-x_ego)

        if v_obs == 0:
            self.velocity_obstacle = [x_obs, y_obs]
            return

        elif abs(yaw_obs - yaw_ego) <= 2*pi/180:
            print(abs((y_ego - y_obs)*cos(yaw_obs) - (x_ego - x_obs)*sin(yaw_obs)))
            if self.v_min <= v_obs <= self.v_max and abs((y_ego - y_obs)*cos(yaw_obs) - (x_ego - x_obs)*sin(yaw_obs)) <= 0.5:
                self.v = v_obs
                self.velocity_obstacle = None
                # print("here")
                return
            elif v_obs < self.v:
                th = angle - yaw_ego
                initial_dis = cos(th)*np.sqrt((x_obs - x_ego) ** 2 + (y_obs - y_ego) ** 2)
                final_dis = initial_dis*v_obs/(self.v - v_obs)
                y_velObs = final_dis * sin(yaw_obs) + y_obs
                x_velObs = final_dis * cos(yaw_obs) + x_obs
                self.velocity_obstacle = [x_velObs, y_velObs]
                # print(self.velocity_obstacle)
                return
            else:
                self.velocity_obstacle = None
                return

        elif abs(yaw_obs - yaw_ego) >= pi - (2*pi/180):
            th = angle - yaw_ego
            initial_dis = cos(th) * np.sqrt((x_obs - x_ego) ** 2 + (y_obs - y_ego) ** 2)
            final_dis = initial_dis * v_obs / (self.v + v_obs)
            y_velObs = final_dis * sin(yaw_obs) + y_obs
            x_velObs = final_dis * cos(yaw_obs) + x_obs
            self.velocity_obstacle = [x_velObs, y_velObs]
            # print(self.velocity_obstacle)
            return

        else:
            x_inter = cos(yaw_ego)*cos(yaw_obs)*(y_obs-y_ego) + x_ego*sin(yaw_ego)*cos(yaw_obs) - x_obs*sin(yaw_obs)*cos(yaw_ego)
            x_inter = x_inter/(sin(yaw_ego)*cos(yaw_obs) - sin(yaw_obs)*cos(yaw_ego))
            if (pi/2 - 2*pi/180) <= abs(yaw_ego) <= (pi/2 + 2*pi/180):
                y_inter = tan(yaw_obs) * (x_inter - x_obs) + y_obs
            else:
                y_inter = tan(yaw_ego) * (x_inter - x_ego) + y_ego

            ObsDistToInter = np.sqrt((x_obs - x_inter) ** 2 + (y_obs - y_inter) ** 2)
            EgoDistToInter = np.sqrt((x_ego - x_inter) ** 2 + (y_ego - y_inter) ** 2)
            r = self.collision_radius
            theta = abs(yaw_ego - yaw_obs) # angle between two trajectories
            # if theta <= pi/2:     # thinking something else
            #     collision_dis = r/sin(theta/2)
            # else:
            collision_dis = r/sin((pi-theta)/2)     # this is accurate

            t_min = (ObsDistToInter - collision_dis)/v_obs
            t_max = (ObsDistToInter + collision_dis)/v_obs

            v_top = (EgoDistToInter + collision_dis)/t_min  # Velocity obstacle upper range
            v_low = (EgoDistToInter - collision_dis)/t_max  # Velocity obstacle lower range
            print("V Range:")
            print(v_low, v_top)

            if self.v_min < v_low:
                self.v = (self.v_min + v_low) / 2
                self.velocity_obstacle = None
                print("Decreased Velocity")
                return

            elif self.v_max > v_top:
                self.v = (self.v_max + v_top) / 2
                self.velocity_obstacle = None
                print("Increased Velocity")
                return

            else:
                self.v = self.v_max
                t = (EgoDistToInter+5)/self.v
                final_dis = v_obs*t
                y_velObs = final_dis * sin(yaw_obs) + y_obs
                x_velObs = final_dis * cos(yaw_obs) + x_obs
                self.velocity_obstacle = [x_velObs, y_velObs]
                print("Avoiding Velocity Obstacle")
                return


    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        if self.velocity_obstacle is None:
            return False

        points_between = zip(np.linspace(node1.x, node2.x, dtype=int),
                             np.linspace(node1.y, node2.y, dtype=int))
        # check if any of these are obstacles
        yaw_node = atan2(node2.y-node1.y, node2.x-node1.x)
        for point in points_between:
            # self.world.carla_world.tick()
            yaw_obstacle = self.obstacle[0].rotation.yaw * pi/180
            x_obs = self.velocity_obstacle[0]
            y_obs = self.velocity_obstacle[1]
            # print(x_obs, y_obs)
            dist = np.sqrt((x_obs - point[0]) ** 2 + (y_obs - point[1]) ** 2)
            yaw = atan2(y_obs-point[1], x_obs-point[0])
            r_big = 2.7
            r_small = 1.66
            theta = 0.646
            # 37 degrees = 0.646 radians
            if -theta < (yaw-yaw_node) < theta or (pi - theta) < (yaw-yaw_node) <= pi \
                    or -pi <= (yaw-yaw_node) <= (-pi+theta):
                if -theta < (yaw - yaw_obstacle) < theta or (pi - theta) < (yaw - yaw_obstacle) <= pi \
                        or -pi <= (yaw - yaw_obstacle) <= (-pi + theta):
                    min_dist = 2*r_big
                else:
                    min_dist = r_big + r_small

            else:
                if -theta < (yaw - yaw_obstacle) < theta or (pi - theta) < (yaw - yaw_obstacle) <= pi \
                        or -pi <= (yaw - yaw_obstacle) <= (-pi + theta):
                    min_dist = r_big + r_small
                else:
                    min_dist = 2*r_small

            # print(min_dist, dist)
            if dist <= min_dist:
                # print("collision detected")
                return True

        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        # select goal
        if np.random.random() < goal_bias:
            point = [self.goal.x, self.goal.y]
        # or generate a random point
        else:
            trans = self.world.ego_vehicle.get_transform()
            # yaw = trans.rotation.yaw * pi / 180
            # while True:
            point = [np.random.randint(self.start.x-self.size_x, self.start.x+self.size_x), np.random.randint(self.start.y-self.size_y, self.start.y+self.size_y)]
                # angle = atan2(point[1] - self.start.y, point[0] - self.start.x)
                # if -pi/2 < (angle - yaw) < pi/2:
                #     break
        return point

    def get_random_ball(self):
        u = np.random.random()
        v = np.random.random()
        r = u ** 0.5  # sqrt function
        theta = 2 * math.pi * v
        x = r * math.cos(theta)
        y = r * math.sin(theta)

        return np.mat([y, x])

    def get_new_point_in_ellipsoid(self, goal_bias, c_best):
        '''Choose the goal or generate a random point in an ellipsoid
           defined by start, goal and current best length of path
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point
            c_best - the length of the current best path

        return:
            point - the new point
        '''
        # Select goal
        if np.random.random() < goal_bias:
            point = [self.goal.x, self.goal.y]

        #### TODO ####
        # Generate a random point in an ellipsoid
        else:
            c_min = self.dis(self.start, self.goal)
            x_center = [(self.goal.x + self.start.x) / 2, (self.goal.y + self.start.y) / 2]
            x_center = np.mat(x_center)
            # print(x_center)
            # theta = math.atan2(self.goal.row - self.start.row, -self.goal.col + self.start.col)
            theta = math.atan2((self.goal.y - self.start.y), (self.goal.x - self.start.x))

            C = [[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]]
            C = np.mat(C)

            L = np.mat([[c_best / 2, 0], [0, ((c_best ** 2 - c_min ** 2) ** 0.5) / 2]])
            # L = np.array(L)

            Xball = self.get_random_ball()

            x_rand = C * L * Xball.transpose() + x_center.transpose()
            x_rand = np.array(x_rand)
            # print(x_rand.transpose()[0][0], x_rand.transpose()[0][1])
            point = [x_rand.transpose()[0][0], x_rand.transpose()[0][1]]

            # pass
            # Compute the distance between start and goal - c_min

            # Calculate center of the ellipsoid - x_center

            # Compute rotation matrix from elipse to world frame - C

            # Compute diagonal matrix - L

            # Cast a sample from a unit ball - x_ball

            # Map ball sample to the ellipsoid - x_rand

        #### TODO END ####

        return point

    def get_nearest_node(self, point):
        '''Find the nearest node from the new point in self.vertices
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        # Use kdtree to find the neighbors within neighbor size
        samples = [[v.x, v.y] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        coord, ind = kdtree.query(point)
        return self.vertices[ind]

    def sample(self, goal_bias=0.05, c_best=0):
        '''Sample a random point in the area
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point
            c_best - the length of the current best path (For informed RRT)

        return:
            a new node if this node is valid and added, None if not.

        Generate a new point
        '''
        # Generate a new point

        #### TODO ####

        if c_best <= 0:
            new_point = self.get_new_point(goal_bias)  # del this

        else:
            new_point = self.get_new_point_in_ellipsoid(goal_bias, c_best)

        # Regular sampling if c_best <= 0
        # using self.get_new_point

        # Sampling in an ellipsoid if c_best is a positive value
        # using self.get_new_point_in_ellipsoid

        #### TODO END ####

        return new_point

    def extend(self, new_point, extend_dis=5):
        '''Extend a new node to the current tree structure
        arguments:
            new_point - the new sampled point in the map
            extend_dis - extension distance for each step

        return:
            a new node if this node is valid and added, None if not.

        Extend towards the new point and check feasibility.
        Create and add a new node if feasible.
        '''
        # Get nearest node
        new_waypoint = self.map.get_waypoint(carla.Location(x=new_point[0], y=new_point[1]), project_to_road=True)
        new_point = [new_waypoint.transform.location.x, new_waypoint.transform.location.y]
        nearest_node = self.get_nearest_node(new_point)

        # Calculate new node location
        slope = np.arctan2(new_point[1] - nearest_node.y, new_point[0] - nearest_node.x)
        new_x = nearest_node.x + extend_dis * np.cos(slope)
        new_y = nearest_node.y + extend_dis * np.sin(slope)

        # lane_tracking = True
        # for vehicle in self.world.vehicles:
        #     trans = vehicle.get_transform()
        #     x_obs = trans.location.x
        #     y_obs = trans.location.y
        #     dist = np.sqrt((x_obs - nearest_node.x) ** 2 + (y_obs - nearest_node.y) ** 2)
        #     if dist <= 15:
        #         lane_tracking = False
        #         break

        new_waypoint = self.map.get_waypoint(carla.Location(x=new_x, y=new_y), project_to_road=True)
        new_node = Node(new_waypoint)
        # print("here")

        # Check boundary and collision
        if (self.start.x-self.size_x <= new_node.x < self.start.x+self.size_x) and (self.start.y-self.size_y <= new_node.y < self.start.y+self.size_y) \
                and self.check_collision(nearest_node, new_node) == False:
            # If pass, add the new node
            new_node.parent = nearest_node
            new_node.cost = extend_dis
            # print(new_node.waypoint)
            self.vertices.append(new_node)

            # Check if goal is close
            if not self.found:
                d = self.dis(new_node, self.goal)

                if d < extend_dis:
                    self.goal.cost = d
                    self.goal.parent = new_node
                    self.vertices.append(self.goal)
                    self.found = True

            return new_node
        else:
            return None

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that is within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance
        '''
        # Use kdtree to find the neighbors within neighbor size
        samples = [[v.x, v.y] for v in self.vertices]
        kdtree = spatial.cKDTree(samples)
        ind = kdtree.query_ball_point([new_node.x, new_node.y], neighbor_size)
        neighbors = [self.vertices[i] for i in ind]
        # Remove the new_node itself
        neighbors.remove(new_node)
        return neighbors

    def generate_path(self, start_node, end_node):
        '''Compute path cost starting from start node to end node
        arguments:
            start_node - path start node
            end_node - path end node

        return:
            cost - path cost
        '''
        cost = 0
        curr_node = end_node
        path = []
        while start_node.x != curr_node.x or start_node.y != curr_node.y:
            # Keep tracing back until finding the start_node
            # or no path exists
            path.append(curr_node)
            print(curr_node.waypoint)
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
        path = path[::-1] #reversing the list
        return path

    def path_cost(self, start_node, end_node):
        '''Compute path cost starting from start node to end node
        arguments:
            start_node - path start node
            end_node - path end node

        return:
            cost - path cost
        '''
        cost = 0
        curr_node = end_node
        while start_node.x != curr_node.x or start_node.y != curr_node.y:
            # Keep tracing back until finding the start_node
            # or no path exists
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
            # print(cost)

        return cost

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        # If no neighbors, skip
        if neighbors == []:
            return

        # Compute the distance from the new node to the neighbor nodes
        distances = [self.dis(new_node, node) for node in neighbors]

        # Rewire the new node
        # compute the least potential cost
        costs = [d + self.path_cost(self.start, neighbors[i]) for i, d in enumerate(distances)]
        indices = np.argsort(np.array(costs))
        # check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break

        # Rewire new_node's neighbors
        for i, node in enumerate(neighbors):
            # new cost
            new_cost = self.path_cost(self.start, new_node) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(self.start, node) > new_cost and \
                    not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]

    # def draw_map(self):
    #     '''Visualization of the result
    #     '''
    #     # Create empty map
    #     fig, ax = plt.subplots(1)
    #     img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
    #     ax.imshow(img)
    #
    #     # Draw Trees or Sample points
    #     for node in self.vertices[1:-1]:
    #         plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
    #         plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
    #
    #     # Draw Final Path if found
    #     if self.found:
    #         cur = self.goal
    #         while cur.col != self.start.col or cur.row != self.start.row:
    #             plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
    #             cur = cur.parent
    #             plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')
    #
    #     # Draw start and goal
    #     plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
    #     plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')
    #
    #     # show image
    #     plt.show()

    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()
        # Start searching
        for i in range(n_pts):
            # Extend a new node until all the points are sampled
            # or find the path
            new_point = self.sample(0.05, 0)
            new_node = self.extend(new_point, 10)
            if self.found:
                break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.path_cost(self.start, self.goal)
            self.path = self.generate_path(self.start, self.goal)
            print("It took %d nodes to find the current paths" % steps)
            print("The path length is %.2f" % length)
        if not self.found:
            print("No path found")

        # Draw result
        # self.draw_map()

    def RRT_star(self, n_pts=1000, neighbor_size=10):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            neighbor_size - the neighbor distance

        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()
        # Start searching
        for i in range(n_pts):
            # Extend a new node
            new_point = self.sample(0.05, 0)
            new_node = self.extend(new_point, 8)
            # print(i)
            # Rewire
            if new_node is not None:
                neighbors = self.get_neighbors(new_node, neighbor_size)
                self.rewire(new_node, neighbors)

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.path_cost(self.start, self.goal)
            self.path = self.generate_path(self.start, self.goal)
            # print(self.path)
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        # self.draw_map()

    def informed_RRT_star(self, n_pts=1000, neighbor_size=20):
        '''Informed RRT* search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            neighbor_size - the neighbor distance

        In each step, extend a new node if possible, and rewire the node and its neighbors
        Once a path is found, an ellipsoid will be defined to constrained the sampling area
        '''
        # Remove previous result
        self.init_map()
        # Start searching
        for i in range(n_pts):

            #### TODO ####
            c_best = 0
            if self.found:
                c_best = self.path_cost(self.start, self.goal)
            # Once a path is found, update the best length of path - c_best
            # using the function self.path_cost(self.start, self.goal)

            #### TODO END ####

            # Extend a new node
            new_point = self.sample(0.05, c_best)
            new_node = self.extend(new_point, 10)
            # Rewire
            if new_node is not None:
                neighbors = self.get_neighbors(new_node, neighbor_size)
                self.rewire(new_node, neighbors)

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.path_cost(self.start, self.goal)
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        # self.draw_map()