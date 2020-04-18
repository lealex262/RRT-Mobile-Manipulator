#!/usr/bin/env python
"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)
edited: Alex Le 
"""

import math
import random
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

import cmap
import move

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, map,
                 expand_dis=25.0, path_resolution=0.5, goal_sample_rate=1, max_iter=5000):
        """
        Setting Parameter

        start:Start Position [x,y,theta]
        goal:Goal Position [x,y,theta]
        """

        self.map = map
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = []


    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.collision_check(new_node):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                # self.draw_graph(rnd_node)
                pass

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.collision_check(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                # self.draw_graph(rnd_node)
                pass

        return None  # cannot find path


    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, to_node.theta)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(int(n_expand)):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node


    def collision_check(self, node):
        if node is None:
            return False

        # return self.map.collision_check((node.x, node.y))

        for pose in list(zip(node.path_x, node.path_y))[::5]:
            if not self.map.collision_check(pose):
                return False

        return True


    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.theta]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.theta])
            node = node.parent
        path.append([node.x, node.y, node.theta])

        return path


    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)


    def get_random_node(self):
        size = self.map.get_map_pixel_size()
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(0, size[0] - 1),
                            random.uniform(0, size[1] - 1),
                            random.uniform(0, math.pi * 2))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.theta)
        return rnd


    def draw_graph(self, rnd=None):
        self.map.draw_cmap(rnd, self.node_list, self.end, show=False)


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    # Test

    # Map
    map = cmap.Map()
    while map.get_cmap() is None or map.get_robot_position() is None:
        time.sleep(1)
        pass
  
    # Set Initial parameters
    start = map.position_2_map(np.hstack([map.get_robot_position(), map.get_robot_orientation()]))
    goal = map.position_2_map(np.array([5.0, -7.0, 3.14/2]))
    rrt = RRT(start, goal, map)

    # Search Path with RRT
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
        rrt.draw_graph()
        plt.show()
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y, theta) in path], [y for (x, y, theta) in path], '-r')
            plt.pause(0.01)  # Need for Mac
            plt.show()

    # Move Robot
    for ii in range(len(path)):
        path[ii] = move.node_2_goal(path[ii][0:2], path[ii][2], cmap=map)
    path = path[::-1]
    move.move_along_path(path)


if __name__ == '__main__':
    main()
