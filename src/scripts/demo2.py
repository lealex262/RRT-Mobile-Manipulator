#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

import cmap
import move
import rrt as rt

def main():
    # Map
    map = cmap.Map()
  
    # Set Initial parameters
    start = map.position_2_map(np.hstack([map.get_robot_position(), map.get_robot_orientation()]))
    goal = map.position_2_map(np.array([5, 2.0, 0]))
    rrt = rt.RRT(start, goal, map)

    # Search Path with RRT
    path = rrt.planning(animation=True)

    if path is None:
        print("Cannot find path")
        rrt.draw_graph()
        plt.show()
    else:
        print("found path!!")

        rrt.draw_graph()
        rrt.draw_path(path)
        plt.pause(0.01)  # Need for Mac
        plt.show()

    # Move Robot
    rt.move_robot(path, map)

if __name__ == "__main__":
    main()