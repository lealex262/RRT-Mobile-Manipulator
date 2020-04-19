#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

import numpy as np
import matplotlib.pyplot as plt
import time
import math


class Map:

    def __init__(self):
        self.tick = 0
        self.robot_position = None
        self.robot_orientation = None
        self.map_position = None
        self.map_size = None
        self.map_pixel_size = None
        self.cmap = None
        self.resolution = 0.05
        self.robot_pixel_size = 6
        self.robot_size = self.robot_pixel_size * self.resolution

        # Run Subscribe nodes
        rospy.init_node('map_listener')
        print("Run position listner")
        self.robot_position_listener()
        print("Run cost map listner")
        self.costmap_listener()

        # Wait
        while self.get_cmap() is None or self.get_robot_position() is None:
            time.sleep(1)


    """
    /move_base/global_costmap/costmap [nav_msgs/OccupancyGrid] 1 publisher

    header: 
        seq: 0
        stamp: 
            secs: 388
            nsecs: 689000000
        frame_id: "map"
    info: 
        map_load_time: 
            secs: 0
            nsecs:         0
        resolution: 0.0500000007451
        width: 312
        height: 345
        origin: 
            position: 
                x: -8.458448
                y: -8.348908
                z: 0.0
            orientation: 
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
    data: []

    note: y then x
    """
    def cmap_callback(self, msg):
        cmap_data = np.array(msg.data)
        width_pixel = msg.info.width
        height_pixel = msg.info.height
        cmap_data = cmap_data.reshape((height_pixel, width_pixel))
        self.cmap = cmap_data

        self.map_position = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        width_meter = width_pixel * self.resolution
        height_meter = height_pixel * self.resolution
        self.map_size = np.array((width_meter,height_meter))
        self.map_pixel_size = np.array((width_pixel, height_pixel))


    """
    /pose [geometry_msgs/PoseStamped] 1 publisher

    header: 
        seq: 190
        stamp: 
            secs: 1120
            nsecs: 244000000
        frame_id: "/map"
    pose: 
        position: 
            x: 5.05099202886
            y: 4.95224533782
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.036077132877
            w: 0.999349008347
    """
    def position_callback(self, msg):
        self.tick = (self.tick + 1) % 10
        if self.tick % 10 == 0:
            # Positionpose
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.robot_position = np.array((x, y))

            # Orientatpose
            orientation_q = msg.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            theta = yaw
            self.robot_orientation = theta


    def costmap_listener(self):
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.cmap_callback, queue_size=1)


    def robot_position_listener(self):
        rospy.Subscriber('/pose', PoseStamped, self.position_callback)


    def get_robot_position(self):
        return self.robot_position


    def get_robot_orientation(self):
        return self.robot_orientation


    def get_map_position(self):
        return self.map_position


    def get_map_size(self):
        return self.map_size


    def get_map_pixel_size(self):
        return self.map_pixel_size


    def get_cmap(self):
        return self.cmap


    """
    pixel = meter/resolution
    """
    def position_2_map(self, pose):
        pose[:2] = (pose[:2] - self.map_position) /self.resolution
        return tuple(pose)

    """
    meter = pixel * resolution
    """
    def map_2_position(self, pose):
        pose[:2] = pose[:2] * self.resolution
        pose[:2] += self.map_position
        return tuple(pose)


    """
    True = no collision
    """
    def collision_check(self, position):
        if self.cmap is not None:
            x = int(position[0])
            y = int(position[1])
            radius = int(self.robot_pixel_size/2)
            positions = [(x, y), (x + radius, y), (x, y + radius), (x + radius, y + radius), (x - radius, y), (x, y - radius), (x - radius, y - radius), (x + radius, y - radius), (x - radius, y + radius)]
            # positions = [(x, y)]

            for pose in positions:
                if self.cmap[pose[1],pose[0]] != 0:
                    return False
        else:
            print("Cmap not ready")
            return False

        return True


    def draw_cmap(self, rnd_node=None, node_list=None, goal=None, show=True):
        if self.cmap is not None:
            # Clear
            plt.clf()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

            # Draw
            print("Draw map")
            # ax = self.ax
            # fig = self.fig
            # self.im.set_data(self.cmap)
            fig, ax = plt.subplots()
            ax.imshow(self.cmap, cmap='gray_r')
            
            # Draw Robot
            if self.robot_position is not None:
            
                x, y = self.position_2_map(self.robot_position)

                # Draw robot size
                deg = list(range(0, 360, 5))
                deg.append(0)
                xl = [x + self.robot_pixel_size * math.cos(np.deg2rad(d)) for d in deg]
                yl = [y + self.robot_pixel_size * math.sin(np.deg2rad(d)) for d in deg]
                ax.plot(xl, yl, "-r")

                # Draw robot pose
                self.draw_node(ax, x, y, self.robot_orientation, self.robot_pixel_size, color='r')

            # Draw goal
            if goal is not None:
                self.draw_node(ax, goal.x, goal.y, goal.theta, self.robot_pixel_size, color='r')

            # Draw random new node
            if rnd_node is not None:
                self.draw_node(ax, rnd_node.x, rnd_node.y, rnd_node.theta, self.robot_pixel_size/2, color='b')


            # Draw other nodes
            if node_list is not None:
                for node in node_list:
                    # Node
                    self.draw_node(ax, node.x, node.y, node.theta, self.robot_pixel_size/2, color='b')

                    # Edge
                    plt.plot(node.path_x, node.path_y, "-g")


            # Show
            ax.tick_params(axis='both', which='both', bottom=False,   
                            left=False, labelbottom=False, labelleft=False) 
            fig.set_size_inches((16, 20), forward=False)
            plt.grid(True)
            # plt.draw()
            # plt.pause(0.01)

            if show:
                plt.show()
            
            return True
        
        else:
            print("No Map")
            return False

    def draw_node(self, ax, x, y, theta, size, color='r'):
        # Draw Arrow head
        dx = math.cos(theta)
        dy = math.sin(theta)
        x -= dx * size/2
        y -= dy * size/2
        ax.arrow(x, y, dx, dy, color=color, head_width=size, head_length=size, overhang=0.4)
    

def main():
    # Test
    map = Map()
    map.draw_cmap()


if __name__ == "__main__":
    main()