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
        self.cmap = None
        self.robot_size = 10
        self.resolution = 0.05

        # Run Subscribe nodes
        rospy.init_node('map_listener')
        print("Run position listner")
        self.robot_position_listener()
        print("Run cost map listner")
        self.costmap_listener()


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
    data: [[]]
    """
    def cmap_callback(self, msg):
        cmap_data = np.array(msg.data)
        width_pixel = msg.info.width
        height_pixel = msg.info.height
        cmap_data = cmap_data.reshape((height_pixel, width_pixel))
        self.cmap = cmap_data

        self.map_position = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        width_meter = height_pixel * self.resolution
        height_meter = height_pixel * self.resolution
        self.map_size = np.array((width_meter,height_meter))


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
            # Position
            x = msg.pose.position.x
            y = msg.pose.position.y
            self.robot_position = np.array((x, y))

            # Orientation
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


    """
    pixel = meter/resolution
    """
    def position_2_map(self, pose):
        pose = (pose - self.map_position) /self.resolution
        return tuple(pose)

    """
    meter = pixel * resolution
    """
    def map_2_position(self, pose):
        pose = pose * self.resolution
        pose += self.map_position
        return tuple(pose)


    def draw_cmap(self):
        if self.cmap is not None:
            # Clear
            plt.clf()

            # Draw
            print("Draw map")
            fig, ax = plt.subplots()
            ax.imshow(self.cmap, cmap='gray_r')
            
            # Draw Robot
            if self.robot_position is not None:
            
                x, y = self.position_2_map(self.robot_position)

                # Draw robot size
                deg = list(range(0, 360, 5))
                deg.append(0)
                xl = [x + self.robot_size * math.cos(np.deg2rad(d)) for d in deg]
                yl = [y + self.robot_size * math.sin(np.deg2rad(d)) for d in deg]
                ax.plot(xl, yl, "-b")

                # Draw Arrow head
                dx = math.cos(self.robot_orientation)
                dy = math.sin(self.robot_orientation)
                x -= dx * self.robot_size/2
                y -= dy * self.robot_size/2
                ax.arrow(x, y, dx, dy, color='b', head_width=self.robot_size, head_length=self.robot_size, overhang=0.4)

            # Show
            ax.tick_params(axis='both', which='both', bottom=False,   
                            left=False, labelbottom=False, labelleft=False) 
            fig.set_size_inches((8.5, 11), forward=False)
            plt.show()
            return True
        
        else:
            print("No Map")
            return False
    

def main():
    map = Map()
    time.sleep(1)
    while not map.draw_cmap():
        time.sleep(1)
        pass


if __name__ == "__main__":
    main()