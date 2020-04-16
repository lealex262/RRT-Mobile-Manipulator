#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt


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
def cmap_callback(msg):
    cmap_data = np.array(msg.data)
    width_pixel = msg.info.width
    height_pixel = msg.info.height
    cmap_data = cmap_data.reshape((height_pixel, width_pixel))

    cmap_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
    width_meter = height_pixel * 0.05
    height_meter = height_pixel * 0.05    

    draw_cmap(cmap_data)


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
def position_callback(msg):
    # Position
    x = msg.pose.position.x
    y = msg.pose.position.y

    # Orientation
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # theta = yaw


def costmap_listener():
    rospy.init_node('costmap_listener')
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, cmap_callback, queue_size=1)
    rospy.spin()


def robot_position_listener():
    rospy.init_node
    rospy.Subscriber('/pose', PoseStamped, callback)
    rospy.spin()


def draw_cmap(map):
    print("Draw map")
    fig, ax = plt.subplots()
    ax.imshow(map, cmap='gray_r')
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=0.5)
    plt.tick_params(axis='both', which='both', bottom=False,   
                    left=False, labelbottom=False, labelleft=False) 
    fig.set_size_inches((8.5, 11), forward=False)
    plt.show()
    

def main():
    costmap_listener()


if __name__ == "__main__":
    main()