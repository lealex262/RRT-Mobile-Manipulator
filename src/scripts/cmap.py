#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import matplotlib.pyplot as plt


def cmap_callback(msg):
    cmap_data = np.array(msg.data)
    width = msg.info.width
    height = msg.info.height
    cmap_data = cmap_data.reshape((height, width))

    cmap_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])

    draw_cmap(cmap_data)

# def position_callback(msg):
#     # Position
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y

#     # Orientation
#     orientation_q = msg.pose.pose.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#     theta = yaw

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
def costmap_listener():
    rospy.init_node('costmap_listener')
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, cmap_callback, queue_size=1)
    rospy.spin()

# """
# /odom [nav_msgs/Odometry] 1 publisher

# header: 
#   seq: 52446
#   stamp: 
#     secs: 524
#     nsecs: 611000000
#   frame_id: "odom"
# child_frame_id: "base_link"
# pose: 
#   pose: 
#     position: 
#       x: -0.0016886485426
#       y: 2.45101691152e-05
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -0.0181039317554
#       w: 0.999836110398
#   covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# """
# def robot_position_listener():
#     rospy.init_node
#     rospy.Subscriber('odom',Odometry, callback)


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