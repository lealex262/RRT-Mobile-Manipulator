#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Setup node
rospy.init_node("send_goal")


def move_to_goal(goal):
    # Setup client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo('Waiting for the action server to start')
    client.wait_for_server()

    # Create Goal
    rospy.loginfo('Action server started, sending the goal')
    current_goal = MoveBaseGoal()

    # Goal Params
    current_goal.target_pose.header.stamp = rospy.Time.now()
    current_goal.target_pose.header.frame_id = 'map'

    # Set Position
    current_goal.target_pose.pose.position.x = goal[0][0]
    current_goal.target_pose.pose.position.y = goal[0][1]
    current_goal.target_pose.pose.position.z = goal[0][2]

    # Set Orientation
    current_goal.target_pose.pose.orientation.x = goal[1][0]
    current_goal.target_pose.pose.orientation.y = goal[1][1]
    current_goal.target_pose.pose.orientation.z = goal[1][2]
    current_goal.target_pose.pose.orientation.w = goal[1][3]

    # Send
    client.send_goal(current_goal)
    rospy.loginfo('Waiting for the result')
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")

    # Get Results
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo('Succeeded')
        return True
    else:
        rospy.loginfo('Failed')
        return False


def move_along_path(path):
    for goal in path:
        if not move_to_goal(goal):
            return False
    return True


if __name__ == "__main__":
    # Test
    path = [((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), ((5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),((5.0, 5.0, 0.0), (0.0, 0.0, 0.0, 1.0))]
    move_along_path(path)