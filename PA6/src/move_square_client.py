#!/usr/bin/env python3

import rospy
from PA6.srv import turtlebot_move_square

def move_square_client(side_length, repetitions):
    rospy.wait_for_service('turtlebot_move_square')
    try:
        move_square = rospy.ServiceProxy('turtlebot_move_square', turtlebot_move_square)
        resp = move_square(side_length, repetitions)
        return resp.successful
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

if __name__ == "__main__":
    rospy.init_node('move_square_client')
    side_length = 1.0  # Example side length
    repetitions = 4  # Example repetitions
    success = move_square_client(side_length, repetitions)
    if success:
        rospy.loginfo("TurtleBot successfully moved in a square.")
    else:
        rospy.loginfo("TurtleBot failed to move in a square.")
