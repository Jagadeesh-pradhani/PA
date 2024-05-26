#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from PA6.srv import turtlebot_move_square, turtlebot_move_squareResponse

def move_square(side_length, repetitions):
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    forward_speed = 0.2  # m/s
    turn_speed = 0.5  # rad/s
    forward_duration = side_length / forward_speed
    turn_duration = 1.57 / turn_speed  # 90 degrees turn

    move_cmd = Twist()
    move_cmd.linear.x = forward_speed
    move_cmd.angular.z = 0.0

    turn_cmd = Twist()
    turn_cmd.linear.x = 0.0
    turn_cmd.angular.z = turn_speed

    for _ in range(repetitions):
        for _ in range(4):
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < forward_duration:
                vel_pub.publish(move_cmd)
                rate.sleep()
            
            vel_pub.publish(Twist())  # Stop before turning
            rospy.sleep(1)
            
            start_time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start_time < turn_duration:
                vel_pub.publish(turn_cmd)
                rate.sleep()
            
            vel_pub.publish(Twist())  # Stop after turning
            rospy.sleep(1)

    vel_pub.publish(Twist())  # Stop at the end
    return True

def handle_move_square(req):
    success = move_square(req.sideLength, req.repetitions)
    return turtlebot_move_squareResponse(success)

def move_square_server():
    rospy.init_node('move_square_server')
    service = rospy.Service('turtlebot_move_square', turtlebot_move_square, handle_move_square)
    rospy.loginfo("Ready to move TurtleBot in a square.")
    rospy.spin()

if __name__ == "__main__":
    move_square_server()
