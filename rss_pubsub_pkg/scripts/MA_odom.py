#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class MoveDistance:
    def __init__(self):
        rospy.init_node('pengwang_pubsub', anonymous=True)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.target_distance = rospy.get_param('~target_distance', 1.0)  # Default to 1.0 meter
        self.start_position = None
        self.current_position = None
        self.distance_traveled = 0.0

        self.forward_speed = 0.2  # meters per second
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        if self.start_position is None:
            self.start_position = position
        self.current_position = position

        self.distance_traveled = self.calculate_distance(self.start_position, self.current_position)
        rospy.loginfo(f"Distance traveled: {self.distance_traveled:.2f} meters")

    def calculate_distance(self, start, current):
        return math.sqrt((current.x - start.x)**2 + (current.y - start.y)**2)

    def move(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.forward_speed

        while not rospy.is_shutdown() and self.distance_traveled < self.target_distance:
            self.vel_pub.publish(move_cmd)
            self.rate.sleep()

        # Stop the robot
        stop_cmd = Twist()
        self.vel_pub.publish(stop_cmd)
        rospy.loginfo("Target distance reached. Robot stopped.")

if __name__ == '__main__':
    try:
        move_distance = MoveDistance()
        move_distance.move()
    except rospy.ROSInterruptException:
        pass
