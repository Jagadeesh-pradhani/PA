#!/usr/bin/env python3
import rospy
from std_msgs . msg import Int32
from geometry_msgs . msg import Twist

def callback(msg):
    print('Linear velocity is') 
    print(msg.linear) 
    print('Angular velocity is') 
    print (msg.angular)

rospy.init_node('pengwang_subscriber')
sub = rospy.Subscriber ('/cmd_vel', Twist , callback)
rospy.spin()