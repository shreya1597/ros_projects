#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point 

def odom_callback(msg):
    print "------------------------------------------------"
    print "pose x = " + str(msg.pose.pose.position.x)
    print "pose y = " + str(msg.pose.pose.position.y)
    print "orientacion x = " + str(msg.pose.pose.orientation.x)
    print "orientacion y = " + str(msg.pose.pose.orientation.y)

rospy.init_node("robo_odom")
print "*********************"
sub= rospy.Subscriber('/odom', Odometry, odom_callback)
#pub = rospy.Publisher('/cmd_vel', Twist, queue_size =1)

## Import Odometry for navigation 
from nav_msgs.msg import Odometry