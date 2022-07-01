#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry

def quaternion_to_theta(odom):
    
    return(2*math.degrees(math.asin(odom.pose.pose.orientation.z)*math.copysign(1,odom.pose.pose.orientation.w)))

def callback(msg):
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y
    theta = quaternion_to_theta(msg)  
    print ('X = ', X)
    print ('Y = ', Y)
    print ('theta = ', theta)
        

rospy.init_node('odom_subscriber')
sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.loginfo("I am a odom_subscriber")
rospy.spin()