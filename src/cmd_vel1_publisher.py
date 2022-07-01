#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cmd_vel1_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)
speed = Twist()

while not rospy.is_shutdown():
    speed.linear.x = 0.16
    speed.angular.z = 2
    pub.publish(speed)
    rate.sleep()