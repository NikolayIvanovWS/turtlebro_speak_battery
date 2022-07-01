#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cmd_vel1_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed = Twist()


rospy.sleep(1)
speed.linear.x = 0.1
speed.angular.z = 0
pub.publish(speed)
rospy.sleep(5)

speed.linear.x = 0
speed.angular.z = 2
pub.publish(speed)
rospy.sleep(3)

speed.linear.x = 0.1
speed.angular.z = 0
pub.publish(speed)
rospy.sleep(5)

speed.linear.x = 0
speed.angular.z = 2
pub.publish(speed)
rospy.sleep(3)

speed.linear.x = 0
speed.angular.z = 0
pub.publish(speed)
rospy.sleep(1)