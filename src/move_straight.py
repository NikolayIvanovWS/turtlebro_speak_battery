#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
global distance_passed


def callback_handler(msg):
    global distance_passed
    distance_passed = msg.pose.pose.position.x

rospy.init_node('one_meter')
rospy.Subscriber('/odom', Odometry, callback_handler)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed = Twist()

rospy.sleep(1)

while distance_passed <= 1:
    print (distance_passed)
    speed.linear.x = 0.1
    pub.publish(speed)
    rospy.sleep(0.1)

speed.linear.x = 0
pub.publish(speed)
rospy.sleep(2)


while distance_passed >= 0:
    print (distance_passed)
    speed.linear.x = -0.1
    pub.publish(speed)
    rospy.sleep(0.1)


speed.linear.x = 0
pub.publish(speed)
rospy.sleep(1)