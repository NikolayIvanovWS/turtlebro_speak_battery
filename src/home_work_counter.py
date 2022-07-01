#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

rospy.init_node('counter_publisher')
pub = rospy.Publisher('/counter', Int32, queue_size=1)
rate = rospy.Rate(5)
count = Int32()
count = 0

while not rospy.is_shutdown():
    count = count + 1 
    pub.publish(count)
    rate.sleep()