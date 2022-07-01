#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

def callback(msg):
    print (msg.data)
    
rospy.init_node('temp_subscriber')
sub = rospy.Subscriber('cpu_temp', Float32, callback)
rospy.loginfo("I am a subscriber")
rospy.spin()