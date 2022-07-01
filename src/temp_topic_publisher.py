#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32

rospy.init_node('temp_publisher')
int_var = rospy.get_param("~my_integer", 0)
float_var = rospy.get_param("~my_float", 8.1)
string_var = rospy.get_param("~my_string", "blabla")
rospy.loginfo("Int: %s, Float: %s, String: %s", int_var, float_var, string_var)

pub = rospy.Publisher('/cpu_temp', Float32, queue_size=1)
temp = Float32()

def getCPUTemp():
    data = open('/sys/class/thermal/thermal_zone0/temp', 'r').read()
    return round(float(int(data)/1000.0),1)

while not rospy.is_shutdown():
    temp.data = getCPUTemp()
    pub.publish(temp)
    rospy.sleep(int_var)