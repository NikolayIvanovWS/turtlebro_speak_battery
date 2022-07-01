#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
from base_ros.msg import ImuRaw

rospy.init_node('ImuRawNode')
pub = rospy.Publisher('/raw_imu_test', ImuRaw, queue_size=1)
rate = rospy.Rate(1)
Imu = ImuRaw()


while not rospy.is_shutdown():
    Imu.acc.ax = random.random()
    Imu.acc.ay = random.random()
    Imu.acc.az = random.random()
    Imu.gyro.gx = random.random()
    Imu.gyro.gy = random.random()
    Imu.gyro.gz = random.random()
    pub.publish(Imu)
    rate.sleep()