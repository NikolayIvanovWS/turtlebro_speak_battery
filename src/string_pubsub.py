#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from std_msgs.msg import String

class GreetingWorker(object):

    def __init__(self):
        self.pub = rospy.Publisher('/greeting', String, queue_size=10)

    def sayHello(self, income_msg):
        self.pub.publish('Hello, {}'.format(income_msg.data))

if __name__ == '__main__':

    rospy.init_node('greeting_node')
    rospy.loginfo("Start Greeting Node")

    greeter = GreetingWorker()

    rospy.Subscriber("/name", String, greeter.sayHello)
    rospy.spin()