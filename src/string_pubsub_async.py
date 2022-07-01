#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

from std_msgs.msg import String

class GreetingWorker(object):

    def __init__(self):
        self.pub = rospy.Publisher('/greeting', String, queue_size=10)
        self.sub = rospy.Subscriber('/name', String, self.cbName)
        self.rate = rospy.Rate(1)
        self.name = ""
        self.run()
        
    def cbName(self, income_msg):
        self.name = income_msg.data
    
    def pubGreet(self):
        self.pub.publish('Hello, {}'.format(self.name))
        
    def run(self):
        while not rospy.is_shutdown():
            self.pubGreet()
            self.rate.sleep()

if __name__ == '__main__':

    rospy.init_node('greeting_indep_node')
    rospy.loginfo('Start Greeting Node')
    greeter = GreetingWorker()
    rospy.spin()