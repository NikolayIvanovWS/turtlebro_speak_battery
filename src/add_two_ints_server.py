#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from base_ros.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.x, req.y, (req.x + req.y)))
    return AddTwoIntsResponse(req.x + req.y)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print('Ready to add two ints.')
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()