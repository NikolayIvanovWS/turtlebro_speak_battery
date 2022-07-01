#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from base_ros.srv  import AddTwoInts

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp = add_two_ints(x, y)
        return resp.sum

    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

if __name__ == "__main__":
    print(add_two_ints_client(10, 22))