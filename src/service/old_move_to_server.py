#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from base_ros.srv import MoveTo, MoveToResponse

def handle_move_to(req):
    if req.dist > 1:
        ans = "Too long"
        print ("Too long")       
    else:
        print ("Ok")
        ans = "Ok"

    return MoveToResponse(ans)

def move_to_server():
    rospy.init_node('move_to_server')
    s = rospy.Service('move_to', MoveTo, handle_move_to)
    print('Ready to add distance and angle.')
    rospy.spin()

if __name__ == "__main__":
    move_to_server()