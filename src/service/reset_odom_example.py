#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty

rospy.init_node('reset_world')

rospy.wait_for_service('/reset')
reset_world = rospy.ServiceProxy('/reset', Empty)

reset_world()