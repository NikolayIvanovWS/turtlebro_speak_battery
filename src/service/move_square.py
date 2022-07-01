#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.wait_for_service('/reset')
reset_odom = rospy.ServiceProxy('/reset', Empty)

class RobotMover():
    
    def __init__(self):
        rospy.init_node('move_to_client')
        self.vel = Twist()
        self.current_pose_x = 0
        self.previous_pose_x = 0

        self.current_pose_y = 0
        self.previous_pose_y = 0

        self.current_pose_theta = 0
        self.previous_pose_theta = 0

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def quaternion_to_theta(self, odom):
        return(2*math.degrees(math.asin(odom.pose.pose.orientation.z)*math.copysign(1,odom.pose.pose.orientation.w)))

    def odom_cb(self, msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
        self.current_pose_theta = self.quaternion_to_theta(msg)

    def move_forward_x(self, dist):
            self.previous_pose_x = self.current_pose_x
            delta_pose = abs(self.current_pose_x) - abs(self.previous_pose_x)
            while abs(delta_pose) < dist: 
                print("X: %f, Y: %f, Theta: %f Delta: %f" %(self.current_pose_x, self.current_pose_y, self.current_pose_theta, delta_pose))
                delta_pose = self.current_pose_x - self.previous_pose_x
                self.vel_publisher(0.2, 0)
                rospy.sleep(0.01)
            else:
                self.vel_publisher(0, 0)
                rospy.sleep(1)  

    def move_forward_y(self, dist):
            self.previous_pose_y = self.current_pose_y
            delta_pose = abs(self.current_pose_y) - abs(self.previous_pose_y)
            while delta_pose < dist: 
                print("X: %f, Y: %f, Theta: %f Delta: %f" %(self.current_pose_x, self.current_pose_y, self.current_pose_theta, delta_pose))
                delta_pose = self.current_pose_y - self.previous_pose_y
                self.vel_publisher(0.2, 0)
                rospy.sleep(0.01)
            else:
                self.vel_publisher(0, 0)
                rospy.sleep(1) 

    def turn_left(self, dist):
            self.previous_pose_theta = self.current_pose_theta
            delta_angle = 0
            angle = 0
            while abs(angle) < dist: 
                print("Current_Theta: %f, Previous_Theta: %f, Delta %f,  Angle: %f" %(abs(self.current_pose_theta), abs(self.current_pose_theta),abs(delta_angle), abs(angle)))
                delta_angle = abs(self.current_pose_theta) - abs(self.previous_pose_theta)
                angle = angle + abs(delta_angle)
                self.vel_publisher(0, 0.4)
                self.previous_pose_theta = self.current_pose_theta
                rospy.sleep(0.05)                
            else:
                self.vel_publisher(0, 0)
                rospy.sleep(1)
            
            
    def vel_publisher(self, vel_value, ang_value):
        self.vel.linear.x = vel_value
        self.vel.angular.z = ang_value
        self.pub.publish(self.vel)
        
r = RobotMover()

if __name__ == "__main__":
    reset_odom()
    rospy.sleep(3)
    r.move_forward_x(0.2)

    r.turn_left(88)

    r.move_forward_y(0.2)

    r.turn_left(88)

    r.move_forward_x(0.2)

    r.turn_left(88)

    r.move_forward_y(0.2)

    r.turn_left(88)
    reset_odom()