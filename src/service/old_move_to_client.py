#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from base_ros.srv import MoveTo

rospy.wait_for_service('/reset')
rospy.wait_for_service('/move_to')
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
            while delta_pose < dist: 
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

    def turn(self, dist):
            self.previous_pose_theta = self.current_pose_theta
            delta_pose = self.current_pose_theta - self.previous_pose_theta
            while abs(delta_pose) < abs(dist): 
                print("X: %f, Y: %f, Theta: %f Delta: %f" %(self.current_pose_x, self.current_pose_y, self.current_pose_theta, delta_pose))
                delta_pose = abs(self.current_pose_theta) - self.previous_pose_theta
                if dist > 0:
                    self.vel_publisher(0, 0.4)
                else:
                    self.vel_publisher(0, -0.4)
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
    move_to= rospy.ServiceProxy('move_to', MoveTo)
    distance = move_to.dist
    angle = move_to.theta
    r.move_forward_x(distance)
    r.turn(angle)
    reset_odom()

