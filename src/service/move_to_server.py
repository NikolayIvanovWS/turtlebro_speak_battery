#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from base_ros.srv import MoveTo, MoveToRequest

rospy.wait_for_service('/reset')
reset_odom = rospy.ServiceProxy('/reset', Empty)

class RobotMover():
    
    def __init__(self):
        rospy.init_node('move_to_client')
        self.vel = Twist()
        self.current_pose_x = 0
        self.previous_pose_x = 0

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

    def turn(self, dist):
            self.previous_pose_theta = self.current_pose_theta
            delta_angle = 0
            angle = 0
            while abs(angle) < dist: 
                print("Current_Theta: %f, Previous_Theta: %f, Delta %f,  Angle: %f" %(abs(self.current_pose_theta), abs(self.current_pose_theta),abs(delta_angle), abs(angle)))
                delta_angle = abs(self.current_pose_theta) - abs(self.previous_pose_theta)
                angle = angle + abs(delta_angle)
                if dist > 0 :
                    self.vel_publisher(0, 0.4)
                else:
                    self.vel_publisher(0, -0.4)

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

def handle_move_to(self, req):
    self.service_dist = req.dist
    self.service_ang= req.theta
    self.go()
    
    
def move_to_server(self):
    s = rospy.Service('move_to', MoveTo, handle_move_to)
    self.service_dist = req.dist
    self.service_ang= req.theta
    rospy.spin()

if __name__ == "__main__":

    distance = move_to_server()
    angle = move_to.theta

    reset_odom()
    rospy.sleep(3)

    r.move_forward_x(distance)

    r.turn(angle)
    reset_odom()