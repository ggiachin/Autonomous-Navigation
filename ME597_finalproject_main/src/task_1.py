#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
import time


class wallFollower:
    def __init__(self, node_name="wallFollower"):
        self.node_name = node_name
        self.rate = 0
        self.currentPose = PoseStamped()
        self.laser = []
        self.dataCollected = False

    def init_app(self):

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        print("initialised")

        #Subcribers
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/scan', LaserScan , self.laser_callback)
        #Publishers  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

    def pose_callback(self, data):
        self.currentPose = data
        rospy.loginfo('current_pose:{:.4f}'.format(self.currentPose.pose.position))

    def laser_callback(self, data):
        self.laser = data.ranges
        self.dataCollected = True

    def find_closest_wall(self):
        #laser angles 
        front = min(self.laser[340:360]+self.laser[0:20])
        right = min(self.laser[270:275])
        topRight = min(self.laser[300:330])
        bottomRight = min(self.laser[210:240])


        states = ['find wall', 'turn', 'follow wall']
        minDist = 0.3
        maxDist = 0.7
        megaDist = 1.1
        
        #states determined by how close to each wall -> to determine robots speed
        state = 0
        dif = right - topRight
        if topRight > maxDist and bottomRight < maxDist and right > maxDist:
           state = 3
           print("state = 3")
        elif front > maxDist and right > maxDist: #too far from wall
           state = 0 
           print("state = 0")
        elif front < maxDist and topRight < maxDist: #too close to front wall
            state = 1
            print("state 1 double")
            
        elif front > minDist and right < maxDist:
            if abs(dif) < 0.05:
                state = 2
                print("state 2 bc dif")
            elif dif > 0.04:
                state = 1
                print("state 1 bc dif")
            else:
                state = 0
                print("state 0 bc dif")
        elif front < 0.259: #front is stuck, robot cannot move
            state = 4
            print("robot is stuck")
        print()
            
            
        return state


       
    def run(self):
        print("Start of alg")
       
       
        while not self.dataCollected:
            ready = False
           
        print("Ready")
        cmd_vel = Twist()
        while not rospy.is_shutdown():

            state = self.find_closest_wall()

            #publish speed according to slected state above
            if state == 0: #find wall
                cmd_vel.angular.z = -0.2
                cmd_vel.linear.x = 0.2
            elif state == 1: #turn left
                cmd_vel.angular.z = 0.2
                cmd_vel.linear.x = 0
            elif state == 2: #follow wall
                cmd_vel.angular.z = 0
                cmd_vel.linear.x = 0.4  
            elif state == 3: #turn right
                cmd_vel.angular.z = -0.7
                cmd_vel.linear.x = 0
            else: #stuck
                cmd_vel.angular.z = -0.1
                cmd_vel.linear.x = -0.2          
           
            self.cmd_vel_pub.publish(cmd_vel)
           
            self.rate.sleep()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))

       

   
if __name__ == "__main__":

    explorer = wallFollower(node_name='wallFollower')
    explorer.init_app()
    try:
        explorer.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

