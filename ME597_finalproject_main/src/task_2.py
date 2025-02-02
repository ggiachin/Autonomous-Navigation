#!/usr/bin/env python3

import sys
import os
import numpy as np

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from mapToGraph import Map, MapProcessor, Queue
from Astar import AStar
import matplotlib.pyplot as plt

class Navigation:
    """! Navigation node class.
    This class should server as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.startset = False
        self.endset = False

    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        self.endset = True

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        # MAKE SURE YOUR POSITION ESTIMATE IS GOOD ENOUGH.
        self.ttbot_pose.pose = data.pose.pose
        cov = data.pose.covariance

        update_speed = Twist()

        if cov[0] > 0.19:
            update_speed.angular.z = 0.1
            self.c = False
        else:
            update_speed.angular.z = 0
            self.c = True
        self.cmd_vel_pub.publish(update_speed)
        
    def pose_to_pixel(self,x,y):
        #resolution -> 0.05*384/200 = 0.096 for width
        #resolution -> 0.05*608/200 = 0.152 for height
        
        #resolutions and origin adjusted for increased accuracy
        x_pixel = (y / -0.15) + 59
        y_pixel = (x / 0.156) + 65

        return [round(x_pixel), round(y_pixel)]
        
    def pixel_to_pose(self,pixel):
        pixel = tuple(map(int, pixel.split(',')))
        x = (pixel[1] - 65) * 0.156
        y = (pixel[0] - 59) * -0.15
        return [x, y]
    
    def a_star_path_planner(self,start_pose,end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        
        #CONVERT POSE TO PIXEL POSITION IN MAP
        print(start_pose.pose.position.x, start_pose.pose.position.y, end_pose.pose.position.x, end_pose.pose.position.y)
        start_pixel = self.pose_to_pixel(start_pose.pose.position.x, start_pose.pose.position.y)
        end_pixel = self.pose_to_pixel(end_pose.pose.position.x, end_pose.pose.position.y)
        
        #IMPORT MAP
        mp = MapProcessor('map')
        kr = mp.rect_kernel(5,1)
        mp.inflate_map(kr, True)
        mp.get_graph_from_map()
        
        mp.map_graph.root = str(int(start_pixel[0]))+","+str(int(start_pixel[1]))
        mp.map_graph.end = str(int(end_pixel[0]))+","+str(int(end_pixel[1]))
        print(mp.map_graph.root, mp.map_graph.end)
      
        #calculate A* path
        as_maze = AStar(mp.map_graph) 
        as_maze.solve(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end])
        path_as,dist_as = as_maze.reconstruct_path(mp.map_graph.g[mp.map_graph.root],mp.map_graph.g[mp.map_graph.end])
        
        #append poses to path
        path.header.frame_id = "map"
        new_start_pose = PoseStamped()
        new_start_pose.pose.position.x = start_pose.pose.position.x
        new_start_pose.pose.position.y = start_pose.pose.position.x
        path.poses.append(new_start_pose)
        for i in range(1, len(path_as)-1):
            new_pose = PoseStamped()
            pos = self.pixel_to_pose(path_as[i])
            new_pose.pose.position.x = pos[0]
            new_pose.pose.position.y = pos[1]
            path.poses.append(new_pose)
        path.poses.append(end_pose)
        
        return path
    
    def get_path_idx(self,path,vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position int the path pointing to the next goal pose to follow.
        """
        # IMPLEMENT A MECHANISM TO DECIDE WHICH POINT IN THE PATH TO FOLLOW idx<=len(path)
        # Find minimum distance between current location and all poses in path
        
        min_dist = float("inf")
        min_index = 0
        
        for idx in range(len(path.poses)):
            path_pos = path.poses[idx]
            
            dist = np.sqrt((path_pos.pose.position.x-vehicle_pose.pose.position.x)**2 + (path_pos.pose.position.y-vehicle_pose.pose.position.y)**2)

            if dist < min_dist:
                min_dist = dist
                min_index = idx
                
        #Return the next index found

        return min_index+1


    def path_follower(self,vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0
        heading = 0
        # IMPLEMENT PATH FOLLOWER

        # PID variables
        K_dist = [1.3, 0, 0]
        K_angle = [0.5, 0.05, 0.3]
        iter_time = 0.1    
        integral_angle = 0
        error_prior_angle = 0
        integral_dist = 0
        error_prior_dist = 0
        
        # Calculate Errors
        error_dist = np.sqrt((current_goal_pose[0]-vehicle_pose.pose.position.x)**2 + (current_goal_pose[1]-vehicle_pose.pose.position.y)**2)
        
        #convert from quaternion to euler form (need yaw):
        v_r, v_p, v_yaw = euler_from_quaternion([vehicle_pose.pose.orientation.x, vehicle_pose.pose.orientation.y, vehicle_pose.pose.orientation.z, vehicle_pose.pose.orientation.w])
        
        tan_y = current_goal_pose[1]-vehicle_pose.pose.position.y
        tan_x = current_goal_pose[0]-vehicle_pose.pose.position.x
        angle_waypt = np.arctan2(tan_y,tan_x)
        error_angle = (angle_waypt - v_yaw + np.pi) % (2*np.pi) - np.pi
        
        integral_angle = integral_angle + (error_angle*iter_time)
        derivative_angle = (error_angle - error_prior_angle)/iter_time      
        integral_dist = integral_dist + (error_dist*iter_time)
        derivative_dist = (error_dist - error_prior_dist)/iter_time      
        
        #implement PID controller
        speed = K_dist[0]*error_dist + K_dist[1]*integral_dist +  K_dist[2]*derivative_dist
        heading = K_angle[0]*error_angle + K_angle[1]*integral_angle + K_angle[2]*derivative_angle

        return speed,heading

    def move_ttbot(self,speed,heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  speed     Desired yaw angle.
        @param  heading   Desired speed.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        # IMPLEMENT YOUR CONTROLLER LOW LEVEL CONTROLLER
        cmd_vel.angular.z = heading
        cmd_vel.linear.x = speed

        self.cmd_vel_pub.publish(cmd_vel)


    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        
        '''
            Main loop
        '''
        path_complete = False
        timeout = False
        idx = 0


        while self.startset == False and self.endset == False:
            ready = False
            
        # Create the path to follow
        path = self.a_star_path_planner(self.ttbot_pose,self.goal_pose)
        self.path_pub.publish(path)
        
        while not rospy.is_shutdown():

            # Loop through the path and move the robot
            idx = self.get_path_idx(path,self.ttbot_pose)
            if idx > len(path.poses) - 1:
                print("You arrived at your destination")
                self.move_ttbot(0,0)
                
            else:
                current_goal = [path.poses[idx].pose.position.x, path.poses[idx].pose.position.y]
                speed,heading = self.path_follower(self.ttbot_pose,current_goal)
                self.move_ttbot(speed,heading)

            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.node_name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
