# Autonomous-Navigation
Final Project from ME597 Autonomous Systems Course at Purdue


# ME597_Final_Project_S23
In this project, I used ROS and Gazebo to automate a Turtlebot3 Waffle Pi to perform three taks: (1) autonomous exploration, (2) navigation in static environment, and (3) navigation in dynamic environment.

The first part is to explore an area autonomously and generate a map by using a wall following algorithm and the turtlebot's built in SLAM techniques. Navigating autonomously the entire area had to be done under 10 minutes. Moreover, the program had to be robust enough to work on any given space.

The second part consisted on navigating that area to an input goal location using the map generated in the first part. For this, the map was converted from pixels to meters, A star algorithm was implemented to go from the initial to goal pose, and a PID was introduced and tuned to follow the path precisely.

The third and final part adds a new variable to the second milestone: moving objects. The goal is to navigate from initial pose to end pose, using the map generated, and avoid the moving green trash cans. For this, the turtlebot's front camera was used; these images were processed to detect green. MOreover, the PID controller was modified to bring the robot to a stop whenever the trash cans where detected and move again once the way is clear.

## Clone and build
git clone this repo to your workspace's src folder, then do `catkin build` or `catkin_make` in workspace directory.

## Set environment variable of the robot model we use for this project
`export TURTLEBOT3_MODEL=waffle`

## Source the package in the workspace directory
`source devel/setup.bash`

## Task 1 - Autonomous exploration
`roslaunch final_project task_1.launch`

After you have mapped the whole environment, save map in another terminal by 
`rosrun map_server map_saver -f map`

## Task 2 - Navigation in static environment
`roslaunch final_project task_2.launch`

## Task 3 - Navigation in environment with dynamic obstacles
`roslaunch final_project task_3.launch`


