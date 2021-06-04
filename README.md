# turtlebot3_multiple_goal_navigatiion
This repository is created to send the list of desired goals to robot in robots "map" co-ordinate system using navigation stack of ROS.

# About the node
The "my_multi_goal_send" node send the list of goals specified in the robots "map" coordinate system. You can specify X, Y and orientation angle of robot along its Z axis. The robot will go all the goals listed in the list and in the end prints the time required by the robot for completing the robot. At any instance, one can cancel the goal and in that case the robot will go back to its initial spawned position and print the time requird to drive the route. 

# How to install the node

1. For this node ROS Melodic Version is used and progamming langauge python is used
2. In order to run the node, you should have turtlebot3_gazebo, turtlebot3_simulation, turtlebot3_navigation, turtlebot3_teleop, turtlebot3_slam and turtlebot3_msgs packages install.
   ```
   sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations ros-melodic-turtlebot3-teleop ros-melodic-turtlebot3-navigation ros-melodic-turtlebot3-slam ros-melodic-turtlebot3-msgs
   ```
3.Create a folder name 'my_multi_goal_send' in the src directory of your catkin workspace. This folder will represent the package name. Clone the master branch of this repository   into newly created 'my_multi_goal_send' directory.
4. Move all the file in the 'turtlebot3_multiple_goal_navigation' navigation into the 'my_multi_goal_send' directory.
5. Move the map.pgm and map.yaml file to the directory 'maps'directory of 'turtlebot3_navigation' package.  
   path:- /opt/ros/melodic/share/turtlebot3_navigation/maps
7. Make the 'my_multi_goal_send.py' file executable.
   ```
   cd 
   cd catkin_ws/src/my_multi_goal_send/src
   chmod +x my_multi_goal_send.py
   ```
9. Build the node
   ```
   cd
   cd catkin_ws/
   catkin_make
   source devel/setup.bash
   ```


