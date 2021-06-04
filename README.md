# turtlebot3_multiple_goal_navigatiion
This repository is created to send the list of desired goals to robot in robots "map" co-ordinate system using navigation stack of ROS.

# About the node
The "my_multi_goal_send" node send the list of goals specified in the robots "map" coordinate system. You can specify X, Y and orientation angle of robot along its Z axis. The robot will go all the goals listed in the list and in the end prints the time required by the robot for completing the robot. At any instance, one can cancel the goal and in that case the robot will go back to its initial spawned position and print the time requird to drive the route. 

# How to install the node

1. For this node ROS Melodic Version is used and progamming langauge python is used
2. In order to run the node, you should have turtlebot3_gazebo turtlebot3_simulation turtlebot3_navigation turtlebot3_teleop turtlebot3_slam and turtlebot3_msgs packages install.
   ```
   sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-teleop
   ```

