#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import time
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from tf.transformations import quaternion_from_euler


class GoalsExecution(object):

    def __init__(self, goal_pos_list, initial_spwan_pos):
        """
        Class constructor

        :param goal_pos_list: contains list of goals
        :param initial_spwan_pos: contains the initial spawned position
        """
        # Initializing ros node named 'robot_move_in_goal_seq'
        rospy.init_node('my_multi_goal_send')

        # Assigning list of goals to instance's attribute
        self.goal_pos_list = goal_pos_list

        # Assigning initial spawn position to instance's attribute
        self.initial_pos = initial_spwan_pos

        # represents the current value of goal under execution
        self.current_goal_number = 0

        # represents number of goals send to robot
        self.total_goal_number = len(self.goal_pos_list)

        # time when the robot starts moving towards first goal
        self.start = 0

        # time when robot reaches last goals/ initial spawn position
        self.end = 0

        # Create Action Client of 'move_base' action
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Wait for action server
        wait = self.client.wait_for_server()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        # Call method to send first goal to turtlebot3
        self.move_base_client()

    def orientation_yaw_angle(self, yaw_angle_deg):
        """
         Takes Orientation specified in goal and
         returns its quaternion values
        """

        yaw_angle_rad = (math.pi / 180) * yaw_angle_deg
        quat = quaternion_from_euler(0.0, 0.0, yaw_angle_rad)
        return quat

    def goal_value_assign(self, goal_coord):
        """
        Creates MoveBaseGoal object and assigns values specified is goal
        to respective variable in created object
        and returns it
        """
        goal_pos = MoveBaseGoal()
        goal_pos.target_pose.header.frame_id = "map"
        goal_pos.target_pose.header.stamp = rospy.Time.now()
        goal_pos.target_pose.pose.position.x = goal_coord[0]
        goal_pos.target_pose.pose.position.y = goal_coord[1]
        goal_orientation = self.orientation_yaw_angle(goal_coord[2])
        goal_pos.target_pose.pose.orientation.x = goal_orientation[0]
        goal_pos.target_pose.pose.orientation.y = goal_orientation[1]
        goal_pos.target_pose.pose.orientation.z = goal_orientation[2]
        goal_pos.target_pose.pose.orientation.w = goal_orientation[3]
        return goal_pos

    def time_to_drive_route_calc(self):
        """ Calculates time required for driving the route"""
        return self.end - self.start

    def goal_cancelled_result_callback(self, status, result):
        """ This callback will executed if the goal is cancelled"""

        # status = 3 means SUCCEEDED
        if status == 3:
            # Save the time when turtlebot3 reaches initial spawned position
            self.end = time.time()
            rospy.loginfo("Initial spawned position reached!")
            print("Time required by robot to cover path {} secs".
                  format(self.time_to_drive_route_calc()))
            rospy.signal_shutdown("Initial goal pose reached")
            return None

    def done_cb(self, status, result):

        self.current_goal_number += 1
        # status = 2 means PREEMPTED that is goal is cancelled externally
        if status == 2:
            rospy.loginfo("Goal is cancelled Externally!")
            rospy.loginfo("Going to initial spawned position!")

            # calling goal_value_assign method
            goal_initial = self.goal_value_assign(self.initial_pos)

            # sending goal
            self.client.send_goal(goal_initial, self.goal_cancelled_result_callback)

        # status = 3 means SUCCEEDED means previously sent goal is reached
        if status == 3:
            rospy.loginfo("Goal "+str(self.current_goal_number)+" reached")

            # checking whether goals in route are remaining or not
            if self.current_goal_number < self.total_goal_number:

                # calling goal_value_assign method
                new_goal_pos = self.goal_value_assign(self.goal_pos_list[self.current_goal_number])

                # sending goal
                self.client.send_goal(new_goal_pos, self.done_cb)
            else:
                # Save the time when turtlebot3 reaches last goal of route
                self.end = time.time()
                rospy.loginfo("All listed goal positions reached!")
                print("Time required by robot to cover path {} secs".format(self.time_to_drive_route_calc()))
                rospy.signal_shutdown("Final goal pose reached!")
                return

    def move_base_client(self):
        """
        Sends the first goal of the route to robot.
        """
        goal_pos = self.goal_value_assign(self.goal_pos_list[self.current_goal_number])
        self.client.send_goal(goal_pos, self.done_cb)

        # Save the time when turtlebot3 starts moving from the first goal
        self.start = time.time()

        # node will run until it receives a shutdown signal
        rospy.spin()


if __name__ == '__main__':
    try:
        # goal to be send
        # goal coordinates and orientation are specified in "map" coordinate system
        # first value in list represents x coord
        # second value in list represents y coord
        # third value in list represents robot's orientation in degrees in Z direction
        first_goal = [0.8, -1.75, 45]
        second_goal = [-0.5, -0.5, 30]
        third_goal = [2.0, 0.75, -100]
        forth_goal = [-0.9, 1.9, 75]

        # All goals are combined in one list
        # It can also be written as
        # goal_list = [[1.6, -0.8, 45],[1.6, -0.8, 45],[1.6, -0.8, 45],[1.6, -0.8, 45]]
        goal_list = [first_goal,
                     second_goal,
                     third_goal,
                     forth_goal]

        # Initial spawn position of the  robot when turtlebot3_world.launch file is launched
        initial_spawn_pos = [-2.0, -0.5, 0.0]

        # Class created for performing the tasks specified in Task
        GoalsExecution(goal_list, initial_spawn_pos)

    except rospy.ROSInterruptException:
        rospy.loginfo("Test Finished!!!")
