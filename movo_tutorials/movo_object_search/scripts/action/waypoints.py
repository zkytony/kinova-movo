#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WaypointApply(object):
    def __init__(self, position, orientation):
        # Get an action client
        self.client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        # print "type(position[0]):", type(position[0])
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        self.waypoint_execute()

    def waypoint_execute(self):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()