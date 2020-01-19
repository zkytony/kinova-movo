#!/usr/bin/env python

import os
from os.path import expanduser
from copy import copy
import rospy
import math
import random
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from action.action_set import ActionApply
from action.action_set import WaypointApply
from action.action_set import HeadJTAS
from action.action_set import TorsoJTAS
from world.world import WorldModel

home = expanduser("~")

def main():
    rospy.init_node('movo_object_search_main')

    num_regions = rospy.get_param('~num_regions')
    object_marker_id = rospy.get_param('~object_marker_id')
    marker_goal_pose_x = rospy.get_param('~marker_goal_pose_x')
    marker_goal_pose_y = rospy.get_param('~marker_goal_pose_y')

    world_x_res = rospy.get_param('~world_x_res')
    world_y_res = rospy.get_param('~world_y_res')
    world_z_res = rospy.get_param('~world_z_res')
    horizontal_cell_size = rospy.get_param('~horizontal_cell_size')
    vertical_cell_size = rospy.get_param('~vertical_cell_size')
    trans_vel = rospy.get_param('~trans_vel')
    ang_vel = rospy.get_param('~ang_vel')
    goal_pose_tolerance = rospy.get_param('~goal_pose_tolerance')
    goal_angle_tolerance = rospy.get_param('~goal_angle_tolerance')

    region_marker_id = []
    region_x_pose = []
    region_y_pose = []
    for i in range(0, num_regions):
        region_marker_id_str = "~region_%d/marker_id" % i
        region_x_pose_str = "~region_%d/x_pose" % i
        region_y_pose_str = "~region_%d/y_pose" % i
        region_marker_id.append(rospy.get_param(region_marker_id_str))
        region_x_pose.append(rospy.get_param(region_x_pose_str))
        region_y_pose.append(rospy.get_param(region_y_pose_str))

    if np.remainder(world_x_res, 2) == 1:
        odd_number = True
    else:
        odd_number = False

    r = rospy.Rate(10)
    for i in range(0, num_regions):
        # Robot traverses to the new region.
        wa = WaypointApply(trans_vel, ang_vel, goal_pose_tolerance, goal_angle_tolerance, region_marker_id[i], \
            region_x_pose[i], region_y_pose[i], marker_goal_pose_x, marker_goal_pose_y)
        while True: # Check if the robot odometry is obtained for the first time.
            if wa.check_cur_robot_pose():
                break
            r.sleep()
        wa.waypoint_execute()
        rospy.loginfo("Movo reached near the region.")

        while True: # Check if the robot's position is corrected by detecting the AR tag.
            if wa.check_marker_detect():
                break
            r.sleep()
        rospy.loginfo("Movo's position is corrected by the AR tag.")

        # Robot reaches the region.
        # wm = WorldModel(world_x_res, world_y_res, world_z_res, horizontal_cell_size, vertical_cell_size, odd_number)
        # while True:
        #     if wm.check_init_robot_pose():
        #         break
        #     r.sleep()
        # grid_x, grid_y, grid_z = wm.get_grid()

        # # Read the text file from the pomdp solver.

        # aa = ActionApply(grid_x, grid_y, grid_z, trans_vel, ang_vel, goal_pose_tolerance, goal_angle_tolerance, 1, 0, 0, -90)
        # while True:
        #     if aa.check_cur_robot_pose():
        #         break
        #     r.sleep()
        # aa.compute_action()

        # Write the text file to update observations for the pomdp solver.

    # Torso motion.
    # tmp_torso = rospy.wait_for_message("/movo/torso_controller/state", JointTrajectoryControllerState)
    # current_angles_torso = tmp_torso.desired.positions
    # print "current torso angle:", current_angles_torso
    # traj_torso = TorsoJTAS()
    # traj_torso.add_point(list(current_angles_torso), 0.0)
    
    # total_time_torso = 0.0
    # points_torso = [list(current_angles_torso), 0.0]
    # for i in range(0,1):
        
    #     pos = 0.3 # Maximum height: 0.4.
    #     vel = 0.05
        
    #     dt = abs(pos)/vel
    #     total_time_torso+=dt
   
    #     traj_torso.add_point([pos],total_time_torso)
        
    # traj_torso.start()

    # traj_torso.wait(total_time_torso+3.0)
    # print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()