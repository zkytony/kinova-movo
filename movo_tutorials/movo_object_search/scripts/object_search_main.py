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
from action.action_set import HeadJTAS
from action.action_set import TorsoJTAS
from world.world import WorldModel

home = expanduser("~")

def main():
    rospy.init_node('movo_object_search_main')
    num_regions = rospy.get_param('~num_regions')
    world_x_res = rospy.get_param('~world_x_res')
    world_y_res = rospy.get_param('~world_y_res')
    world_z_res = rospy.get_param('~world_z_res')
    horizontal_cell_size = rospy.get_param('~horizontal_cell_size')
    vertical_cell_size = rospy.get_param('~vertical_cell_size')
    trans_vel = rospy.get_param('~trans_vel')
    ang_vel = rospy.get_param('~ang_vel')
    goal_pose_tolerance = rospy.get_param('~goal_pose_tolerance')
    goal_angle_tolerance = rospy.get_param('~goal_angle_tolerance')
    if np.remainder(world_x_res,2) == 1:
        odd_number = True
    else:
        odd_number = False

    for i in range(0,num_regions):
        wm = WorldModel(world_x_res, world_y_res, world_z_res, horizontal_cell_size, vertical_cell_size, odd_number)
        r = rospy.Rate(10)
        while True:
            if wm.get_init_robot_pose_check():
                break
            r.sleep()
        grid_x, grid_y, grid_z = wm.get_grid()

        # Read the text file from the pomdp solver.

        aa = ActionApply(grid_x, grid_y, grid_z, trans_vel, ang_vel, goal_pose_tolerance, goal_angle_tolerance, 1, 0, 0, -90)
        while True:
            if aa.get_cur_robot_pose_check():
                break
            r.sleep()
        aa.compute_action()

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