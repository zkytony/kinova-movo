#!/usr/bin/env python

import argparse
import sys
from copy import copy
import rospy
import actionlib
import math
import random
import numpy as np
from movo.system_defines import TRACTOR_REQUEST
from geometry_msgs.msg import Twist
from movo_msgs.msg import ConfigCmd
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from sensor_msgs.msg import JointState

from control_msgs.msg import JointTrajectoryControllerState

class HeadJTAS(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['pan_joint','tilt_joint']

class TorsoJTAS(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['linear_joint']

class HeadJTASTest(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['pan_joint','tilt_joint']

class BaseMotionTest(object):
    def __init__(self):
        self._base_vel_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)

        # set robot mode to active base motion
        self._cfg_cmd = ConfigCmd()
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

        # for motion sequence command request
        self.dist_cmdList = []
        self.vel_cmdList = []
        self.dist_tolerance = 0.001
        self.rot_tolerance = math.radians(1.0)
        self.dist_vel_tolerance = 0.001
        self.rot_vel_tolerance = math.radians(1.0)
        self.duration_limit = 3600  # a motion dont plan for more than 1hour

    def motion_dist(self, dist_cmd, vel_cmd=None):
        """
        Command the base to move certain distance
        @param dist_cmd: distance along x, y in meter, rotation along z in degree, distance is absolute value.(Positive)
        @param vel_cmd: velocity during the motion, unit in dist_cmd per second.
        @return:
        """

        # default velocity, velocity is the abosulute value. Negative should be in distance command.
        if vel_cmd is None:
            vel_cmd = [0.1, 0.1, 30]

        dist_cmd = map(math.fabs, map(float, dist_cmd))
        vel_cmd = map(float, vel_cmd)

        # duration is as the action takes most time consumption.
        duration_temp = [0.0, 0.0, 0.0]
        for i in range(3):
            # validation of command input
            if i < 2:  # translation
                if dist_cmd[i] != 0.0 and dist_cmd[i] < self.dist_tolerance:
                    rospy.logwarn("distance command " + str(dist_cmd[i]) + " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0
                elif vel_cmd[i] != 0.0 and math.fabs(vel_cmd[i]) < self.dist_vel_tolerance:
                    rospy.logwarn("translation velocity command " + str(vel_cmd[i]) + 
                                  " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0

            elif i == 2:  # rotation
                if dist_cmd[i] != 0.0 and dist_cmd[i] < self.rot_tolerance:
                    rospy.logwarn("rotation command " + str(dist_cmd[i]) + " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0
                elif vel_cmd[i] != 0.0 and math.fabs(vel_cmd[i]) < self.rot_vel_tolerance:
                    rospy.logwarn("rotation velocity command " + str(vel_cmd[i]) + 
                                  " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0

            if vel_cmd[i] == 0.0:
                duration_temp[i] = 0.0
            else:
                duration_temp[i] = math.fabs(dist_cmd[i] / vel_cmd[i])

        duration = max(duration_temp)

        # revise vel_cmd so that all motion finish at same duration.
        if duration == 0.0:
            rospy.logwarn("duration is zero")
            return
        elif duration > self.duration_limit:
            rospy.logwarn("motion duration exceeded " + str(self.duration_limit) + " seconds, execution cancelled")
        else:
            self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self._cfg_cmd.gp_param = TRACTOR_REQUEST
            self._cfg_cmd.header.stamp = rospy.get_rostime()
            self._cfg_pub.publish(self._cfg_cmd)
            rospy.sleep(0.1)

            twist_cmd = Twist()
            twist_cmd.linear.x = vel_cmd[0]
            twist_cmd.linear.y = vel_cmd[1]
            twist_cmd.linear.z = 0.0
            twist_cmd.angular.x = 0.0
            twist_cmd.angular.y = 0.0
            twist_cmd.angular.z = math.radians(vel_cmd[2])

            rospy.logdebug("Send velocity command to movo base from BaseVelTest class ...")
            rate = rospy.Rate(100)
            start_time = rospy.get_time()
            while ((rospy.get_time() - start_time) < duration) and not (rospy.is_shutdown()):
                self._base_vel_pub.publish(twist_cmd)
                rate.sleep()

    def move_forward(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([math.fabs(meters), 0.0, 0.0], [math.fabs(speed), 0.0, 0.0])

    def move_backward(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([math.fabs(meters), 0.0, 0.0], [-1.0*math.fabs(speed), 0.0, 0.0])

    def move_left(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([0.0, math.fabs(meters), 0.0], [0.0, math.fabs(speed), 0.0])

    def move_right(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([0.0, math.fabs(meters), 0.0], [0.0, -1.0*math.fabs(speed), 0.0])

    def rotate_clock(self, degrees, speed=None):
        if speed is None:
            speed = 30
        self.motion_dist([0.0, 0.0, math.fabs(degrees)], [0.0, 0.0, math.fabs(speed)])

    def rotate_anticlock(self, degrees, speed=None):
        if speed is None:
            speed = 30
        self.motion_dist([0.0, 0.0, math.fabs(degrees)], [0.0, 0.0, -1.0*math.fabs(speed)])

    def add_motion_to_list(self, dist_cmd, vel_cmd=None):
        self.dist_cmdList.append(dist_cmd)
        self.vel_cmdList.append(vel_cmd)

    def clear_motion_list(self):
        self.dist_cmdList = []
        self.vel_cmdList = []

    def move_sequence(self):
        for i in range(0, len(self.dist_cmdList)):
            self.motion_dist(self.dist_cmdList[i], self.vel_cmdList[i])

    def motion_stop(self, duration=1.0):
        self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        self._cfg_cmd.gp_param = 0
        self._cfg_cmd.header.stamp = rospy.get_rostime()
        self._cfg_pub.publish(self._cfg_cmd)

        rospy.logdebug("Stopping velocity command to movo base from BaseVelTest class ...")
        try:
            r = rospy.Rate(10)
            start_time = rospy.get_time()
            while (rospy.get_time() - start_time) < duration:
                self._base_vel_pub.publish(Twist())
                r.sleep()
        except Exception as ex:
            print "Message of base motion failed to be published, error message: ", ex.message
            pass

class TorsoJTASTest(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['linear_joint']

class WorldModel(object):
    def __init__(self, world_x_res, world_y_res, world_z_res, horizontal_cell_size, vertical_cell_size, odd_number):
        self._grid_x = []
        self._grid_y = []
        self._grid_z = []
        self._world_x_res = world_x_res
        self._world_y_res = world_y_res
        self._world_z_res = world_z_res
        self._horizontal_cell_size = horizontal_cell_size
        self._vertical_cell_size = vertical_cell_size
        self._odd_number = odd_number
        self._init_robot_pose_sub = rospy.Subscriber('/movo/odometry/local_filtered', Odometry, self.init_robot_pose_cb)
        self._init_robot_pose_check = False

    def init_robot_pose_cb(self, data):
        if not self._init_robot_pose_check:
            thx, rthy, rthz = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
            init_robot_pose = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, rthz)
            print "Initial robot position: ", init_robot_pose

            # Origin of the world.
            if self._odd_number:
                x_st = init_robot_pose[0]-(math.floor(self._world_x_res/2)*self._horizontal_cell_size)
            else:
                x_st = init_robot_pose[0]-((self._world_x_res/2-1)*self._horizontal_cell_size)
            y_st = init_robot_pose[1]
            z_st = 0

            # Positions for all grid cells.
            for i in range(0, self._world_x_res):
                self._grid_x.append(x_st+self._horizontal_cell_size*i)
            for i in range(0, self._world_y_res):
                self._grid_y.append(y_st+self._horizontal_cell_size*i)
            for i in range(0, self._world_z_res):
                self._grid_z.append(z_st+self._vertical_cell_size*i)

            self._init_robot_pose_check = True

    def get_grid(self):
        return self._grid_x, self._grid_y, self._grid_z

    def get_init_robot_pose_check(self):
        return self._init_robot_pose_check

class ActionApply(object):
    def __init__(self, grid_x, grid_y, grid_z, trans_vel, ang_vel, goal_pose_tolerance, goal_angle_tolerance, grid_x_val, grid_y_val, grid_z_val, heading_angle):
        self._trans_vel = trans_vel
        self._ang_vel = ang_vel
        self._goal_pose_tolerance = goal_pose_tolerance
        self._goal_angle_tolerance = goal_angle_tolerance
        self._cur_robot_pose_sub = rospy.Subscriber('/movo/odometry/local_filtered', Odometry, self.cur_robot_pose_cb)
        self._base_vel_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)

        # Set robot mode to active base motion
        self._cfg_cmd = ConfigCmd()
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

        self._goal_pose_x = grid_x[grid_x_val]
        self._goal_pose_y = grid_y[grid_y_val]
        self._goal_pose_z = grid_z[grid_z_val]
        self._goal_heading_angle = heading_angle

        self._cur_robot_pose_check = False

    def cur_robot_pose_cb(self, data):
        self._cur_robot_pose_x = data.pose.pose.position.x
        self._cur_robot_pose_y = data.pose.pose.position.y
        self._cur_robot_pose_z = data.pose.pose.position.z
        cur_euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self._cur_robot_heading_angle = math.degrees(cur_euler[2])
        self._cur_robot_pose_check = True

    def compute_action(self):
        if self._cur_robot_pose_check:
            rospy.loginfo("Goal pose is (%.2f, %.2f, %.2f, %.2f)", self._goal_pose_x, self._goal_pose_y, self._goal_pose_z, self._goal_heading_angle)
            diff_x = self._goal_pose_x-self._cur_robot_pose_x
            diff_y = self._goal_pose_y-self._cur_robot_pose_y

            if diff_x >= 0:
                vel_cmd_x = self._trans_vel
            else:
                vel_cmd_x = -1*self._trans_vel
            if diff_y >= 0:
                vel_cmd_y = self._trans_vel
            else:
                vel_cmd_y = -1*self._trans_vel

            self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self._cfg_cmd.gp_param = TRACTOR_REQUEST
            self._cfg_cmd.header.stamp = rospy.get_rostime()
            self._cfg_pub.publish(self._cfg_cmd)
            rospy.sleep(0.1)

            twist_cmd = Twist()
            twist_cmd.linear.x = vel_cmd_x
            twist_cmd.linear.y = 0.0
            twist_cmd.linear.z = 0.0
            twist_cmd.angular.x = 0.0
            twist_cmd.angular.y = 0.0
            twist_cmd.angular.z = 0.0

            rate = rospy.Rate(100)
            rospy.loginfo("Robot starts moving along x axis.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_pose_x-self._goal_pose_x) < self._goal_pose_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

            twist_cmd.linear.x = 0.0
            twist_cmd.linear.y = vel_cmd_y

            rospy.loginfo("Robot starts moving along y axis.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_pose_y-self._goal_pose_y) < self._goal_pose_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

            twist_cmd.linear.y = 0.0
            if self._goal_heading_angle >= 0:
                twist_cmd.angular.z = math.radians(self._ang_vel)
            else:
                twist_cmd.angular.z = -1*math.radians(self._ang_vel)

            rospy.loginfo("Robot starts rotating.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_heading_angle-self._goal_heading_angle) < self._goal_angle_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

    def motion_stop(self, duration=1.0):
        self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        self._cfg_cmd.gp_param = 0
        self._cfg_cmd.header.stamp = rospy.get_rostime()
        self._cfg_pub.publish(self._cfg_cmd)

        rospy.logdebug("Stopping velocity command to movo base from BaseVelTest class ...")
        try:
            r = rospy.Rate(10)
            start_time = rospy.get_time()
            while (rospy.get_time() - start_time) < duration:
                self._base_vel_pub.publish(Twist())
                r.sleep()
        except Exception as ex:
            print "Message of base motion failed to be published, error message: ", ex.message
            pass

    def get_cur_robot_pose_check(self):
        return self._cur_robot_pose_check