#!/usr/bin/env python

import argparse
import sys
from copy import copy
import rospy
import actionlib
import math
import random
from action.head_and_torso import HeadJTAS, TorsoJTAS
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

def to_rad(deg):
    return math.pi * deg / 180.0

def to_deg(rad):
    return 180.0 * rad / math.pi

def wait_for_torso_height(torso_topic="/movo/torso_controller/state"):
    if torso_topic=="/movo/torso_controller/state":
        msg = rospy.wait_for_message(torso_topic, JointTrajectoryControllerState, timeout=15)
        assert msg.joint_names[0] == 'linear_joint', "Joint is not linear joint (not torso)."
        position = msg.actual.positions[0]
    else:
        assert torso_topic == "/movo/linear_actuator/joint_states"
        msg = rospy.wait_for_message(torso_topic, JointState, timeout=15)
        assert msg.name[0] == 'linear_joint', "Joint is not linear joint (not torso)."        
        position = msg.position[0]
    return position

def wait_for_head():
    head_topic="/movo/head_controller/state"
    msg = rospy.wait_for_message(head_topic, JointTrajectoryControllerState, timeout=15)
    assert msg.joint_names[0] == 'pan_joint', "Joint is not head joints (need pan or tilt)."
    cur_pan = msg.actual.positions[0]
    cur_tilt = msg.actual.positions[1]
    return cur_pan, cur_tilt

def main():
    rospy.init_node('movo_search_object_init')
    print("---head---")
    print(tuple(map(to_deg, wait_for_head())))
    HeadJTAS.move(to_rad(0), to_rad(-15))
    print(tuple(map(to_deg, wait_for_head())))
    
    print("---torso---")
    rostopics = rospy.get_published_topics()
    torso_topic = "/movo/torso_controller/state"
    if torso_topic not in rostopics:
        torso_topic =  "/movo/linear_actuator/joint_states"
    print(wait_for_torso_height(torso_topic=torso_topic))
    TorsoJTAS.move(0.6, torso_topic=torso_topic)
    print(wait_for_torso_height(torso_topic=torso_topic))

if __name__ == "__main__":
    main()
