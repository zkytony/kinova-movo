#!/usr/bin/env python
# Object search within a region
#
# NOTE: This script should be run in a Python2 shell
# which has access to ROS packages. It expects launch
# files that will serve the rosparams to be already
# running.

import argparse
import rospy
import time
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import MarkerArray as ArMarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import message_filters
import tf
import subprocess
import os
import yaml
from action_type import ActionType
from waypoint_apply import WaypointApply
from head_and_torso import TorsoJTAS
from scipy.spatial.transform import Rotation as scipyR

# Start a separate process to run POMDP; Use the virtualenv
VENV_PYTHON = "/home/kaiyuzh/pyenv/py37/bin/python"
POMDP_SCRIPT = "/home/kaiyuzh/repo/3d-moos-pomdp/moos3d/robot_demo/build_pomdp.py"

def euclidean_dist(p1, p2):
    return math.sqrt(sum([(a - b)** 2 for a, b in zip(p1, p2)]))

def read_pose_msg(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    return (x,y,z,qx,qy,qz,qw)


def str_arg(self, arg):
    return "\"" + arg + "\""

def list_arg(self, l):
    return "\"" + " ".join(l) + "\""

def execute_action(action_info,
                   robot_state,
                   last_action_observation):
    """Exxecute the action described in action_info.
    navclient (SimpleActionClient) used to send goal
        to the navigation components."""
    obs_info = {"action_type": action_info["type"]}
    if action_info["type"] == ActionType.TopoMotionAction:
        # check source
        cur_robot_pose = wait_for_robot_pose()
        goal_tolerance = rospy.get_param('~goal_tolerance')
        if euclidean_dist(cur_robot_pose[:3], action_info["src_pose"]) > goal_tolerance:
            rospy.logerror("Robot is at %s, far away from expected location %s"\
                           (str(cur_robot_pose[:3]), action_info["src_pose"]))
        
        # navigate to destination
        position = action_info["dst_pose"]  # x,y,z
        # default rotation is (0,0,0) --> will look "forward"
        orientation = (0,0,0,1)  # equivalent to (0,0,0) in euler.
        if WaypointApply(position, orientation).status == WaypointApply.SUCCESS:
            # Successfully moved robot; Return an observation about the robot state.
            rospy.loginfo("Applied motion action successfully")
            obs_info["status"] = "success"
        else:
            rospy.logwarn("Failed to apply motion action")
            obs_info["status"] = "failed"
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_pose()  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
        obs_info["camera_direction"] = None  # consistent with transition model.

    elif action_info["type"] == ActionType.TorsoAction:
        torso_client = TorsoJTAS()
        # obtain current torso pose
        torso_height = wait_for_torso_height()
        if action_info['direction'].endswith("up"):
            desired_height = torso_height + action_info["displacement"]
        else:
            desired_height = torso_height - action_info["displacement"]
        # TODO: This is probably not right
        torso_client.add_point([desired_height], 0.0)
        torso_client.start()
        torso_client.wait(10.0)
        rospy.loginfo("Torso motion complete")

        # Get observation about robot state
        obs_info["status"] = "success"
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_pose()  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
        obs_info["camera_direction"] = None  # consistent with transition model.        

    elif action_info["type"] == ActionType.LookAction:
        rotation = action_info["rotation"]
        cur_robot_pose = wait_for_robot_pose()
        # Rotate the robot
        position = cur_robot_pose[:3]
        orientation = scipyR.from_euler("xyz", rotation, degrees=True).as_quat()
        if WaypointApply(position, orientation).status == WaypointApply.SUCCESS:
            # Successfully moved robot; Return an observation about the robot state.
            rospy.loginfo("Robot rotate successfully")
            obs_info["status"] = "success"
        else:
            rospy.logwarn("Failed to rotate robot")
            obs_info["status"] = "failed"

        # robot state            
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_pose()  # provides z pose.
        obs_info["objects_found"] = robot_state["objects_found"]
            
        if obs_info["status"] == "success":
            # Project field of view; Start the point cloud processing script,
            # one with ar tag detection, one without. Upon publishing the
            # volumetric observation, these scripts will save the observation
            # to a file.
            rospy.loginfo("Projecting field of view; Processing point cloud.")
            start_time = rospy.Time.now()
            voxels_dir = os.path.dirname(rospy.get_param('~observation_file'))
            vpath_ar = os.path.join(voxels_dir, "voxels_ar.yaml")
            vpath = os.path.join(voxels_dir, "voxels.yaml")
            start_pcl_process(save_path=vpath_ar,
                              detect_ar=True)
            start_pcl_process(save_path=vpath,
                              detect_ar=False)
            # wait until files are present
            wait_time = max(1, rospy.get_param('~observation_wait_time') - 3)
            observation_saved = False
            while rospy.Time.now() - start_time < rospy.Duration(wait_time):
                if os.path.exists(vpath):
                    observation_saved = True
                    break
                rospy.loginfo("Waiting for voxel observation file.")
                rospy.sleep(1.0)

            rospy.loginfo("Voxel observation saved.")
            if os.path.exists(vpath_ar):
                # use ar tag detection observation
                rospy.loginfo("Using the voxels that may contain AR tag labels.")
                with open(vpath_ar) as f:
                    voxels = yaml.safe_load(f)
            else:
                with open(vpath_ar) as f:
                    voxels = yaml.safe_load(f)
            obs_info["camera_direction"] = orientation  # consistent with transition model.
            obs_info["voxels"] = voxels # volumetric observation
        else:
            # robot didn't reach the desired rotation; no observation received
            obs_info["camera_direction"] = None  # consistent with transition model.
            obs_info["voxels"] = {}
            rospy.logerror("No observation received because desired rotation not reached.")


    elif action_info["type"] == ActionType.DetectAction:
        # Based on last observation, mark objects as detected
        last_action, last_observation = last_action_observation
        if last_action is not None and last_observation is not None:
            target_object_ids = set(rospy.get_param('~target_object_ids'))
            new_objects_found = set({})            
            if last_action["type"] == ActionType.LookAction:
                voxels = last_observation["voxels"]
                for voxel_pose in voxels:
                    _, label = voxels[voxel_pose]
                    if int(label) in target_object_ids:
                        new_objects_found.add(label)
                obs_info["objects_found"] = robot_state["objects_found"] | new_objects_found
        else:
            obs_info["objects_found"] = robot_state["objects_found"]
        # robot state
        obs_info["robot_pose"] = wait_for_robot_pose()
        obs_info["torso_height"] = wait_for_torso_pose()  # provides z pose.
        obs_info["camera_direction"] = obs_info["robot_pose"][3:]  # consistent with transition model.
    return obs_info
        
        
def start_pcl_process(save_path, detect_ar=False):
    search_space_dimension = rospy.get_param('~search_space_dimension')
    fov = rospy.get_param('~fov')
    asp = rospy.get_param('~aspect_ratio')
    near = rospy.get_param('~near')
    far = rospy.get_param('~far')
    sparsity = rospy.get_param("~sparsity")
    occupied_threshold = rospy.get_param("~occupied_threshold")
    mark_nearby = rospy.get_param("~mark_nearby")
    assert type(mark_nearby) == bool
    marker_topic = rospy.get_param("~marker_topic") #"/movo_pcl_processor/observation_markers"
    point_cloud_topic = rospy.get_param("~point_cloud_topic") #"/movo_pcl_processor/observation_markers"    
    
    optional_args = []
    if detect_ar:
        optional_args.append("-M")
        marker_topic += "_ar"
        if mark_nearby:
            optional_args.append("-N")
    
    subprocess.Popen(["rosrun", "movo_object_search", "process_pcl.py",
                      "--save-path", str_arg(save_path),
                      "--quit-when-saved",
                      "--point-cloud-topic", str_arg(point_cloud_topic),
                      "--marker-topic", str_arg(marker_topic),
                      "--resolution", str(search_space_resolution),
                      "--fov", camera_config['fov'],
                      "--asp", camera_config['asp'],
                      "--near", camera_config['near'],
                      "--far", camera_config['far'],
                      "--sparsity", str(sparsity),
                      "--occupied-threshold", str(occupied_threshold)]\
                     + optional_args)

def wait_for_robot_pose():
    robot_pose_topic = rospy.get_param('~robot_pose_topic')
    msg = rospy.wait_for_message(robot_pose_topic, PoseWithCovarianceStamped, timeout=15)
    robot_pose = read_pose_msg(msg)
    return robot_pose

def wait_for_torso_height():
    torso_topic = rospy.get_param('~torso_height_topic')  # /movo/linear_actuator/joint_states
    msg = rospy.wait_for_message(torso_topic, JointState, timeout=15)
    assert msg.name[0] == 'linear_joint', "Joint is not linear joint (not torso)."
    return msg.position[0]

def main():
    rospy.init_node("movo_object_search_in_region")

    region_origin_x = rospy.get_param('~region_origin_x')
    region_origin_y = rospy.get_param('~region_origin_y')

    search_space_dimension = rospy.get_param('~search_space_dimension')
    search_space_resolution = rospy.get_param('~search_space_resolution')
    target_object_ids = rospy.get_param('~target_object_ids')  # a list
    print("Total search space area: %.3f x %.3f x %.3f m^3"
          % (search_space_dimension * search_space_resolution))

    # files
    topo_map_file = rospy.get_param('~topo_map_file')
    action_file = rospy.get_param('~action_file')
    observation_file = rospy.get_param('~observation_file')
    prior_file = rospy.get_param('~prior_file')

    # other config
    observation_wait_time = rospy.get_param('~observation_wait_time')
    action_wait_time = rospy.get_param('~action_wait_time')    
    fov = rospy.get_param('~fov')
    asp = rospy.get_param('~aspect_ratio')
    near = rospy.get_param('~near')
    far = rospy.get_param('~far')
    torso_min = rospy.get_param('~torso_min')
    torso_max = rospy.get_param('~torso_max')

    # for volumetric observation
    sparsity = rospy.get_param("~sparsity")
    occupied_threshold = rospy.get_param("~occupied_threshold")
    mark_nearby = rospy.get_param("~mark_nearby")
    camera_config = {"fov": fov, "asp": asp,
                     "near": near, "far": far}

    # Listen to robot pose
    robot_pose = wait_for_robot_pose()

    subprocess.Popen([VENV_PYTHON, POMDP_SCRIPT,
                      # arguments
                      topo_map_file,
                      robot_pose,
                      search_space_dimension,
                      list_arg(target_object_ids),
                      list_arg([region_origin_x, region_origin_y]),
                      search_space_resolution,
                      action_file,
                      observation_file,
                      "--torso-min", torso_min,
                      "--torso-max", torso_max,
                      "--wait-time", observation_wait_time,
                      "--prior-file", prior_file,
                      "--fov", fov,
                      "--asp", asp,
                      "--near", near,
                      "--far", far])
    
    # Wait for an action and execute this action
    robot_state = {"objects_found": set({})}
    last_action_observation = (None, None)
    step = 0
    while True:
        rospy.loginfo("Waiting for action...(t=%d)" % step)
        start_time = rospy.Time.now()
        observation = None
        observation_issued = False
        while rospy.Time.now() - start_time < rospy.Duration(action_wait_time):
            if os.path.exists(action_file):
                rospy.loginfo("Got action! Executing action...")
                with open(action_file) as f:
                    action_info = load_action_from_file(action_file)

                # observation obtained from robot                    
                obs_info = execute_action(action_info,  
                                          robot_state,
                                          last_action_observation)
                robot_state["objects_found"] = obs_info["objects_found"]
                
                with open(observation_file, "w") as f:
                    yaml.dump(obs_infp, f)
                    
                rospy.loginfo("Action executed. Observation written to file %s" % observation_file)
                last_action_observation = (action_info, obs_info)
                observation_issued = True

                # remove action file
                os.remove(action_file)
                break  # break the loop                
            else:
                rospy.loginfo("Waiting for POMDP action...")
                rospy.sleep(0.5)
        if not observation_issued:
            rospy.logerror("Timed out waiting for POMDP action.")
            break
        if robot_state["objects_found"] == set(target_object_ids):
            rospy.loginfo("Done! All objects found in this region")
            break
        step += 1

if __name__ == "__main__":
    main()
