### Listens to point cloud and process it
import argparse
import rospy
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import MarkerArray as ArMarkerArray
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
import tf
import util

import time
from camera_model import FrustumCamera

# THE VOXEL TYPE; If > 0, it's an object id.
VOXEL_OCCUPIED = -1
VOXEL_FREE = -2
VOXEL_UNKNOWN = -3

# A very good ROS Ask question about the point cloud message
# https://answers.ros.org/question/273182/trying-to-understand-pointcloud2-msg/
class PCLProcessor:
    """
    Subscribes to point cloud and ar tag topics, and publish volumetric
    observation as a result of processing the messages.

    The reason to use Subscriber inteader of Client/Service is because
    point cloud data and ar tag markers themselves are published via topics.
    """
    def __init__(self,
                 # frustum camera configuration
                 fov=90,
                 aspect_ratio=1,
                 near=1,
                 far=5,
                 resolution=0.5,  # m/grid cell
                 pcl_topic="/movo_camera/point_cloud/points",
                 marker_topic="/movo_pcl_processor/observation_markers",
                 artag_topic="/aruco_marker_publisher/markers",
                 voxel_pose_frame="movo_camera_color_optical_frame",
                 sparsity=1000,
                 occupied_threshold=5,
                 real_robot=False,
                 mark_nearby=False,  # mark voxels within 1 distance of the artag voxel as well.
                 mark_ar_tag=True):  # true if use message filter to process ar tag and point cloud messages together
        self._real_robot = real_robot
        self._resolution = resolution
        self._sparsity = sparsity  # number of points to skip
        self._occupied_threshold = occupied_threshold
        self._voxel_pose_frame = voxel_pose_frame
        self._mark_nearby = mark_nearby
        self._mark_ar_tag = mark_ar_tag
        self._cam = FrustumCamera(fov=fov, aspect_ratio=aspect_ratio,
                                  near=near, far=far)
        # Listen to tf
        self._tf_listener = tf.TransformListener()
        
        # The AR tag message and point cloud message are synchronized using
        # message filter. The point cloud message also has its own callback,
        # in case there is no AR tag detected.
        self._sub_pcl = message_filters.Subscriber(pcl_topic, PointCloud2)
        if self._mark_ar_tag:
            self._sub_artag = message_filters.Subscriber(artag_topic, ArMarkerArray)
            self._pcl_artag_ats = message_filters.ApproximateTimeSynchronizer([self._sub_pcl, self._sub_artag],
                                                                              5,1)#queue_size=10, slop=1.0)
            self._pcl_artag_ats.registerCallback(self._pcl_artag_cb)            
        else:
            self._sub_pcl.registerCallback(self._pcl_cb)        
        self._processing_point_cloud = False
        
        # Publish processed point cloud
        self._pub_pcl = rospy.Publisher(marker_topic,
                                        MarkerArray,
                                        queue_size=10,
                                        latch=True)

    def _pcl_cb(self, msg):
        # We just process one point cloud message at a time.
        if self._processing_point_cloud:
            return
        else:
            self._processing_point_cloud = True
            voxels = self.process_cloud(msg)
            msg = self.make_markers_msg(voxels)
            # publish message
            r = rospy.Rate(3) # 3 Hz
            self._pub_pcl.publish(msg)
            print("Published markers")
            self._processing_point_cloud = False
            r.sleep()

    def _pcl_artag_cb(self, pcl_msg, artag_msg):
        """Called when received an artag message and a point cloud."""
        if self._processing_point_cloud:
            return
        else:
            self._processing_point_cloud = True
            voxels = self.process_cloud(pcl_msg)

            # Mark voxel at artag location as object
            for artag in artag_msg.markers:
                # Transform pose to voxel_pose_frame
                artag_pose = self._get_transform(self._voxel_pose_frame, artag.header.frame_id, artag.pose.pose)
                if artag_pose is False:
                    return # no transformed pose obtainable
                atx = artag_pose.position.x
                aty = artag_pose.position.y
                atz = artag_pose.position.z
                arvoxel_pose = (int(round(atx / self._resolution)),
                                int(round(aty / self._resolution)),
                                int(round(atz / self._resolution)))
                # (Approach1) Find the voxel_pose in the volume closest to above
                if not self._mark_nearby:
                    closest_voxel_pose = min(voxels, key=lambda voxel_pose: util.euclidean_dist(voxel_pose, arvoxel_pose))
                    voxels[closest_voxel_pose] = (closest_voxel_pose, artag.id)
                else:
                    # (Approach2) Mark all voxel_poses in the volume within a certain dist.
                    nearby_voxel_poses = {voxel_pose
                                          for voxel_pose in voxels
                                          if util.euclidean_dist(voxel_pose, arvoxel_pose) <= 1}
                    for voxel_pose in nearby_voxel_poses:
                        voxels[voxel_pose] = (voxel_pose, artag.id)
                
            msg = self.make_markers_msg(voxels)
            
            # publish message
            r = rospy.Rate(3) # 3 Hz
            self._pub_pcl.publish(msg)
            print("Published markers")
            self._processing_point_cloud = False
            r.sleep()
    

    def point_in_volume(self, voxel, point):
        """Check if point (in point cloud) is inside the volume covered by voxel"""
        vx,vy,vz = voxel[:3]
        xmin = vx*self._resolution
        xmax = (vx+1)*self._resolution
        ymin = vy*self._resolution
        ymax = (vy+1)*self._resolution
        zmin = vz*self._resolution
        zmax = (vz+1)*self._resolution
        px, py, pz = point[:3]
        # print("%s | %s" % (voxel, point))
        if xmin <= px and px < xmax\
           and ymin <= py and py < ymax\
           and zmin <= pz and pz < zmax:
            return True
        else:
            return False
        
    def _get_transform(self, target_frame, source_frame, pose_msg):
        # http://wiki.ros.org/tf/TfUsingPython
        if self._tf_listener.frameExists(source_frame)\
           and self._tf_listener.frameExists(target_frame):
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = source_frame
            pose_stamped_msg.pose = pose_msg
            pose_stamped_transformed = self._tf_listener.transformPose(target_frame, pose_stamped_msg)
            return pose_stamped_transformed.pose
        else:
            rospy.logwarn("Frame %s or %s does not exist. (Check forward slash?)" % (target_frame, source_frame))
            return False

    def process_cloud(self, msg):
        # Iterate over the voxels in the FOV
        points = []
        for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            points.append(point)
        voxels = {}  # map from voxel_pose xyz to label
        oalt = {}
        pesp_to_plel = {}  # map from xyz in perspective to xy key in parallel
        for xyz in self._cam.volume:
            # Iterate over the whole point cloud sparsely
            i = 0
            count = 0
            occupied = False
            # In the camera model, robot looks at -z direction.
            # But robot actually looks at +z in the real camera.
            original_z = xyz[2]
            xyz[2] = abs(xyz[2])
            for point in points:
                if i % self._sparsity == 0:
                    if self.point_in_volume(xyz, point):
                        count += 1
                        if count > self._occupied_threshold:
                            occupied = True
                            break
                i += 1

            # forget about the homogenous coordinate; use xyz as key                
            xyz = tuple(xyz[:3])
            if occupied:
                xyz2 = (xyz[0], xyz[1], xyz[2]+1)  # TODO: HACK
                voxels[xyz] = (xyz, VOXEL_OCCUPIED)
                voxels[xyz2] = (xyz2, VOXEL_UNKNOWN)                
            else:
                voxels[xyz] = (xyz, VOXEL_FREE)
        # TODO: Actually do frustum filter
        return voxels

    def _make_pose_msg(self, posit, orien):
        pose = Pose()
        pose.position.x = posit[0] * self._resolution
        pose.position.y = posit[1] * self._resolution
        pose.position.z = posit[2] * self._resolution
        pose.orientation.x = orien[0]
        pose.orientation.y = orien[1]
        pose.orientation.z = orien[2]
        pose.orientation.w = orien[3]
        return pose

    def make_markers_msg(self, voxels):
        """Convert voxels to Markers message for visualizatoin"""
        timestamp = rospy.Time.now()
        i = 0
        markers = []
        for voxel_pose in voxels:
            xyz, label = voxels[voxel_pose]
            
            h = Header()
            h.stamp = timestamp
            h.frame_id = self._voxel_pose_frame
            
            marker_msg = Marker()
            marker_msg.header = h
            marker_msg.type = 1  # CUBE
            marker_msg.ns = "volumetric_observation"
            marker_msg.id = i; i+=1
            marker_msg.action = 0 # add an object
            marker_msg.pose = self._make_pose_msg(xyz, [0,0,0,1])
            marker_msg.scale.x = self._resolution
            marker_msg.scale.y = self._resolution
            marker_msg.scale.z = self._resolution
            if label == VOXEL_OCCUPIED:  # red
                marker_msg.color.r = 0.8
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
                marker_msg.color.a = 0.7
            elif label == VOXEL_FREE:  # cyan
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.8
                marker_msg.color.a = 0.1
            elif label == VOXEL_UNKNOWN:  # grey
                marker_msg.color.r = 0.8
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.8
                marker_msg.color.a = 0.7
            elif label >= 0:  # it's an object. Mark as Green
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.8
                marker_msg.color.b = 0.0
                marker_msg.color.a = 1.0
            else:
                raise ValueError("Unknown voxel label %s" % str(label))
            marker_msg.lifetime = rospy.Duration.from_sec(3.0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)

        marker_array_msg = MarkerArray(markers)
        return marker_array_msg


def main():
    parser = argparse.ArgumentParser(description='Process Point Cloud as Volumetric Observation')
    parser.add_argument('-p', '--point-cloud-topic', type=str,
                        default="/movo_camera/sd/points")
    parser.add_argument('-m', '--marker-topic', type=str,
                        default="/movo_pcl_processor/observation_markers")
    parser.add_argument('-R', '--real-robot', action="store_true")
    parser.add_argument('-M', '--mark-ar-tag', action="store_true")    
    args = parser.parse_args()
    
    rospy.init_node("movo_pcl_processor",
                    anonymous=True, disable_signals=True)

    # Some info about the Kinect
    # The Kinect has a range of around . 5m and 4.5m (1'8"-14'6".)
    #    (ref: https://docs.depthkit.tv/docs/kinect-for-windows-v2)
    # The old Kinect has a color image resolution of 640 x 480 pixels with a fov of
    # 62 x 48.6 degrees resulting in an average of about 10 x 10 pixels per degree. (see source 1)
    # The new Kinect has color image resolution of 1920 x 1080 pixels and a fov
    # of 84.1 x 53.8 resulting in an average of about 22 x 20 pixels per degree. (see source 2)
    proc = PCLProcessor(fov=60, aspect_ratio=1.0,
                        near=1.0, far=7, resolution=0.3,
                        sparsity=500, occupied_threshold=3,
                        pcl_topic=args.point_cloud_topic,
                        marker_topic=args.marker_topic,
                        real_robot=args.real_robot,
                        mark_ar_tag=args.mark_ar_tag)  # this covers a range from about 0.32m - 4m
    rospy.spin()

if __name__ == "__main__":
    main()
