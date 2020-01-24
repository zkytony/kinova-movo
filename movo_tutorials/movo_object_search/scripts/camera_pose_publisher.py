# Simply publishes camera pose transforms
import rospy
import tf

def main():
    tf_listener = tf.TransformListener()

    source_frame = "map"
    target_frame = "movo_camera_link"

    if self._tf_listener.frameExists(source_frame)\
       and self._tf_listener.frameExists(target_frame):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = source_frame
        pose_stamped_msg.pose = pose_msg
        pose_stamped_transformed = self._tf_listener.transformPose(target_frame, pose_stamped_msg)
        return pose_stamped_transformed.pose
