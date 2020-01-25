import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class WaypointApply(object):
    class Status:
        NOT_RUNNING = "not_running"
        RUNNING = "running"
        SUCCESS = "success"
        FAIL = "fail"
    def __init__(self, position, orientation, action_name="navigate"):
        # Get an action client
        self.client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
        rospy.loginfo("Waiting for movo_move_base AS...")
        if not self.client.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to movo_move_base AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)

        self.status = WaypointApply.Status.NOT_RUNNING
        self.action_name = action_name

        # Define the goal
        rospy.loginfo("Waypoint (%.2f,%.2f) and (%.2f,%.2f,%.2f,%.2f) is sent.", position[0], position[1], orientation[0], \
            orientation[1], orientation[2], orientation[3])
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
        self.status = WaypointApply.Status.RUNNING
        self.client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)
        if self.client.wait_for_result():
            rospy.loginfo("Goal is reached at (%.2f, %.2f).", self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y)

    def active_cb(self):
        rospy.loginfo("Navigation action "+str(self.action_name)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.action_name)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.action_name)+" received")

    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Navigation action "+str(self.action_name)+" received a cancel request after it started executing, completed execution!")
            self.status = WaypointApply.Status.FAIL            
        elif status == 3:
            rospy.loginfo("Navigation action "+str(self.action_name)+" reached")
            self.status = WaypointApply.Status.SUCCESS
        elif status == 4:
            rospy.loginfo("Navigation action "+str(self.action_name)+" was aborted by the Action Server")
            rospy.signal_shutdown("Navigation action "+str(self.action_name)+" aborted, shutting down!")
            self.status = WaypointApply.Status.FAIL
        elif status == 5:
            rospy.loginfo("Navigation action "+str(self.action_name)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Navigation action "+str(self.action_name)+" rejected, shutting down!")
            self.status = WaypointApply.Status.FAIL
        elif status == 8:
            rospy.loginfo("Navigation action "+str(self.action_name)+" received a cancel request before it started executing, successfully cancelled!")
            self.status = WaypointApply.Status.FAIL
