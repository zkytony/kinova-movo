import rospy
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
import json

def make_pose_msg(posit, orien):
    pose = Pose()
    pose.position.x = posit[0]
    pose.position.y = posit[1]
    pose.position.z = posit[2]
    pose.orientation.x = orien[0]
    pose.orientation.y = orien[1]
    pose.orientation.z = orien[2]
    pose.orientation.w = orien[3]
    return pose            

class PublishSearchRegionMarkers:
    def __init__(self,
                 region_origin,
                 search_region_dimension,
                 search_region_resolution,
                 marker_frame="map",
                 marker_topic="/movo_object_search_in_region/region_markers"):
        self._marker_frame = marker_frame
        self._region_size = search_region_dimension * search_region_resolution
        self._region_origin = region_origin
        
        markers_msg = self.make_markers_msg()
        self._pub = rospy.Publisher(marker_topic,
                                    MarkerArray,
                                    queue_size=10,
                                    latch=True)
        self._pub.publish(markers_msg)        
        
    def make_markers_msg(self):
        """Convert voxels to Markers message for visualizatoin"""
        timestamp = rospy.Time.now()
        i = 0
        markers = []
        
        region_center = (self._region_origin[0] + self._region_size / 2.0,
                         self._region_origin[1] + self._region_size / 2.0)
        
        # rectangle
        h = Header()
        h.stamp = timestamp
        h.frame_id = self._marker_frame
        marker_msg = Marker()
        marker_msg.header = h
        marker_msg.type = 1  # cube
        marker_msg.ns = "search_region"
        marker_msg.id = i; i+=1
        marker_msg.action = 0 # add an object
        marker_msg.pose = make_pose_msg((region_center[0], region_center[1], 0.0),
                                        [0,0,0,1])
        marker_msg.scale.x = self._region_size
        marker_msg.scale.y = self._region_size
        marker_msg.scale.z = 0.02
        marker_msg.color.r = 0.2
        marker_msg.color.g = 0.7
        marker_msg.color.b = 0.7
        marker_msg.color.a = 0.4            
        marker_msg.lifetime = rospy.Duration(0)  # forever
        marker_msg.frame_locked = True
        markers.append(marker_msg)
        marker_array_msg = MarkerArray(markers)
        return marker_array_msg
    

class PublishTopoMarkers:
    """Debug markers include: topological graph; search region."""
    def __init__(self,
                 map_file,
                 resolution,
                 marker_frame="map",
                 marker_topic="/movo_object_search_in_region/topo_markers"):
        self._marker_frame = marker_frame
        self._nodes = {}
        self._edges = {}
        self._resolution = resolution
        with open(map_file) as f:
            data = json.load(f)

        for node_id in data["nodes"]:
            node_data = data["nodes"][node_id]
            x, y = node_data["x"], node_data["y"]
            self._nodes[int(node_id)]= (x,y,0.0)

        for i, edge in enumerate(data["edges"]):
            node_id1, node_id2 = edge[0], edge[1]
            self._edges[i] = (node_id1, node_id2)

        markers_msg = self.make_markers_msg()
        self._pub = rospy.Publisher(marker_topic,
                                    MarkerArray,
                                    queue_size=10,
                                    latch=True)
        self._pub.publish(markers_msg)

    @property
    def nodes(self):
        return self._nodes

    @property
    def edges(self):
        return self._edges

    def make_markers_msg(self):
        """Convert voxels to Markers message for visualizatoin"""
        timestamp = rospy.Time.now()
        i = 0
        markers = []
        # nodes
        for nid in self._nodes:
            xyz = self._nodes[nid]
            h = Header()
            h.stamp = timestamp
            h.frame_id = self._marker_frame

            # node cylinder
            marker_msg = Marker()
            marker_msg.header = h
            marker_msg.type = 3  # Cylinder
            marker_msg.ns = "topo_map_node"
            marker_msg.id = i; i+=1
            marker_msg.action = 0 # add an object
            marker_msg.pose = make_pose_msg(xyz, [0,0,0,1])
            marker_msg.scale.x = self._resolution
            marker_msg.scale.y = self._resolution
            marker_msg.scale.z = 0.05
            marker_msg.color.r = 0.8
            marker_msg.color.g = 0.5
            marker_msg.color.b = 0.5
            marker_msg.color.a = 0.7            
            marker_msg.lifetime = rospy.Duration(0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)

            # node id text
            marker_msg = Marker()
            marker_msg.header = h
            marker_msg.type = 9  # TEXT_VIEW_FACING
            marker_msg.ns = "topo_map_node"
            marker_msg.id = i; i+=1
            marker_msg.action = 0 # add an object
            marker_msg.pose = make_pose_msg(xyz, [0,0,0,1])
            marker_msg.scale.x = self._resolution * 4
            marker_msg.scale.y = self._resolution * 4
            marker_msg.scale.z = 0.05
            marker_msg.color.r = 0.9
            marker_msg.color.g = 0.9
            marker_msg.color.b = 0.9
            marker_msg.color.a = 0.9
            marker_msg.text = str(nid)
            marker_msg.lifetime = rospy.Duration(0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)            

        # edges
        for eid in self._edges:
            nid1, nid2 = self._edges[eid]
            xyz1, xyz2 = self._nodes[nid1], self._nodes[nid2]

            point1 = Point()
            point1.x = xyz1[0]
            point1.y = xyz1[1]
            point1.z = xyz1[2]
            point2 = Point()
            point2.x = xyz2[0]
            point2.y = xyz2[1]
            point2.z = xyz2[2]                        
            
            h = Header()
            h.stamp = timestamp
            h.frame_id = self._marker_frame

            # refer to: http://library.isr.ist.utl.pt/docs/roswiki/rviz(2f)DisplayTypes(2f)Marker.html#Line_Strip_.28LINE_STRIP.3D4.29
            marker_msg = Marker()
            marker_msg.header = h
            marker_msg.type = 4  # Line strip
            marker_msg.ns = "topo_map_edge"
            marker_msg.id = i; i+=1
            marker_msg.action = 0 # add an object
            marker_msg.pose = make_pose_msg((0,0,0), [0,0,0,1])
            marker_msg.scale.x = 0.01
            marker_msg.points = [point1, point2]

            # black
            marker_msg.color.r = 0.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0            
            marker_msg.lifetime = rospy.Duration(0)  # forever
            marker_msg.frame_locked = True
            markers.append(marker_msg)            

        marker_array_msg = MarkerArray(markers)
        return marker_array_msg
