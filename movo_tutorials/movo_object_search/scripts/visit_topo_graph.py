# Visit every node of a topological graph

from search_in_region import get_param
from action.waypoint import WaypointApply

def main():
    rospy.init_node("movo_topo_map_visitor",
                    anonymous=True)
    topo_map_file = get_param('topo_map_file')
    search_space_resolution = get_param("search_space_resolution")    
    pub = PublishTopoMarkers(topo_map_file, search_space_resolution)
    
    waypoints = []
    i = 0
    for nid in sorted(pub.nodes):
        posit = pub.nodes[nid]
        orien = (0,0,0,1)
        waypoints.append(((posit, orient), nid))
        rospy.loginfo("[%d] Node %d, %s" % (i, str(posit)))
        i += 1

    i = 0
    for goal, nid in waypoints:
        posit, orien = goal
        rospy.loginfo("Navigating to Node %d at %s [step %d]" % (nid, str(goal), i))
        if WaypointApply(posit, orien).status != WaypointApply.Status.SUCCESS:
            rospy.logerr("Waypoint to %d failed" % nid)
            break
        i += 1
    rospy.loginfo("All done!")
    
if __name__ == "__main__":
    main()
