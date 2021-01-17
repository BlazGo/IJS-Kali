"""
Maybe this could be implemented via parameter server? 
Publish/initialize at the beginning and read from server.
"""

import roslib
import rospy
import tf
from visualization_msgs.msg import MarkerArray, Marker
print("[INFO] Starting object positioner")


class meshDisplay():
    """ 
    Class with functions to draw and delete markers
    """

    def __init__(self, color=(255, 0, 255, 255)):
        self.color = color
        #rate = rospy.Rate(1)

        # Define ROS node
        try:
            rospy.init_node("pose_track", anonymous = True)
        except:
            print("Node not initialized (already exists?).")
        
        self.publisher = rospy.Publisher("obj_model_ref", MarkerArray, queue_size = 10)
        rospy.sleep(1)

    def draw_mesh(self, pose, rot=(0.0, 0.0, 0.0, 1.0), color=(0.5, 0.5, 0.5, 1.0), scale=0.01):

        self.marker_list = MarkerArray()
        self.marker = Marker()

        self.marker.header.frame_id = "/panda_link0"
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://obj_calib/meshes/ipad7.stl"
        self.marker.action = self.marker.ADD

        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = color[3]

        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale

        self.marker.pose.orientation.w = rot[3]
        self.marker.pose.orientation.x = rot[0]
        self.marker.pose.orientation.y = rot[1]
        self.marker.pose.orientation.z = rot[2]

        self.marker.pose.position.x = -pose[0]+2*0.08705
        self.marker.pose.position.y = -pose[1]
        self.marker.pose.position.z = -pose[2]

        self.marker_list.markers.append(self.marker)
        self.publisher.publish(self.marker_list)

    def delete_markers(self):
        try:
            for m in self.marker_list.markers:
                m.action = m.DELETE
        
            self.publisher.publish(self.marker_list)
        except AttributeError:
            pass

if __name__ == "__main__":
    if False:
        mTf = modelTf()
        while rospy.is_shutdown() == False:
            mTf.pub_tf(mTf.trans, mTf.eulers)
            rospy.sleep(2)
    else:
        md = meshDisplay()

        while rospy.is_shutdown() == False:
            md.draw_mesh(pose = [0.5,0.5,0])
            rospy.sleep(2)
