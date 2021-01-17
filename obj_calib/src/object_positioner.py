"""
Maybe this could be implemented via parameter server? 
Publish/initialize at the beginning and read from server.
"""

import roslib
import rospy
import tf
from visualization_msgs.msg import MarkerArray, Marker

class modelTf():
    """
    tf publisher for model
    """

    def __init__(self, node_name="obj_calib", object_name="my_object", base_link="panda_link0"):

        self.object_name = object_name
        self.base_link = base_link
        
        self.X_POS = 0.3
        self.Y_POS = 0.3
        self.Z_POS = 0

        self.trans = [self.X_POS, self.Y_POS, self.Z_POS]
        self.eulers = [0,0,0]

        try:
            rospy.init_node("object_positioner", anonymous=True)
        except:
            print("[WARN] Node already exists")

        self.tfBroadcaster = tf.TransformBroadcaster()

    def pub_tf(self, trans, eulers):
        self.tfBroadcaster.sendTransform(trans,
                                        tf.transformations.quaternion_from_euler(eulers[0], eulers[1], eulers[2], axes="rzyx"),
                                        rospy.Time.now(),
                                        self.object_name,
                                        self.base_link)


class markerDisplay():
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
        
        #rospy.sleep(rate)
        self.publisher = rospy.Publisher("points_array", MarkerArray, queue_size = 10)
        rospy.sleep(1)

    def draw_markers(self, pose):

        self.marker_list = MarkerArray()
        self.marker = Marker()

        self.marker.header.frame_id = "/panda_link0"
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://obj_calib/meshes/robot_link1.stl"
        self.marker.action = self.marker.ADD

        self.marker.color.r = 0
        self.marker.color.g = 0
        self.marker.color.b = 255
        self.marker.color.a = 255

        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = pose[0]
        self.marker.pose.position.y = pose[1]
        self.marker.pose.position.z = pose[2]

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
        md = markerDisplay()

        while rospy.is_shutdown() == False:
            md.draw_markers([0,0,0])
            rospy.sleep(2)
