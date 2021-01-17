import roslib
import rospy
from visualization_msgs.msg import MarkerArray, Marker

print("\n[INFO] Starting marker display\n")

class markerDisplay():
    """ 
    Class with functions to draw and delete markers
    """

    def __init__(self, color=(255, 0, 255, 255)):
        self.color = color
        rate = rospy.Rate(1)

        # Define ROS node
        try:
            rospy.init_node("pose_track", anonymous = True)
        except:
            print("Node not initialized (already exists?).")
            
        self.publisher = rospy.Publisher("points_array", MarkerArray, queue_size = 10)

    def draw_markers(self, position_list):
        """
        Draws given points in position_list.
        
        Input:
            - position_list
                List of poses, each pose contains x,y,z coordinates 
        Output
            - publishes marker array to given topic
        """
        
        if position_list != None:
            self.marker_list = MarkerArray()        

            for pose in position_list:
                self.marker = Marker()

                self.marker.header.frame_id = "/panda_link0"
                self.marker.type = self.marker.SPHERE
                self.marker.action = self.marker.ADD

                self.marker.scale.x = 0.05
                self.marker.scale.y = 0.05
                self.marker.scale.z = 0.05

                self.marker.color.r = 0
                self.marker.color.g = 0
                self.marker.color.b = 255
                self.marker.color.a = 255

                self.marker.pose.orientation.w = 1.0
                self.marker.pose.position.x = pose[0]
                self.marker.pose.position.y = pose[1]
                self.marker.pose.position.z = pose[2]

                self.marker_list.markers.append(self.marker)

            id = 0
            for m in self.marker_list.markers:
                m.id = id
                id += 1

            self.publisher.publish(self.marker_list)
        
        else:
            print("[INFO] No points recorded yet, cannot draw.")

    def delete_markers(self):
        """
        Input:
            /
        Output: 
            - deletes points defined in markerArray
        """
        #print("deleting markers")
        try:
            for m in self.marker_list.markers:
                m.action = m.DELETE
        
            self.publisher.publish(self.marker_list)
        except AttributeError:
            pass

if __name__ == "__main__":

    pose = [0.1, 0.2, 0.0]
    position_list = []
    position_list.append(pose)

    while not rospy.is_shutdown():

        md = markerDisplay()
        md.draw_markers(position_list)
        rospy.sleep(5)
        md.delete_markers()
        rospy.sleep(5)