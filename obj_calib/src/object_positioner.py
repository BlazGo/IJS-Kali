"""
Maybe this could be implemented via parameter server? 
Publish/initialize at the beginning and read from server.
"""

import roslib
import rospy
import tf

class modelTf():
    """
    random pos generator
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

    def pub_tf(self):
        self.tfBroadcaster.sendTransform(self.trans,
                                        tf.transformations.quaternion_from_euler(self.eulers[0], self.eulers[1], self.eulers[2], axes="rzyx"),
                                        rospy.Time.now(),
                                        self.object_name,
                                        self.base_link)

if __name__ == "__main__":
    mTf = modelTf()
    while rospy.is_shutdown() == False:
        mTf.pub_tf()
        rospy.sleep(2)
        