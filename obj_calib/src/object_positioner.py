"""
Maybe this could be implemented via parameter server? 
Publish/initialize at the beginning and read from server.
"""

import roslib
import rospy
import tf
from random import random

x, y, z = 0, 0, 0

class randPos():
    """
    random pos generator
    """
    def __init__(self, x_range = (-0.5, 0.5),
                y_range = (-0.5, 0.5),
                z_range = (-0.2, 0.2)):

        """
        Maybe make a functionality to scale the workspace
        """

        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        
        self.x = random() - 1
        self.y = random() - 1
        self.z = (random() - 1) * 0.2 
        print("Initialised random class")

    def generate_Newpos(self):
        self.x = random() - 1
        self.y = random() - 1
        self.z = (random() - 1) * 0.2 
        
        print("generated random pos")

    def get_pos(self):
        print("Retrieved pos")
        return self.x, self.y, self.z

objectname = "my_object"
rospy.init_node("object_positioner")
broTf = tf.TransformBroadcaster()

pos = randPos()
trans = pos.get_pos()

while not rospy.is_shutdown():
    broTf.sendTransform(trans,
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(),
                        objectname,
                        "panda_link0")

    rospy.sleep(2)