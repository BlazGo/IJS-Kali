#! /usr/bin/env python
import tf
import rospy
from tf.transformations import random_rotation_matrix as randm

rospy.init_node("tf_tool", anonymous=True)
pub = tf.TransformBroadcaster()
rospy.sleep(0.2)

i = 0
while not rospy.is_shutdown():
    pub.sendTransform((0.0, -0.22, 0.16),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "tool",
                        "panda_EE")
    
    if i >= 200:
        print("[INFO] Publishing tool tf")
        i = 0
        
    i += 1
    #print(randm())
    rospy.sleep(0.05)
