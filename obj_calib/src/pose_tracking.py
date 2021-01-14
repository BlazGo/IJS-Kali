import rospy
import moveit_commander
import moveit_msgs.msg
import sys
import tf
import tty
import termios

print("\n[INFO] Starting pose tracking\n")

class poseTrack():
    """
    Class with ros node to read the position of robot end effector
    """

    def __init__(self):
        """
        Input:
            - topic on which the base tf is broadcasted
              (default panda_link0)
            - topic on which the endeffector tf is broadcasted on
              (default panda_hand)
        
        Function:
            Save the pose into class variable when run
        """

        self.points = []
        
        # Defifning ros node
        try:
            rospy.init_node("pose_track", anonymous = True)
        except:
            print("Node not initialized (already exists?).")
            pass

        self.listener = tf.TransformListener()
        rospy.sleep(1) # Need to wait to fill buffer or error for no data will appear
    
    def curr_pose(self):
        trans, rot = self.listener.lookupTransform( "panda_link0",
                                                    "panda_hand",
                                                    rospy.Time(0))
        return trans, rot
        
    def record_pose(self):
        trans, rot = self.curr_pose()
        self.points.append(trans)
        #print("Point recorded:\n{}".format(trans))
        return trans

    def reset_points(self):
        self.points = []
        #print("Points reset!") 

if __name__ == "__main__":

    # --------------------------------------------------#
    # Defifning keypresses
    # --------------------------------------------------#
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    pt = poseTrack()
    print("Entered loop")

    while not rospy.is_shutdown():
        try:
            # Colect keypress (currently if multiple keypresses are pressed it queues them)
            key = sys.stdin.read(1)[0]
            
            # if keypress is detected do something
            if key == "p":
                print(key)
                pt.get_pose()
                print("%s" % pt.points())
            
            # If less than 3 sets of points collected no action 
            if key == "s" and len(pt.points) < 3:
                print("Not enough points")
            
            # If p is pressed and enough points is collected
            if key == "s" and len(pt.points) >= 3:
                print("Sending points")
                print(pt.points)
            rospy.sleep(2) 

        # Exit if CTRL + c is pressed
        except rospy.ROSInterruptException:
            pass        
        #print("%s" % robot.get_current_state())
        key = ""
    print("Stopped gathering pose...")