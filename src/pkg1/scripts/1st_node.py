#!/usr/bin/env python3
import rospy

if __name__=='__main__':
    rospy.init_node("node1")
    rospy.loginfo("Hello from test node")
    rospy.logwarn("This is warning")
    rospy.logerr("This is an error")

    rospy.sleep(1.0)

    rospy.loginfo('End of program')