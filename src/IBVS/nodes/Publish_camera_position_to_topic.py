#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
import signal
import sys
import rospy 
import rospkg 
import numpy as np

    
def publish_camera_pose(Camera_pose):
    pub = rospy.Publisher('/Camera_position', Float64MultiArray, queue_size=10)
    rospy.init_node('set_pose_once', anonymous=True)
    rate = rospy.Rate(100)
    my_msg = Float64MultiArray()
    my_msg.data = Camera_pose.flatten()
    my_msg.layout.data_offset =  0
    pub.publish(my_msg)    
    
if __name__ == '__main__':
    #values = np.array([-1.5,0.0,0.0,90.0,0.0,-90.0])
    #values = np.array([-1.0,0.0,0.0,90.0,0.0,-90.0])
    values = np.array([0.0,4.0,0.0,90.0,0.0,0.0])
    publish_camera_pose(values)
