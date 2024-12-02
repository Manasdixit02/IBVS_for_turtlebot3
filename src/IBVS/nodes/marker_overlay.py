#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray  # Example message type, replace with the appropriate message type
import numpy as np

def callback(msg):
    """
    Callback function that gets triggered when a message is received
    from /some_topic. This will publish markers to RViz.
    """
    
    #post processing the data
    v1 = np.append(msg.data[0:2], 1)
    v2 = np.append(msg.data[2:4], 1)
    v3 = np.append(msg.data[4:6], 1)
    v4 = np.append(msg.data[6:8], 1)
    
    rotation_pos = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    
    point1 = rotation_pos@v1
    point2 = rotation_pos@v2
    point3 = rotation_pos@v3
    point4 = rotation_pos@v4
    
    # Initialize marker publisher for MarkerArray
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    
    # Create a MarkerArray to hold multiple markers
    marker_array = MarkerArray()

    # List of 3D points (example)
    points_3d = [
        Point(x=point1[0], y=point1[1], z=point1[2]),
        Point(x=point2[0], y=point2[1], z=point2[2]),
        Point(x=point3[0], y=point3[1], z=point3[2]),
        Point(x=point4[0], y=point4[1], z=point4[2])
        #Point(x=0.5, y=1.0, z=3.0)
    ]

    # Iterate through the points and create a marker for each
    for i, point_3d in enumerate(points_3d):
        marker = Marker()
        marker.header.frame_id = "base_footprint"  # The frame of reference for the markers
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = i  # Unique ID for each marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_3d.x
        marker.pose.position.y = point_3d.y
        marker.pose.position.z = point_3d.z
        marker.scale.x = 0.05  # Sphere size
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque
        
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        

        # Add the marker to the MarkerArray
        marker_array.markers.append(marker)

    # Publish the MarkerArray
    marker_pub.publish(marker_array)

def listener():
    """
    Sets up the subscriber and listens for messages on a topic.
    """
    rospy.init_node('marker_publisher', anonymous=True)
    
    # Subscribe to the topic (e.g., /some_topic)
    rospy.Subscriber('/marker_overlay', Float64MultiArray, callback)  # Change String to the actual message type you expect

    # Keep the node running and listening for incoming messages
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()  # Start listening for messages
    except rospy.ROSInterruptException:
        pass

