#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

class DepthEstimation:
    def __init__(self):
        rospy.init_node('depth_estimation_node', anonymous=True)

        # Initialize CvBridge for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Camera model to get intrinsic parameters
        self.camera_model = PinholeCameraModel()

        # Subscribe to the depth image and camera info topics
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)

        # Store the camera info
        self.camera_info = None

        rospy.spin()

    def camera_info_callback(self, msg):
        """
        Callback function to store camera intrinsic parameters.
        """
        self.camera_model.fromCameraInfo(msg)
        self.camera_info = msg
        rospy.loginfo("Camera information received.")

    def depth_image_callback(self, msg):
        """
        Callback function to process the depth image.
        """
        if self.camera_info is None:
            rospy.logwarn("Camera info not received yet.")
            return

        # Convert the ROS depth image message to an OpenCV format
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Check the encoding type and handle accordingly
        if msg.encoding == "16UC1":
            # Depth values are in millimeters, convert them to meters
            depth_image = depth_image / 1000.0
        elif msg.encoding == "32FC1":
            # Depth values are already in meters
            pass
        else:
            rospy.logwarn(f"Unexpected depth image encoding: {msg.encoding}")
            return

        # Create a copy of the depth image to modify
        depth_image_copy = np.copy(depth_image)

        # Replace invalid depth values (0) with NaN
        depth_image_copy[depth_image_copy == 0] = np.nan

        # Example: Get the min and max depth in the entire frame
        min_depth = np.nanmin(depth_image_copy)
        max_depth = np.nanmax(depth_image_copy)

        rospy.loginfo(f"Min Depth: {min_depth:.3f} meters")
        rospy.loginfo(f"Max Depth: {max_depth:.3f} meters")

        # Optionally, log depth values at random positions to check
        for i in range(10):  # Print depth at 10 random positions
            random_x = np.random.randint(0, depth_image_copy.shape[1])
            random_y = np.random.randint(0, depth_image_copy.shape[0])
            depth_value = depth_image_copy[random_y, random_x]
            rospy.loginfo(f"Depth at ({random_x},{random_y}): {depth_value:.3f} meters")

        # Visualize depth (for debugging)
        cv2.imshow("Depth Image", depth_image_copy)
        cv2.waitKey(1)

        # Example: Find the minimum depth in the image
        min_depth_value = np.nanmin(depth_image_copy)
        rospy.loginfo(f"Minimum depth in the image: {min_depth_value:.3f} meters")

        # Project the 3D point from pixel coordinates to camera frame
        height, width = depth_image_copy.shape
        u = width // 2  # Center of the image (you can change this to any other pixel)
        v = height // 2

        # Get depth at center pixel
        depth_at_center = depth_image_copy[v, u]
        rospy.loginfo(f"Depth at center: {depth_at_center:.3f} meters")

        # If depth is valid (not NaN), project into camera frame (3D coordinates)
        if not np.isnan(depth_at_center):
            Z = depth_at_center
            fx = self.camera_model.fx()
            fy = self.camera_model.fy()
            cx = self.camera_model.cx()
            cy = self.camera_model.cy()

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy

            rospy.loginfo(f"3D point in camera frame: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
        else:
            rospy.logwarn(f"Invalid depth at center")
    
    def shutdown(self):
        """
        Perform any shutdown cleanup if needed.
        """
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        depth_estimation = DepthEstimation()
    except rospy.ROSInterruptException:
        pass

