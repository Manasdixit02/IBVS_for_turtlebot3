#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Callback function to process the depth image
def depth_callback(msg):
    # Initialize CvBridge
    bridge = CvBridge()

    # Check the encoding of the depth image and convert it accordingly
    if msg.encoding == '32FC1':
        # Depth image is in 32-bit float format (depth in meters)
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    elif msg.encoding == '16UC1':
        # Depth image is in 16-bit unsigned int format (depth in millimeters)
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_image = depth_image / 1000.0  # Convert from mm to meters
    elif msg.encoding == '8UC1':
        # Depth image is in 8-bit unsigned int format (not typical for depth, but handling for safety)
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        depth_image = depth_image.astype(np.float32)  # Convert to float32
    else:
        rospy.logwarn(f"Unknown encoding {msg.encoding}. Converting to 32FC1.")
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    # Define the pixel coordinates for which to estimate depth (x, y)
    x, y = 1076, 316  # Example: center pixel (adjust as needed)
    
    # Extract the depth value at (x, y) in meters
    depth_at_point = depth_image[y, x]  # Depth value in meters
    
    # Print the depth at the specific point
    rospy.loginfo(f"Depth at point ({x}, {y}): {depth_at_point} meters")
    
    # Optionally, display the depth image using OpenCV
    #cv2.imshow("Depth Image", depth_image)
    #cv2.waitKey(1)
    
def Corner_det(self,img):
    # Convert the image to HSV color space
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # Lower and upper red range in HSV
    lower_red = np.array([0, 100, 100], dtype="uint8")
    upper_red = np.array([10, 255, 255], dtype="uint8")
    lower_red2 = np.array([170, 100, 100], dtype="uint8")
    upper_red2 = np.array([180, 255, 255], dtype="uint8")
    # Mask for red areas
    mask1 = cv.inRange(hsv, lower_red, upper_red)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
    # Combine the two masks
    red_mask = cv.bitwise_or(mask1, mask2)

    lower_blue = np.array([100, 100, 100], dtype="uint8")
    upper_blue = np.array([140, 255, 255], dtype="uint8")
    # Create a mask for blue color
    blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

    lower_green = np.array([40, 100, 50], dtype="uint8")
    upper_green = np.array([90, 255, 255], dtype="uint8")
    # Create a mask for green color
    green_mask = cv.inRange(hsv, lower_green, upper_green)

    lower_black = np.array([0, 0, 0], dtype="uint8")
    upper_black = np.array([180, 255, 50], dtype="uint8")
    # Create a mask for black color
    black_mask = cv.inRange(hsv, lower_black, upper_black)
        
    kek_sp = scipy.ndimage.center_of_mass(blue_mask)
    zold_sp = scipy.ndimage.center_of_mass(green_mask)
    piros_sp = scipy.ndimage.center_of_mass(red_mask)
    fekete_sp = scipy.ndimage.center_of_mass(black_mask)
             
    corners = np.array([[piros_sp[0],piros_sp[1]],[zold_sp[0],zold_sp[1]],[kek_sp[0],kek_sp[1]],[fekete_sp[0],fekete_sp[1]]])
    #print(corners)
    return corners

def main():
    # Initialize the ROS node
    rospy.init_node('depth_estimation_node', anonymous=True)

    # Subscribe to the depth image topic (replace with your actual topic)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)

    # Spin to keep the program running
    rospy.spin()

    # Clean up OpenCV windows on exit
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

