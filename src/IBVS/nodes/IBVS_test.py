#!/usr/bin/env python3
import os
import sys
import math
import time
import scipy
import rospy
import cv2 as cv
import numpy as np
import math

from geometry_msgs.msg import Pose, Twist, Vector3

from os import system, name
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
#from csv import writer
#import matplotlib.pyplot as plt

bridge = CvBridge()
##################################
# Used class
##################################
class Datas:
    pub = None
    def __init__(self,timestep,P,epsilon):
        self.image = None
        self.camera_coords = None
        self.camera_parameters = None
        self.Feature_image = cv.imread("Feature_image.jpg")
        file1 = open('Feature_image.txt', 'r')
        line = file1.readlines()
        self.Feature_Z = float(line[0])
        self.timestep = timestep
        self.P = P
        self.epsilon = epsilon
        
    def stop_robot(self,pub):
    	stop = Twist()
    	stop.linear = Vector3(0,0,0)
    	stop.angular = Vector3(0,0,0)
    	pub.publish(stop)      
            
    def IBVS(self,msg):

            val = 0
            for i in range(0,len(msg.name)):
                if(msg.name[i]=='turtlebot3_waffle'):
                    val = i
            cam_coords = msg.pose[val]   
            #print (cam_coords)
            r = R.from_quat(np.array([cam_coords.orientation.x,cam_coords.orientation.y,cam_coords.orientation.z,cam_coords.orientation.w]))
            r = r.as_matrix()
            r = np.matmul(r,np.array([[0,0,1],[1,0,0],[0,1,0]]))
            r = R.from_matrix(r)
            r = r.as_euler('xyz',degrees = True)
            if ((self.camera_coords != np.array([cam_coords.position.x,cam_coords.position.y,cam_coords.position.z,r[0],r[1],r[2]])).all()):
              self.camera_coords = np.array([cam_coords.position.x,cam_coords.position.y,cam_coords.position.z,r[0],r[1],r[2]])
              #print(self.camera_coords)
              
              #Perfect so far
              
              #with open('output.csv','a') as csvfile:
              #  np.savetxt(csvfile, np.reshape(self.camera_coords,(1,6)),delimiter=',',fmt='%f')
              projected_desired = self.Corner_det(self.Feature_image)
              #print(projected_desired)
              
              #not perfect, sensible but a little off
              
              projected_current = self.Corner_det(self.image)
              #print(projected_current)
              
              depth = self.depthEst(self.depth_image, projected_current[0,:])
              #print(depth)
              
              if np.isnan(projected_current).any():
                  H = Get_homography(self.Feature_image, self.image)
                  for i in range(projected_current.shape[0]):
                      if np.isnan(projected_current[i,1]):
                          l = np.append(projected_desired[i,:],1)
                          k = H@np.transpose(l)
                          projected_current[i,:] = np.transpose(k[0,0:1])
                      else:
                          continue
                          
                  
              print(projected_current)
              
              
              
              
              
              #Perfect, makes a lot of sense but points possibly way close to each other
              
              p_mm_desired = self.Pixel_to_mm(self.camera_parameters,projected_desired)
              
              
              
              p_mm_current = self.Pixel_to_mm(self.camera_parameters,projected_current)       
              
              #print(p_mm_current)
              #p_mm_desired and current seems good
              sc = p_mm_desired.flatten()
              s = p_mm_current.flatten()
              #error = (s-sc)/self.timestep
              error = (s-sc)
              print(error)
              
              if np.sum(abs(error))<=self.epsilon:
                print("Hiba elhanyagolhato")
                global pub
                stop1 = Twist()
                stop1.linear = Vector3(0,0,0)
                stop1.angular = Vector3(0,0,0)
                pub.publish(stop1) 
                
                #os._exit(0)
              else:
                Interaction_matrix_desired = np.zeros((projected_desired.shape[0]*2,6));
                for i in range(projected_desired.shape[0]):
                    Interaction_matrix_desired[2*i:2*i+2,:] = self.Create_Lx(self.Feature_Z,p_mm_desired[i,:])
              
                Interaction_matrix_current = np.zeros((projected_current.shape[0]*2,6));
                for i in range(projected_current.shape[0]):
                    #Interaction_matrix_current[2*i:2*i+2,:] = self.Create_Lx(self.camera_coords[1],p_mm_current[i,:]) #changed from 1 to 0 in camera_coords[1]
                    Interaction_matrix_current[2*i:2*i+2,:] = self.Create_Lx(depth,p_mm_current[i,:])
              
                Interaction_matrix = (Interaction_matrix_desired +Interaction_matrix_current)/2
                #Interaction_matrix = Interaction_matrix_current
                Interaction_matrix = Interaction_matrix + np.ones((8,6)) * 1e-6
                print(Interaction_matrix)
                
                #perfect,Makes sense
                
                Interaction_matrix_inv = np.linalg.pinv(Interaction_matrix)
                vc = -self.P*Interaction_matrix_inv@np.matrix.transpose(error)
                #print(vc)
                
                rotation_matrix = R.from_euler('xyz',self.camera_coords[3:6],degrees = True).as_matrix()
                
                #original
                #rotation_pos = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
                #rotation_ori = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
                
                #rotation_pos = np.array([[0,1,0],[-1,0,0],[0,0,1]])
                #rotation_ori = np.array([[0,1,0],[1,0,0],[0,0,1]])
                
                rotation_pos = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
                rotation_ori = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
                
                
                #Linear_vec = np.reshape(rotation_pos@rotation_matrix@np.reshape(vc[0:3],(3,1)),(1,3))
                #Rotational_vec = np.reshape(rotation_ori@np.reshape(vc[3:6],(3,1)),(1,3))
                
                Linear_vec = np.reshape(rotation_pos@np.reshape(vc[0:3],(3,1)),(1,3))
                Rotational_vec = np.reshape(rotation_ori@np.reshape(vc[3:6],(3,1)),(1,3))  
                
                #Linear_vec = np.clip(Linear_vec, None, 0.05)
                #print(Rotational_vec)
                
                t = (self.camera_coords[0:3] + np.reshape(Linear_vec,(1,3))*self.timestep)  
                #print(t)  
                state_msg = ModelState()
                state_msg.model_name = 'turtlebot3_waffle'
                
                pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                
                twist = Twist()
                
                twist.linear = Vector3(Linear_vec[0,0], Linear_vec[0,1], Linear_vec[0,2])
                twist.angular = Vector3(Rotational_vec[0,0], Rotational_vec[0,1], Rotational_vec[0,2])
                pub.publish(twist)
                #twist.linear = Vector3(*vc[0:3])
                #twist.angular = Vector3(0,0,0)
                
                rospy.on_shutdown(lambda: self.stop_robot(pub))
                
                #while not rospy.is_shutdown():
                    #pub.publish(twist)
                
                
                
                #print(twist.linear.y)
                
                state_msg.pose.position.x = t[0,0]
                #state_msg.pose.position.x = 0
                state_msg.pose.position.y = t[0,1]
                #state_msg.pose.position.y = 1
                state_msg.pose.position.z = t[0,2]
                #state_msg.pose.position.z = 0
                r = self.camera_coords[3:6] + np.reshape(Rotational_vec,(1,3))*self.timestep*180
                r = R.from_euler('xyz',r,degrees=True).as_matrix()
                r = np.matmul(r,np.array([[0,1,0],[0,0,1],[1,0,0]]))
                r = R.from_matrix(r).as_euler('xyz',degrees=True)
                q = R.from_euler('xyz',r,degrees = True).as_quat()
                state_msg.pose.orientation.x = q[0,0]
                #state_msg.pose.orientation.x = cam_coords.orientation.x
                state_msg.pose.orientation.y = q[0,1]
                #state_msg.pose.orientation.y = cam_coords.orientation.y
                state_msg.pose.orientation.z = q[0,2]
                #state_msg.pose.orientation.z = cam_coords.orientation.z
                state_msg.pose.orientation.w = q[0,3]
                #state_msg.pose.orientation.w = cam_coords.orientation.w
                rospy.wait_for_service('/gazebo/set_model_state')
                #try:
                 #   set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                  #  resp = set_state( state_msg )
                   # print (resp)
                    #print(error)
                #except rospy.ServiceException:
                 #   print("Camera position cannot be set!")
                          
    def image_callback(self, msg):
        self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
        time.sleep(0.05)
        
        
    def depth_callback(self, msg):
        # Initialize CvBridge
        #bridge = CvBridge()

        # Check the encoding of the depth image and convert it accordingly
        if msg.encoding == '32FC1':
            # Depth image is in 32-bit float format (depth in meters)
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        elif msg.encoding == '16UC1':
            # Depth image is in 16-bit unsigned int format (depth in millimeters)
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_image = depth_image / 1000.0  # Convert from mm to meters
        elif msg.encoding == '8UC1':
            # Depth image is in 8-bit unsigned int format (not typical for depth, but handling for safety)
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.depth_image = depth_image.astype(np.float32)  # Convert to float32
        else:
            rospy.logwarn(f"Unknown encoding {msg.encoding}. Converting to 32FC1.")
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        time.sleep(0.05)
        
        
        
        
    def camera_parameters_callback(self, msg):
        self.camera_parameters = msg
        time.sleep(0.05)    
                                
    def Pixel_to_mm(self,camera_parameters,D2_points):
        output = D2_points
        Camera_matrix = np.reshape(camera_parameters.K,(3,3))
        #print(Camera_matrix)
       
        
        
        for i in range(D2_points.shape[0]):
            #output[i,0] = (D2_points[i,0] - Camera_matrix[0,2])/Camera_matrix[0,0]
            #output[i,1] = (D2_points[i,1] - Camera_matrix[1,2])/Camera_matrix[1,1]
            Point_mm = np.matrix.transpose(np.linalg.inv(Camera_matrix)@np.reshape(np.append(D2_points[i,:],1),(3,1)))
            output[i,:] = Point_mm[0,0:2]
        #print(output)
        return output

    def Create_Lx(self,Z,D2_point):
        #Lx = np.array([[-1/Z , 1000000 , D2_point[0]/Z , 1000000 , -(1+D2_point[0]**2) , 1000000],
                         #[0 , 1000000 , D2_point[1]/Z , 1000000 , -D2_point[0]*D2_point[1] , 1000000]])
                         
        Lx = np.array([[-1/Z , 1000000 , D2_point[0]/Z , 1000000 , -(1+D2_point[0]**2) , 1000000],
                         [0 , 1000000 , D2_point[1]/Z , 1000000 , 0 , 1000000]])
        #print(Z)
        return Lx
 
 
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
             
        corners = np.array([[piros_sp[1],piros_sp[0]],[zold_sp[1],zold_sp[0]],[kek_sp[1],kek_sp[0]],[fekete_sp[1],fekete_sp[0]]])
        #print(corners)
        return corners
        
    
    def depthEst(self,img,D2_points):
        x, y = int(D2_points[1]), int(D2_points[0])
        #print(x,y)
        depth_at_point = img[x, y]
        if math.isnan(depth_at_point):
            return 7
        else:
            return depth_at_point
        #print(Camera_matrix)    
    
    
    def get_homography(self, img1, img2):

        # Convert RGB images to grayscale
        img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # Create ORB detector
        orb = cv2.ORB_create()

        # Detect keypoints and descriptors
        kp1, des1 = orb.detectAndCompute(img1_gray, None)
        kp2, des2 = orb.detectAndCompute(img2_gray, None)

        # Use BFMatcher to match descriptors
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)

        # Sort matches by distance (smallest distance first)
        matches = sorted(matches, key=lambda x: x.distance)

        # Extract the matched keypoints coordinates
        pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float32)  # Points from img1
        pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float32)  # Points from img2

        # Apply RANSAC to compute a homography (transformation matrix)
        # RANSAC filters out outlier matches by finding the best model (homography) that fits the inlier points
        H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)  # 5.0 is the threshold for RANSAC

        # Mask indicates which points are inliers (1) or outliers (0)
        inliers = mask.ravel().tolist()
        
        return H
    
                
##################################
##################################                     
if __name__ == '__main__':
    os.chdir(sys.path[0])
    rospy.init_node('IBVS')
    ibvs = Datas(1/60,0.05,0.5)
    rospy.Subscriber('/camera/rgb/image_raw', Image, ibvs.image_callback)
    rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, ibvs.camera_parameters_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, ibvs.depth_callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, ibvs.IBVS)
    rospy.spin()
