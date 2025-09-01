# IBVS for TurtleBot3

This project implements **Image-Based Visual Servoing (IBVS)** for real-time object tracking and control of a TurtleBot3 in ROS Noetic with Gazebo simulation. The system tracks four keypoints of a reference marker in the camera feed and uses their image-space error to generate control commands.  

When keypoints are occluded, the pipeline estimates their positions using feature matching, RANSAC filtering, and homography-based reconstruction, ensuring robust tracking even under partial visibility.  

---

## Key Features
- End-to-end **IBVS pipeline** in ROS 1 (Noetic).  
- **Occlusion handling**: reconstructs hidden keypoints via homography and RANSAC.  
- Uses **RGB + depth images** for accurate feature localization.  
- Gazebo world with a marker for simulation (`wall_qr.world`).  
- Modular nodes for position publishing, camera setup, and IBVS control.  

---

## Quick Start (Simulation)

1. **Install prerequisites**  
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-turtlebot3-gazebo                  ros-noetic-cv-bridge ros-noetic-image-transport                  python3-opencv
```

2. **Clone and build**  
```bash
mkdir -p ~/ibvs_ws/src && cd ~/ibvs_ws/src
git clone https://github.com/Manasdixit02/IBVS_for_turtlebot3.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

3. **Pick your TurtleBot3 model**  
```bash
export TURTLEBOT3_MODEL=waffle   
```

4. **Launch Gazebo with the world**  
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch   world_file:=$(rospack find my_turtlebot3_simulation)/worlds/wall_qr.world
```

5. **Publish marker and robot positions**  
```bash
rosrun IBVS Publish_marker_position_to_topic.py
rosrun IBVS Publish_camera_position_to_topic.py
```

6. **Set positions inside Gazebo**  
```bash
rosrun IBVS set_camera_position_from_topic.py
```

7. **Run the IBVS controller**  
```bash
rosrun IBVS IBVS_test_overlay.py
```

---

## How It Works
1. Marker and robot positions are published to ROS topics and placed in Gazebo.  
2. The IBVS node subscribes to **RGB and depth images**, detects four keypoints on the marker, and tracks them over time.  
3. If keypoints are lost:  
   - Performs feature matching between current and last valid image.  
   - Filters matches using **RANSAC**.  
   - Computes **homography** to reconstruct hidden keypoints.  
4. Computes control commands from keypoint errors and drives the TurtleBot via `/cmd_vel`.  

---

## Results

### 1. IBVS Demo in Non-Occluded Scenarios
[![IBVS Demo](https://img.youtube.com/vi/02knEVV_Q8w/0.jpg)](https://www.youtube.com/watch?v=02knEVV_Q8w)  
*TurtleBot3 tracks the reference marker using IBVS in Gazebo.*

### 2. IBVS Performance Under Occlusion (Without Key Estimation)
[![IBVS Occlusion Without Estimation](https://img.youtube.com/vi/IyRRwgQm2kk/0.jpg)](https://www.youtube.com/watch?v=IyRRwgQm2kk)  
*Shows IBVS performance when keypoints are occluded and no estimation logic is applied.*

### 3. IBVS Performance Under Occlusion (With Key Estimation)
[![IBVS Occlusion With Estimation](https://img.youtube.com/vi/0eTwzXML35s/0.jpg)](https://www.youtube.com/watch?v=0eTwzXML35s)  
*Demonstrates IBVS performance when keypoint estimation logic (feature matching, RANSAC, homography) is integrated, enabling robust control under occlusion.*

---

## Repository Structure
```
IBVS_for_turtlebot3/
├─ IBVS/                # Main IBVS nodes (controllers, helpers)
├─ my_turtlebot3_simulation/  # Gazebo world + marker model
├─ marker_teleop/       # Utility launch and teleop
└─ README.md
```

---

## Future Work
- Deploy pipeline on a real TurtleBot3 with RGB-D camera.  
- Extend to multi-marker/object tracking.  
- Integrate adaptive Jacobian learning for improved robustness.  
