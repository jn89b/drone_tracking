Unmanned Traffic Management with ROS and Unreal Engine

######Authors########### 
Justin Nguyen 
Peter Nguyen 
supervised by: Mujahid Abdulrahim 
########################

Developed by: University of Missouri-Kansas City FAST Lab


# Starting
- Follow the tutorials to download mavros, mavlink and px4

- Download apriltag ros 

```
cd ~/catkin_ws/src

git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper

cd ../ 
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages


```