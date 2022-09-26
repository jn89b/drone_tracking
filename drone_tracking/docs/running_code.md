roslaunch px4_sitl.launch 
roslaunch apriltag_sitl.launch
roslaunch broadcast_tf.launch

rosrun drone_tracking px4_offboard_main
rosrun drone_tracking lqr_track.py
rosrun drone_tracking user_cmd.py 