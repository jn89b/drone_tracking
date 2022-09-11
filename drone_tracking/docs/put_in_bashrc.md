In your .bashrc fiile put the following in:


```
px4_dir=~/PX4-Autopilot

source $px4_dir/Tools/simulation/gazebo/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```
