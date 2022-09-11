cmake_minimum_required(VERSION 3.10.0)
project(drone_tracking)

# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  # need to add mavros 
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  ${catkin_LIBRARIES} 
)

add_message_files(
 FILES
 LQRGain.msg
)

#add_service_files(
#  FILES
#)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 geographic_msgs
)

catkin_package(
  # LIBRARIES airsim_ros
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  message_runtime 
  roscpp 
  std_msgs 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${RPC_LIB_INCLUDES}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)



##################
##  Install #####
#################



add_executable(uas_px4_main src/uas_px4_main.cpp src/PID.cpp src/uas_px4.cpp)
add_dependencies(uas_px4_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(uas_px4_main  ${catkin_LIBRARIES})


install(TARGETS
    #TARGETS null_testlogger.
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
    scripts/offboard_drone.py
    scripts/apriltag_position_pub.py
    scripts/mavros_tf.py
    scripts/apriltag_norm.py
    scripts/ekf_tag.py
    scripts/land_permission.py
    scripts/logger.py
    scripts/user_control.py
#    #list of shared libraries
#    pd_position_controller_simple
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES 
  #list of necessary files (xml...)
  #README.md
  # settings.json
  # nodelet_plugins.xml
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
