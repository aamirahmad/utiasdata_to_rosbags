cmake_minimum_required(VERSION 2.8.3)
project(utiasdata_to_rosbags)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  rosbag
  cv_bridge
  message_generation
  image_transport
)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# generate_messages(
#    DEPENDENCIES std_msgs )

   
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MCLAMMeasurementData.msg
  MCLAM_landmark_GTData.msg
  MCLAM_RobotTeamState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs     
  nav_msgs  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES socrobdata_to_rosbags
CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)




add_executable(utiasdata_to_bag src/data_to_bag.cpp)
target_link_libraries(utiasdata_to_bag ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

# add_executable(utiasGTdata_to_bag src/gtdata_to_bag.cpp)
# target_link_libraries(utiasGTdata_to_bag ${catkin_LIBRARIES} ${rosbag_LIBRARIES})