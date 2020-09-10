cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  message_generation
  message_filters
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

add_service_files(
  FILES
  get_pick_pose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES pose_estimation
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf message_runtime
  #INCLUDE_DIRS include
  DEPENDS Eigen
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_obj_pose src/pointcloud_ClusterExtraction.cpp)
add_dependencies(get_obj_pose pose_estimation_gencpp)
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set_target_properties(get_obj_pose PROPERTIES COMPILE_DEFINITIONS "VERBOSE")
endif()
target_link_libraries(get_obj_pose ${catkin_LIBRARIES})

add_executable(ex src/ex.cpp)
add_dependencies(get_obj_pose pose_estimation_gencpp)
target_link_libraries(ex ${catkin_LIBRARIES})
