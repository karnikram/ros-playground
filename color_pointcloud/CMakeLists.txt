cmake_minimum_required(VERSION 2.8.3)
project(color_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  message_filters
  pcl_ros
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES color_pointcloud
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(OpenCV REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(run run.cpp)

target_link_libraries(run
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
