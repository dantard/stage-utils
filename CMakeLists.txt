cmake_minimum_required(VERSION 2.8.3)
project(stage_utils)

add_definitions(-std=gnu++0x)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package( PCL REQUIRED )

catkin_package(
        CATKIN_DEPENDS roscpp rospy geometry_msgs pcl_ros costmap_2d nav_core
)

include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(stage_initial_pose src/stage_initial_pose.cpp)
add_executable(stage_odom_to_posestamped src/stage_odom_to_posestamped.cpp)

target_link_libraries(stage_initial_pose  ${catkin_LIBRARIES})
target_link_libraries(stage_odom_to_posestamped  ${catkin_LIBRARIES})

