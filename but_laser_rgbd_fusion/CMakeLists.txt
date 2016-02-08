cmake_minimum_required(VERSION 2.8.3)
project(but_laser_rgbd_fusion)


find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  actionlib
  amcl
  cv_bridge
  geometry_msgs
  gmapping
  hokuyo_node
  laser_geometry
  move_base_msgs
  nav_msgs
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

)



include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a cpp executable
 add_executable(laser_rgbd_registration src/laser_rgbd_registration.cpp)
 add_executable(filter_hokuyo src/filter_hokuyo.cpp)
 add_executable(laser_rgbd_fusion src/laser_rgbd_fusion.cpp)
 add_executable(laser_rgbd_pc2 src/laser_rgbd_pc2.cpp)

## Add cmake target dependencies of the executable/library
 add_dependencies(laser_rgbd_registration laser_rgbd_registration_generate_messages_cpp)
 add_dependencies(laser_rgbd_fusion laser_rgbd_fusion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(laser_rgbd_registration
   ${catkin_LIBRARIES}
 )

target_link_libraries(filter_hokuyo
   ${catkin_LIBRARIES}
 )

target_link_libraries(laser_rgbd_fusion
   ${catkin_LIBRARIES}
 )

target_link_libraries(laser_rgbd_pc2
   ${catkin_LIBRARIES}
 )
 



