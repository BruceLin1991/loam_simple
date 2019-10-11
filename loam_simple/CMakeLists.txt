cmake_minimum_required(VERSION 2.8.3)
project(loam_simple)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	/usr/include/eigen3/Eigen
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

add_definitions( -march=native )


add_executable(loam_node src/loam_node.cpp src/scanRegistration.cpp src/laserOdometry.cpp src/laserMapping.cpp)
target_link_libraries(loam_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )



