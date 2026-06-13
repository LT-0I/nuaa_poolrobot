cmake_minimum_required(VERSION 3.0.2)
project(mars_bringup)

set(CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  mars_msgs
  tf
)




catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs mars_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(bringup_exe src/bringup.cpp)

add_library(emath_exe SHARED src/emath.cpp)

add_dependencies(bringup_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bringup_exe ${catkin_LIBRARIES})