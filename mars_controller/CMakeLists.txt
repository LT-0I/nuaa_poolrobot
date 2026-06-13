cmake_minimum_required(VERSION 2.8.3)
project(mars_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  mars_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.8)
# find_package(OpenCV)
set(PYTHON_INCLUDE_DIRS "/usr/include/python3.8")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs mars_msgs
)

include_directories(
  include
  ../mars_bringup/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(mars_controller_exe  src/motion_controller.cpp)


add_dependencies(mars_controller_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(mars_controller_exe PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mars_controller_exe  emath_exe ${catkin_LIBRARIES} ${PYTHON_LIBRARIES}  )
install(TARGETS mars_controller_exe RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES   launch/motion_controller.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
