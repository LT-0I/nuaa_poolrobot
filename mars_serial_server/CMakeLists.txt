cmake_minimum_required(VERSION 3.0.2)
project(mars_serial_server)

set(CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  mars_msgs
  tf
  image_transport
  cv_bridge
)
find_package(OpenCV)
# add_message_files(
#   FILES
# )
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )




catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs mars_msgs message_runtime tf cv_bridge
)


include_directories(
  include
  ../mars_bringup/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


set(BOOST_LIBS thread date_time system)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)




add_executable(serial_server_exe src/serial_starup.cpp
                                 src/hardware_driver.cpp
                                 src/AsyncSerial.cpp  
              )


add_dependencies(serial_server_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(serial_server_exe  emath_exe ${catkin_LIBRARIES} ${OpenCV_LIBS})
# set_target_properties(agv_server PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "") 意思是type name 可以用pack

install(TARGETS serial_server_exe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES launch/ares_serial_server.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
