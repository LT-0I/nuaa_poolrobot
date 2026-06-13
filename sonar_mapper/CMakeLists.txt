cmake_minimum_required(VERSION 3.0.2)
project(sonar_mapper)

set(CMAKE_CXX_STANDARD 17)

# C++ 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  cv_bridge
  sensor_msgs
  nav_msgs
  image_transport
  mars_msgs
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time system)
find_package(Eigen3 REQUIRED)

# 正确查找Threads库
find_package(Threads REQUIRED)

# find_package(Pangolin REQUIRED)  # 先注释掉

# 必须在include_directories之前
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    tf 
    cv_bridge 
    image_transport 
    sensor_msgs 
    nav_msgs 
    mars_msgs
    tf2
    tf2_ros
  INCLUDE_DIRS 
    include
  # 如果您没有库要导出，就不要写LIBRARIES这一行
  # LIBRARIES
  #   sonar_mapper_lib  # 如果您有库，取消注释并命名
)

# 必须在catkin_package之后
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  # ${Pangolin_INCLUDE_DIRS}  # 如果需要，取消注释
)

# 添加Eigen3定义
add_definitions(${Eigen3_DEFINITIONS})

# 注释掉：运动补偿库（如果需要，以后再添加）
# add_library(motion_compensator
#   src/MotionCompensator.cpp
# )
# 
# target_link_libraries(motion_compensator
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
# )

# 注释掉：EKF库（如果需要，以后再添加）
# add_library(simple_ekf
#   src/SimpleEKF.cpp
# )
# 
# target_link_libraries(simple_ekf
#   ${catkin_LIBRARIES}
#   ${Eigen3_LIBRARIES}
# )

# 主执行文件
add_executable(SonarDriver
  src/SonarDriver.cpp
  src/AsyncSerial.cpp
  # src/Viewer.cpp  # 如果你需要Viewer，取消注释；如果需要Pangolin，先安装
)

# 正确链接Threads库
target_link_libraries(SonarDriver
  # motion_compensator  # 注释掉
  # simple_ekf         # 注释掉
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}  # 正确引用Threads库
  # ${Pangolin_LIBRARIES}  # 如果需要Viewer且安装了Pangolin，取消注释
)

add_dependencies(SonarDriver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 安装目标
install(TARGETS SonarDriver 
  # motion_compensator  # 注释掉
  # simple_ekf         # 注释掉
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".gitkeep" EXCLUDE
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".gitkeep" EXCLUDE
)