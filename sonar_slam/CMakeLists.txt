cmake_minimum_required(VERSION 3.0.2)
project(sonar_slam)

set(CMAKE_CXX_STANDARD 17)
# ==============================================
# 1. 查找 PCL（只调用一次！）
# ==============================================
find_package(PCL 1.10 REQUIRED COMPONENTS 
    common
    registration
    filters
    io
    kdtree
    features
)

# 如果上面失败，尝试不指定版本
if(NOT PCL_FOUND)
    find_package(PCL REQUIRED COMPONENTS 
        common registration filters io kdtree features)
endif()

# 设置 PCL 定义
if(PCL_FOUND)
    set(PCL_DEFINITIONS ${PCL_DEFINITIONS} 
        -DEIGEN_USE_NEW_STDVECTOR 
        -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
    message(STATUS "PCL found: ${PCL_VERSION}")
    message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "PCL not found! Please install: sudo apt install libpcl-dev")
endif()

# ==============================================
# 2. 查找其他依赖
# ==============================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  sensor_msgs
  nav_msgs
  mars_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  message_generation
  std_srvs
  sonar_return_home_file_detector
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(OpenCV QUIET)

# ==============================================
# 3. 包含目录（统一在这里）
# ==============================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加定义
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++17)

# ==============================================
# 2. 服务和消息配置（在find_package之后添加）
# ==============================================
## 声明服务文件
add_service_files(
  FILES
  StartPhase1.srv
  MoveStraight.srv
  StartPhaseMulti.srv
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
)


# catkin_package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    tf 
    tf2
    tf2_ros
    sensor_msgs 
    nav_msgs 
    mars_msgs
    visualization_msgs
    message_runtime
    std_srvs
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    sonar_return_home_file_detector
  INCLUDE_DIRS 
    include
  LIBRARIES
    sonar_slam
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 添加PCL和OpenCV包含目录
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
endif()

if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# 添加定义
add_definitions(${Eigen3_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++11)

# 链接目录
if(PCL_FOUND)
  link_directories(${PCL_LIBRARY_DIRS})
endif()

# ==============================================
# 检查源文件是否存在
# ==============================================
 
set(SONAR_DATA_PROCESSOR_SRC src/sonar_data_processor.cpp)
set(STATIC_MAPPER_SRC src/static_mapper.cpp)
set(SONAR_SLAM_NODE_SRC src/sonar_slam_node.cpp)
set(SONAR_SLAM_MAIN_SRC src/sonar_slam_main.cpp)
set(SONAR_TO_LASERSCAN_SRC src/sonar_to_laserscan.cpp)

# 新增实时映射器
set(SONAR_REALTIME_MAPPER_SRC src/sonar_realtime_mapper.cpp)

set(PHASE_FUSION_SONAR_MAPPER_SRC src/phase_fusion_sonar_mapper.cpp)

set(INTELLIGENT_ALIGNER_SRC  src/intelligent_submap_aligner.cpp)
set(SONAR_AWARE_ALIGNER_SRC  src/sonar_aware_aligner.cpp)

set(MOVE_CONTROLLER_SRC  src/move_controller.cpp)
set(LINEAR_DUAL_SONAR_MAPPER_SRC  src/linear_dual_sonar_mapper.cpp)

set(MULTI_POSITION_SONAR_MAPPER_SRC  src/multi_position_sonar_mapper.cpp)

# 手动点位声呐采集节点（终端1手动开车 + 服务触发逐点采集）
set(MANUAL_POINT_COLLECTOR_SRC  src/manual_point_collector.cpp)

# 新增 AsyncSerial.cpp
set(ASYNC_SERIAL_SRC src/AsyncSerial.cpp)

 
set(ROBOT_WALKER_SRC src/robot_walker.cpp)

set(ROBOT_WALL_CLEANER_SRC src/robot_wall_cleaner.cpp)

set(ADAPTIVE_RECTANGLE_WALKER_SRC src/adaptive_rectangle_walker.cpp)

set(BOW_COVERAGE_WALKER_SRC src/bow_coverage_walker.cpp)

set(WALL_FOLLOWER_SRC src/wall_follower.cpp)

# ==============================================
# 新增：测试数据生成器
# ==============================================
set(TEST_DATA_GENERATOR_SRC src/test_data_generator.cpp)
 

# ==============================================
# 2. 声纳数据处理器 - sonar_data_processor
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SONAR_DATA_PROCESSOR_SRC})
  message(STATUS "Found sonar_data_processor.cpp")
  
  add_executable(sonar_data_processor
    ${SONAR_DATA_PROCESSOR_SRC}
  )
  
  add_dependencies(sonar_data_processor ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(sonar_data_processor
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  message(STATUS "Added executable: sonar_data_processor")
endif()

# ==============================================
# 3. 静态建图器（实时）- static_mapper
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${STATIC_MAPPER_SRC})
  message(STATUS "Found static_mapper.cpp")
  
  add_executable(static_mapper
    ${STATIC_MAPPER_SRC}
  )
  
  add_dependencies(static_mapper ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(static_mapper
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  message(STATUS "Added executable: static_mapper")
endif()

# ==============================================
# 3. 扫描匹配器库 - 先于主节点编译
# ==============================================

# 扫描匹配器源文件
set(SCAN_MATCHER_SRC
  src/scan_matcher.cpp
)

# 检查扫描匹配器源文件是否存在
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_matcher.cpp)
  message(STATUS "Found scan_matcher.cpp, building scan matcher library")
  
  # 添加扫描匹配器库
  add_library(sonar_scan_matcher
    ${SCAN_MATCHER_SRC}
  )
  
  # 设置库的属性
  set_target_properties(sonar_scan_matcher PROPERTIES
    OUTPUT_NAME "sonar_scan_matcher"
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${PROJECT_VERSION_MAJOR}"
  )
  
  # 添加依赖
  add_dependencies(sonar_scan_matcher ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(sonar_scan_matcher
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(sonar_scan_matcher PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  
  message(STATUS "Added library: sonar_scan_matcher")
else()
  message(WARNING "scan_matcher.cpp not found, scan matching will not be available")
endif()

 

# ==============================================
# 7. 实时声纳映射器 - sonar_realtime_mapper
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SONAR_REALTIME_MAPPER_SRC})
  message(STATUS "Found sonar_realtime_mapper.cpp")
  
  add_executable(sonar_realtime_mapper
    ${SONAR_REALTIME_MAPPER_SRC}
    ${ASYNC_SERIAL_SRC}
  )
  

  add_dependencies(sonar_realtime_mapper 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp   
  )
  target_link_libraries(sonar_realtime_mapper
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  if(PCL_FOUND)
    target_link_libraries(sonar_realtime_mapper ${PCL_LIBRARIES})
  endif()
  
  # 移除 C++11 标志冲突，使用 C++17
  set_target_properties(sonar_realtime_mapper PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
  )
  
  message(STATUS "Added executable: sonar_realtime_mapper")
endif()


# ==============================================
# 7. 实时声纳映射器 - phase_fusion_sonar_mapper
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PHASE_FUSION_SONAR_MAPPER_SRC})
  message(STATUS "Found phase_fusion_sonar_mapper.cpp")
  
  # 创建新变量包含所有需要的源文件
  set(PHASE_FUSION_ALL_SRCS
    ${PHASE_FUSION_SONAR_MAPPER_SRC}
    ${ASYNC_SERIAL_SRC}
    ${INTELLIGENT_ALIGNER_SRC}
    ${SONAR_AWARE_ALIGNER_SRC}
  )
  
  add_executable(phase_fusion_sonar_mapper
    ${PHASE_FUSION_ALL_SRCS}
  )
  
  add_dependencies(phase_fusion_sonar_mapper 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp   
  )
  
  target_link_libraries(phase_fusion_sonar_mapper
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  if(PCL_FOUND)
    target_link_libraries(phase_fusion_sonar_mapper ${PCL_LIBRARIES})
    target_include_directories(phase_fusion_sonar_mapper PRIVATE ${PCL_INCLUDE_DIRS})
  endif()
  
  # 添加Eigen3头文件目录
  target_include_directories(phase_fusion_sonar_mapper PRIVATE ${EIGEN3_INCLUDE_DIR})
  
  # 设置C++标准
  set_target_properties(phase_fusion_sonar_mapper PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  
  message(STATUS "Added executable: phase_fusion_sonar_mapper")
endif()


# ==============================================
# 8. 实时声纳映射器 - linear_dual_sonar_mapper
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${LINEAR_DUAL_SONAR_MAPPER_SRC})
  message(STATUS "Found linear_dual_sonar_mapper.cpp")
  
  # 创建新变量包含所有需要的源文件 
  set(PHASE_FUSION_ALL_SRCS
    ${LINEAR_DUAL_SONAR_MAPPER_SRC}
    ${ASYNC_SERIAL_SRC}
    ${INTELLIGENT_ALIGNER_SRC}
    ${SONAR_AWARE_ALIGNER_SRC}
 
  )
  
  add_executable(linear_dual_sonar_mapper
    ${PHASE_FUSION_ALL_SRCS}
  )
  
  add_dependencies(linear_dual_sonar_mapper 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp   
  )
  
  target_link_libraries(linear_dual_sonar_mapper
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  if(PCL_FOUND)
    target_link_libraries(linear_dual_sonar_mapper ${PCL_LIBRARIES})
    target_include_directories(linear_dual_sonar_mapper PRIVATE ${PCL_INCLUDE_DIRS})
  endif()
  
  # 添加Eigen3头文件目录
  target_include_directories(linear_dual_sonar_mapper PRIVATE ${EIGEN3_INCLUDE_DIR})
  
  # 设置C++标准
  set_target_properties(linear_dual_sonar_mapper PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  
  message(STATUS "Added executable: linear_dual_sonar_mapper")
endif()



# ==============================================
# 8.1 实时声纳映射器 - multi_position_sonar_mapper
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MULTI_POSITION_SONAR_MAPPER_SRC})
  message(STATUS "Found multi_position_sonar_mapper.cpp")
  
  # 创建新变量包含所有需要的源文件
  set(PHASE_FUSION_ALL_SRCS
    ${MULTI_POSITION_SONAR_MAPPER_SRC}
    ${ASYNC_SERIAL_SRC}
    ${INTELLIGENT_ALIGNER_SRC}
    ${SONAR_AWARE_ALIGNER_SRC}
  )
  
  add_executable(multi_position_sonar_mapper
    ${PHASE_FUSION_ALL_SRCS}
  )
  
  add_dependencies(multi_position_sonar_mapper 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp   
  )
  
  target_link_libraries(multi_position_sonar_mapper
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  if(PCL_FOUND)
    target_link_libraries(multi_position_sonar_mapper ${PCL_LIBRARIES})
    target_include_directories(multi_position_sonar_mapper PRIVATE ${PCL_INCLUDE_DIRS})
  endif()
  
  # 添加Eigen3头文件目录
  target_include_directories(multi_position_sonar_mapper PRIVATE ${EIGEN3_INCLUDE_DIR})
  
  # 设置C++标准
  set_target_properties(multi_position_sonar_mapper PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  
  message(STATUS "Added executable: multi_position_sonar_mapper")
endif()


# ==============================================
# 8.2 手动点位声呐采集节点 - manual_point_collector
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MANUAL_POINT_COLLECTOR_SRC})
  message(STATUS "Found manual_point_collector.cpp")

  add_executable(manual_point_collector
    ${MANUAL_POINT_COLLECTOR_SRC}
    ${ASYNC_SERIAL_SRC}
  )

  add_dependencies(manual_point_collector
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
  )

  target_link_libraries(manual_point_collector
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )

  if(PCL_FOUND)
    target_link_libraries(manual_point_collector ${PCL_LIBRARIES})
    target_include_directories(manual_point_collector PRIVATE ${PCL_INCLUDE_DIRS})
  endif()

  target_include_directories(manual_point_collector PRIVATE ${EIGEN3_INCLUDE_DIR})

  set_target_properties(manual_point_collector PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )

  message(STATUS "Added executable: manual_point_collector")
endif()


# ==============================================
# 9. 独立移动控制节点 - move_controller_node
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MOVE_CONTROLLER_SRC})
  message(STATUS "Found move_controller.cpp for standalone node")
  
  # 创建move_controller_node可执行文件
  add_executable(move_controller_node
    ${MOVE_CONTROLLER_SRC}
  )
  
  add_dependencies(move_controller_node 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp   
  )
  
  target_link_libraries(move_controller_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen3_LIBRARIES}
  )
  
  if(PCL_FOUND)
    target_link_libraries(move_controller_node ${PCL_LIBRARIES})
    target_include_directories(move_controller_node PRIVATE ${PCL_INCLUDE_DIRS})
  endif()
  
  # 设置C++标准
  set_target_properties(move_controller_node PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  
  message(STATUS "Added executable: move_controller_node")
endif()



# ==============================================
# 11. - robot_walker
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_WALKER_SRC})
  message(STATUS "Found robot_walker.cpp")
  
  add_executable(robot_walker
    ${ROBOT_WALKER_SRC}
  )
  
  # 依赖关系
  add_dependencies(robot_walker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(robot_walker
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(robot_walker PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # 安装配置（可选）
  install(TARGETS robot_walker
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  message(STATUS "Added executable: robot_walker")
else()
  message(WARNING "robot_walker.cpp not found at ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_WALKER_SRC}, skipping")
endif()


# ==============================================
# 11. - robot_wall_cleaner
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_WALL_CLEANER_SRC})
  message(STATUS "Found robot_wall_cleaner.cpp")
  
  add_executable(robot_wall_cleaner
    ${ROBOT_WALL_CLEANER_SRC}
  )
  
  # 依赖关系
  add_dependencies(robot_wall_cleaner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(robot_wall_cleaner
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(robot_wall_cleaner PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # 安装配置（可选）
  install(TARGETS robot_wall_cleaner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  message(STATUS "Added executable: robot_wall_cleaner")
else()
  message(WARNING "robot_wall_cleaner.cpp not found at ${CMAKE_CURRENT_SOURCE_DIR}/${ROBOT_WALL_CLEANER_SRC}, skipping")
endif()


# ==============================================
# 11. - bow_coverage_walker
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${BOW_COVERAGE_WALKER_SRC})
  message(STATUS "Found bow_coverage_walker.cpp")
  
  add_executable(bow_coverage_walker
    ${BOW_COVERAGE_WALKER_SRC}
  )
  
  # 依赖关系
  add_dependencies(bow_coverage_walker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(bow_coverage_walker
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(bow_coverage_walker PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # 安装配置（可选）
  install(TARGETS bow_coverage_walker
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  message(STATUS "Added executable: bow_coverage_walker")
else()
  message(WARNING "bow_coverage_walker.cpp not found at ${CMAKE_CURRENT_SOURCE_DIR}/${BOW_COVERAGE_WALKER_SRC}, skipping")
endif()



# ==============================================
# 11. - adaptive_rectangle_walker
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${ADAPTIVE_RECTANGLE_WALKER_SRC})
  message(STATUS "Found adaptive_rectangle_walker.cpp")
  
  add_executable(adaptive_rectangle_walker
    ${ADAPTIVE_RECTANGLE_WALKER_SRC}
  )
  
  # 依赖关系
  add_dependencies(adaptive_rectangle_walker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(adaptive_rectangle_walker
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(adaptive_rectangle_walker PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # 安装配置（可选）
  install(TARGETS adaptive_rectangle_walker
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  message(STATUS "Added executable: adaptive_rectangle_walker")
else()
  message(WARNING "adaptive_rectangle_walker.cpp not found at ${CMAKE_CURRENT_SOURCE_DIR}/${ADAPTIVE_RECTANGLE_WALKER_SRC}, skipping")
endif()


# ==============================================
# 13. - set(WALL_FOLLOWER_SRC src/wall_follower.cpp)
# ==============================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${WALL_FOLLOWER_SRC})
  message(STATUS "Found wall_follower.cpp")
  
  add_executable(wall_follower
    ${WALL_FOLLOWER_SRC}
  )
  
  # 依赖关系
  add_dependencies(wall_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # 链接库
  target_link_libraries(wall_follower
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  
  # 包含目录
  target_include_directories(wall_follower PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  
  # 安装配置（可选）
  install(TARGETS wall_follower
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  message(STATUS "Added executable: wall_follower")
else()
  message(WARNING "wall_follower.cpp not found at ${CMAKE_CURRENT_SOURCE_DIR}/${WALL_FOLLOWER_SRC}, skipping")
endif()


# ==============================================
# 测试程序 - test_detector（回环检测函数测试）
# ==============================================
add_executable(test_detector
  src/test_detector.cpp
)

add_dependencies(test_detector 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp   
)

target_link_libraries(test_detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_target_properties(test_detector PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

message(STATUS "Added executable: test_detector")
 

# ==============================================
# 安装目标
# ==============================================
# 安装可执行文件
 
# ==============================================
# 安装目标
# ==============================================
if(TARGET sonar_realtime_mapper)
  install(TARGETS sonar_realtime_mapper
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

if(TARGET multi_position_sonar_mapper)
  install(TARGETS multi_position_sonar_mapper
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# ==============================================
# 安装其他文件
# ==============================================

# 安装头文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/sonar_slam)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
  message(STATUS "Installing headers from include/sonar_slam/")
endif()

# 安装启动文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
  message(STATUS "Installing launch files")
endif()

# 安装配置文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )
  message(STATUS "Installing config files")
endif()

# 安装数据文件
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/data)
  install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )
  message(STATUS "Installing data files")
endif()

# ==============================================
# 编译选项
# ==============================================
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG -g -O0)
  message(STATUS "Building in Debug mode")
else()
  add_definitions(-O2)
  message(STATUS "Building in Release mode")
endif()

# ==============================================
# 输出总结
# ==============================================
message(STATUS "======================================")
message(STATUS "Sonar SLAM Build Configuration")
message(STATUS "======================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Found dependencies:")
message(STATUS "  - Eigen3: ${Eigen3_FOUND}")
message(STATUS "  - Boost: ${Boost_FOUND}")
message(STATUS "  - PCL: ${PCL_FOUND}")
message(STATUS "  - OpenCV: ${OpenCV_FOUND}")
message(STATUS "Executables to build:")
 
if(TARGET sonar_realtime_mapper)
  message(STATUS "  ✓ sonar_realtime_mapper")
endif()

if(TARGET multi_position_sonar_mapper)
  message(STATUS "  ✓ multi_position_sonar_mapper")
endif()


if(TARGET phase_fusion_sonar_mapper)
  message(STATUS "  ✓ phase_fusion_sonar_mapper")
endif()

if(TARGET sonar_to_laserscan)
  message(STATUS "  ✓ sonar_to_laserscan (library)")
endif()
message(STATUS "======================================")