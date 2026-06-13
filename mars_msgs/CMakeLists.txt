cmake_minimum_required(VERSION 3.0.2)
project(mars_msgs)

set(CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
)


add_message_files(
  FILES
  
  TaskState.msg
  MarsStates.msg
  MarsError.msg
  HeartBeats.msg
  BatteryDatas.msg

  Pose.msg
  Twist.msg
  Odom.msg
  MotionStates.msg
  
  WheelActionState.msg
  ActorActionState.msg
  AvoidActionState.msg
  AudioActionState.msg
  SignalActionState.msg
  LightingActionState.msg
  DeviceOnoff.msg
  DeviceAction.msg
  DeviceStates.msg

  MotionActionState.msg
  LocationActionState.msg
  MappingActionState.msg
  QrcodeActionState.msg
  AsrActionState.msg
  ModuleOnoff.msg
  ModuleAction.msg
  ModuleStates.msg
  MapImgData.msg
  Infrared.msg

  Point.msg
  Path.msg
  GlobalPath.msg

  Pose2DArray.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS  roscpp rospy  std_msgs geometry_msgs   message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)