cmake_minimum_required(VERSION 3.1.3)
project(manipulation_rubik)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    franka_gripper
    std_msgs
    message_generation
    genmsg
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV)
find_package(Boost REQUIRED system filesystem date_time thread)
#find_package(Franka REQUIRED)

#add_library(libfranka)

#target_link_libraries(libfranka Franka::Franka)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

add_message_files(
  FILES
  MoveConfiguration.msg
  CubeDto.msg
)


add_service_files(
  FILES
  LfMoveLeft.srv
  ResolveConfiguration.srv
  RotateCube.srv
  RubikFace.srv
  RubikCubies.srv
  RubikFaceDetect.srv
  PrepareForRotation.srv
  Rotate.srv
)



generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    message_runtime
  DEPENDS
    EIGEN3
)






###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_subdirectory(doc/rubik_right)
add_subdirectory(doc/brain)
add_subdirectory(doc/logic)
add_subdirectory(doc/eyes)


