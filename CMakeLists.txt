cmake_minimum_required(VERSION 3.0.2)
project(tracking_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Target.msg
  PositionCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracking_controller
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/trackingController.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)


add_executable(tracking_controller_node src/tracking_controller_node.cpp)

target_link_libraries(tracking_controller_node ${catkin_LIBRARIES} ${PROJECT_NAME})

catkin_install_python(PROGRAMS
  scripts/circle_test.py  # 替换为你的脚本路径
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/pid_analyzer.py  # 替换为你的脚本路径
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
  scripts/pid_vis.py  # 替换为你的脚本路径
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)