cmake_minimum_required(VERSION 2.8.3)
project(linefollowertest)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs nav_msgs)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(FILES pos.msg docking.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(detect src/detect.cpp src/linedetect.cpp)
target_link_libraries(detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(detect linefollowertest_generate_messages_cpp)

add_executable(navig src/motion_node.cpp src/turtlebot.cpp)
target_link_libraries(navig ${catkin_LIBRARIES})
add_dependencies(navig linefollowertest_generate_messages_cpp)

