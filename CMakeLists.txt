cmake_minimum_required(VERSION 2.8.3)
project(final_pkg)

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


add_executable(detect src/detect.cpp)
target_link_libraries(detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(detect final_pkg_generate_messages_cpp)

add_executable(navig src/motion_node.cpp)
target_link_libraries(navig ${catkin_LIBRARIES})
add_dependencies(navig final_pkg_generate_messages_cpp)

add_executable(${PROJECT_NAME}_battery src/battery.cpp)
set_target_properties(${PROJECT_NAME}_battery PROPERTIES OUTPUT_NAME battery PREFIX "")
target_link_libraries(${PROJECT_NAME}_battery
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_dock src/dock.cpp)
set_target_properties(${PROJECT_NAME}_dock PROPERTIES OUTPUT_NAME dock PREFIX "")
target_link_libraries(${PROJECT_NAME}_dock
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_bumper src/bumper.cpp)
set_target_properties(${PROJECT_NAME}_bumper PROPERTIES OUTPUT_NAME bumper PREFIX "")
target_link_libraries(${PROJECT_NAME}_bumper
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_buttonlaunch src/buttonlaunch.cpp)
set_target_properties(${PROJECT_NAME}_buttonlaunch PROPERTIES OUTPUT_NAME buttonlaunch PREFIX "")
target_link_libraries(${PROJECT_NAME}_buttonlaunch
  ${catkin_LIBRARIES}
)