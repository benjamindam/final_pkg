cmake_minimum_required(VERSION 2.8.3)
project(final_pkg)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

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

install(TARGETS
  ${PROJECT_NAME}_battery
  ${PROJECT_NAME}_dock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)