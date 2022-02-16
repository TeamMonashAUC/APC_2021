cmake_minimum_required(VERSION 3.0.2)
project(shell_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS roscpp
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(car_control
#    src/car_control.cpp
#)
#add_executable(goals
#    src/goals.cpp
#)
#add_executable(timer
#    src/timer.cpp
#)
#add_executable(efficiency
#    src/efficiency.cpp
#)

#target_link_libraries(car_control
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(goals
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(timer
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(efficiency
#  ${catkin_LIBRARIES}
#)

add_executable(control_master
    src/control_master.cpp
)

target_link_libraries(control_master
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

#install(TARGETS car_control
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(TARGETS goals
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(TARGETS timer
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(TARGETS efficiency
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS control_master
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
