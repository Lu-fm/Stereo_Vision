cmake_minimum_required(VERSION 2.8.3)
project(seed_serial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  message_generation

)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  cartHorizon.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES seed_serial
   CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS roscpp serial
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

add_library(kinematics
include/seed_serial/kinematics.h
src/kinematics.cpp
src/arm_controller.cpp
)



add_executable(task src/task.cpp)
target_link_libraries(task kinematics ${catkin_LIBRARIES})

add_executable(kine src/kinematics.cpp)
target_link_libraries(task  ${catkin_LIBRARIES})

