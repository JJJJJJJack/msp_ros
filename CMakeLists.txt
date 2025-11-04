cmake_minimum_required(VERSION 2.8.3)
project(msp_ros)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

## Find catkin macros and libraries
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  message_generation)

find_package(ASIO REQUIRED)
add_definitions(-DASIO_STANDALONE)
add_definitions(-DASIO_HAS_STD_ADDRESSOF)
add_definitions(-DASIO_HAS_STD_ARRAY)
add_definitions(-DASIO_HAS_CSTDINT)
add_definitions(-DASIO_HAS_STD_SHARED_PTR)
add_definitions(-DASIO_HAS_STD_TYPE_TRAITS)

find_package(Threads)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_to_sbus
  CATKIN_DEPENDS #other_catkin_pkg
  message_runtime  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(MSP_SOURCE_DIR src)
set(MSP_INCLUDE_DIR inc/msp)
include_directories(${MSP_INCLUDE_DIR})

################################################################################
### libraries

# client library
add_library(mspclient src/Client.cpp src/PeriodicTimer.cpp)
target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT} ASIO::ASIO)

## Declare a cpp executable
add_executable(msp_ros 
  src/msp_ros.cpp
  )


## Specify libraries to link a library or executable target against
target_link_libraries(msp_ros
  ${catkin_LIBRARIES}
  mspclient
  )
