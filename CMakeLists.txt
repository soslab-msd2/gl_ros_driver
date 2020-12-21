cmake_minimum_required(VERSION 2.8.3)
project(gl_ros_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(NOT TARGET serial)
file(GLOB serial_src "include/serial/*.cc")
add_library(serial SHARED ${serial_src})
endif()

if(NOT TARGET gl_driver)
add_library(gl_driver
  include/gl_driver.cpp
)
endif()


add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  gl_driver
  serial
)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
