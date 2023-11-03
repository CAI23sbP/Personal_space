cmake_minimum_required(VERSION 3.0.2)
project(gaussian_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  pedsim_msgs
  nav_msgs
  message_filters 
)
add_message_files(
  FILES
  SingleGaussian.msg
  MultiGaussians.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs#   sensor_msgs#   
)

catkin_package(
#  INCLUDE_DIRS include
# EXPORTED_TARGETS gaussian_generator
# LIBRARIES gaussian_generator
 CATKIN_DEPENDS message_runtime geometry_msgs roscpp sensor_msgs std_msgs  pedsim_msgs nav_msgs
#  DEPENDS message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  gaussian_generator
  src/gaussian_generator.cpp
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/gaussian_generator.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
