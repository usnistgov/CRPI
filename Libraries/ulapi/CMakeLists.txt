cmake_minimum_required(VERSION 2.8)

project(ulapi)

if(COMMAND catkin_package)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ulapi dl pthread
)

endif(COMMAND catkin_package)

add_subdirectory(lib)
add_subdirectory(bin)
