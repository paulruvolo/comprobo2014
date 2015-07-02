cmake_minimum_required(VERSION 2.8.3)
project(neato_node)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp tf laser_geometry)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
add_message_files(FILES Bump.msg)
generate_messages()

add_executable(fix_scan_fast src/fix_scan_fast.cpp)
target_link_libraries(fix_scan_fast ${catkin_LIBRARIES})

catkin_package(DEPENDS message_runtime)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

