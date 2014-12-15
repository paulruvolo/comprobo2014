cmake_minimum_required(VERSION 2.8.3)
project(neato_node)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES Bump.msg)
generate_messages()

catkin_package(DEPENDS message_runtime)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

