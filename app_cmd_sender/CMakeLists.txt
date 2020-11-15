cmake_minimum_required(VERSION 2.8.3)
project(app_cmd_sender)

FIND_PACKAGE(OpenCV REQUIRED)
find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_can_msgs
  autoware_msgs
  roscpp
  std_msgs
  tablet_socket_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS} -pthread -std=c++11")

catkin_package()

include_directories(
  nodes/mnn/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(nodes/mnn/lib)

add_executable(app_cmd_sender nodes/app_cmd_sender.cpp)
target_link_libraries(app_cmd_sender usbcan MNN ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(app_cmd_sender ${catkin_EXPORTED_TARGETS})

install(TARGETS app_cmd_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)