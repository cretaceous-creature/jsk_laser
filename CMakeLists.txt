cmake_minimum_required(VERSION 2.8.3)
project(jsk_laser)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation tf)
find_package(Boost REQUIRED COMPONENTS thread system signals)

add_message_files(FILES JskLaser.msg JskLaserRaw.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES jsk_stm
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime tf
  DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library (serial_comm src/serial_comm.cpp)
add_dependencies(serial_comm ${PROJECT_NAME}_gencpp)
target_link_libraries(serial_comm  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(jsk_laser_node  src/jsk_laser_node.cpp)
add_dependencies(jsk_laser_node ${PROJECT_NAME}_gencpp)
target_link_libraries(jsk_laser_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES} serial_comm)

install(TARGETS jsk_laser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
