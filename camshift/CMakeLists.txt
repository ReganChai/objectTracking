cmake_minimum_required(VERSION 2.8.3)
project(camshift)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  genmsg
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(publish src/publish.cpp)
target_link_libraries(publish ${catkin_LIBRARIES})
add_dependencies(publish ${catkin_EXPORTED_TARGETS} )

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
##add_dependencies(listener ${catkin_EXPORTED_TARGETS})
add_dependencies(listener topic_generate_messages_cpp)
