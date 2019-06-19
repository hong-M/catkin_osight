cmake_minimum_required(VERSION 2.8.3)
project(sexxx)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(SExxx src/SExxx.cpp)
target_link_libraries(SExxx ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(SExxx_node src/SExxx_node.cpp)
target_link_libraries(SExxx_node SExxx ${catkin_LIBRARIES})


install(TARGETS SExxx SExxx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(PROGRAMS scripts/osight_set_IP scripts/osight_set_net_mode
 # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#if (CATKIN_ENABLE_TESTING)
 # catkin_add_gtest(test_buffer test/test_buffer.cpp)
 # target_link_libraries(test_buffer ${catkin_LIBRARIES})

#  find_package(roslint REQUIRED)
 # roslint_cpp()
  #roslint_add_test()

  #find_package(roslaunch REQUIRED)
 # roslaunch_add_file_check(launch/SExxx.launch)
#endif()
