cmake_minimum_required(VERSION 2.8.3)
project(hexacopter_class)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hexacopter_class src/main.cpp src/hexacopter.cpp)
target_link_libraries(hexacopter_class ${catkin_LIBRARIES})
add_dependencies(hexacopter_class hexacopter_class_generate_messages_cpp)

install(
	TARGETS hexacopter_class
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
