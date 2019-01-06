cmake_minimum_required(VERSION 2.8.3)

project(fssim_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        tf
        fsd_common_msgs
        fssim_common)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${rospy_INCLUDE_DIRS}
)

add_executable(fssim_interface_node src/fssim_interface_node.cpp)
target_link_libraries(fssim_interface_node ${catkin_LIBRARIES})