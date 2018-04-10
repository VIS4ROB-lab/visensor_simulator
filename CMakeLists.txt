cmake_minimum_required(VERSION 2.8.3)
project(visensor_simulator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(waypoint_planner src/waypoint_planner.cc)
cs_add_executable(ros_backend_node src/ros_backend_node.cc)


cs_install()

cs_export()
