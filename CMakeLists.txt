cmake_minimum_required(VERSION 2.8.3)
project(visensor_simulator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

catkin_simple()


cs_add_executable(ros_backend_node src/ros_backend_node.cc include/visensor_simulator/logger.h include/visensor_simulator/simple_waypoint_planner.h)


cs_install()

cs_export()
