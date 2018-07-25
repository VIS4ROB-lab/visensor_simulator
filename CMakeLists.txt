cmake_minimum_required(VERSION 2.8.3)
project(visensor_simulator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(ros_backend_node src/ros_backend_node.cc include/visensor_simulator/logger.h include/visensor_simulator/simple_waypoint_planner.h)

cs_add_library(poltergeist_gimbal_plugin src/gazebo_poltergeist_gimbal_plugin.cpp)
target_link_libraries(poltergeist_gimbal_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

cs_install()

cs_export()
