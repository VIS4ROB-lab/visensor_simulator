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

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})


cs_add_library(visensor_sim_backend src/ros_backend_node.cc )

cs_add_executable(simple_planner_backend src/simple_planner_backend.cc src/simple_waypoint_planner.cc)
target_link_libraries(simple_planner_backend ${catkin_LIBRARIES} visensor_sim_backend)

cs_add_executable(extern_planner_backend src/extern_planner_backend.cc )
target_link_libraries(extern_planner_backend ${catkin_LIBRARIES} visensor_sim_backend)

cs_add_library(poltergeist_gimbal_plugin src/gazebo_poltergeist_gimbal_plugin.cpp)
target_link_libraries(poltergeist_gimbal_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS scripts/visensor_sim_bagcreator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()

cs_export()
