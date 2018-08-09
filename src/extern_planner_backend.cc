#include <ros/ros.h>
#include <visensor_simulator/ros_backend_node.h>
#include <visensor_simulator/simple_waypoint_planner.h>

using namespace ros;

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("The project folder argument is missing.");
    return 0;
  }

  ros::init(argc, argv, "extern_planner_backend_node");

  std::string project_folder(argv[1]);

  ROS_INFO("extern_planner_backend_node started");

  RosBackendNode node();

  node.run(project_folder);

  ROS_INFO("shutting down extern_planner_backend_node");

  return 0;
}
