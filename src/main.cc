#include <ros/ros.h>
#include <visensor_simulator/ros_backend_node.h>
#include <visensor_simulator/simple_waypoint_planner.h>


using namespace ros;


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        printf("The project folder argument is missing.");
        return 0;
    }

    ros::init(argc, argv, "ros_backend_node");



    std::string project_folder(argv[1]);

    SimpleWaypointPlanner * planner = new SimpleWaypointPlanner();

    ROS_INFO("ros_backend_node started");

    RosBackendNode node(planner);

    node.run(project_folder);

    delete planner;



    ROS_INFO("shutting down ros_backend_node");

    return 0;
}
