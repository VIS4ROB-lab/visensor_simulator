#include <ros/ros.h>
#include <visensor_simulator/ros_backend_node.h>


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

    RosBackendNode node;

    ROS_INFO("ros_backend_node started");

    //param use_embeed planner
    //param outputfolder

    node.run(project_folder);

    ROS_INFO("shutting down ros_backend_node");

    return 0;
}
