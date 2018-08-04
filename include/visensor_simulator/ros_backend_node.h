#ifndef VISENSOR_SIMULATOR_ROS_BACKEND_H
#define VISENSOR_SIMULATOR_ROS_BACKEND_H


#include <fstream>
#include <list>

#include <ros/ros.h>
#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>

#include <visensor_simulator/simple_waypoint_planner.h>

#include <boost/filesystem.hpp>

using namespace ros;

#define STRING_ENUM(value) case RosBackendNode::value: os << "Current state: " << #value; break;

class Logger;

class RosBackendNode{
public:
    RosBackendNode();
    void run(std::string project_folder);
    ~RosBackendNode();
    enum SimulationState{
        NoMission,
        GoingToStart,
        InStartPose,
        InitializationRotine,
        StartingRecording,
        InMission,
        StopingRecording
    };
private:


    //ROS publishers and subscribers
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_sub_;
    ros::Subscriber reference_odometry_sub_;
    ros::Publisher pose_command_pub_;
    ros::Publisher gimbal_command_pub_;

    Logger* logger_;
    SimpleWaypointPlanner simple_planner_;

    //Callback functions
    void referenceOdometryCallback(const nav_msgs::Odometry& odometry_msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void sendPoseCommand(const Eigen::Vector3d &desired_position, const double &desired_yaw,const float &desired_gimbal_pitch);
    void goNextPose();

    void setState(SimulationState state);
    SimulationState state_;
};


#endif //VISENSOR_SIMULATOR_ROS_BACKEND_H
