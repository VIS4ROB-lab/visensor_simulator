#ifndef VISENSOR_SIMULATOR_ROS_BACKEND_H
#define VISENSOR_SIMULATOR_ROS_BACKEND_H


#include <fstream>
#include <list>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

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
  RosBackendNode(BuiltInPlanner *planner);
  ~RosBackendNode();

  enum SimulationState{
    WAINTING_PLANNER,
    RECORDING,
    FINISHED
  };

  void run(const std::string &project_folder);

private:




  //ROS publishers and subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber imu_sub_;
  ros::Subscriber reference_odometry_sub_;


  Logger* logger_;
  BuiltInPlanner *planner_;

  bool use_builtin_planner_;

  //Callback functions
  void referenceOdometryCallback(const nav_msgs::Odometry& odometry_msg);
  void updateBuiltInPlanner(const nav_msgs::Odometry &odometry_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  //void goNextPose();
  void setState(SimulationState state);
  SimulationState simulation_state_;
  std::string output_folder_path_;
};


#endif //VISENSOR_SIMULATOR_ROS_BACKEND_H
