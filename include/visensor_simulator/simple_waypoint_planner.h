
#ifndef VISENSOR_SIMULATOR_SIMPLE_WAYPOINT_PLANNER_H
#define VISENSOR_SIMULATOR_SIMPLE_WAYPOINT_PLANNER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <visensor_simulator/builtin_planner.h>



struct Waypoint{
  double x; //m
  double y; //m
  double z; //m
  double yaw; //rad
  float gimbal_pitch; //deg
  Waypoint(double x_p, double y_p, double z_p, double yaw_p, float gimbal_pitch_p):x(x_p),y(y_p),z(z_p),yaw(yaw_p),gimbal_pitch(gimbal_pitch_p){}
};


class SimpleWaypointPlanner: public BuiltInPlanner{
public:
  // error in 0.2rad(=11.5deg) and meters- these numbers need to be proportinal to the noise in the odometry used as input.
  SimpleWaypointPlanner();

  SimpleWaypointPlanner(double yaw_max_error, double position_max_error);
  ~SimpleWaypointPlanner(){}

  bool loadConfigurationFromFile(const std::string& filepath ) override;

  PlannerStatus step( const nav_msgs::Odometry& curr_odometry) override;

  //PlannerStatus getStatus() override;
  


private:
  bool getNextWaypoint( Eigen::Vector3d &desired_position, double &desired_yaw, float &desired_gimbal_pitch);
  bool reachedNextWaypoint(const geometry_msgs::Pose& curr_pose);// do not consider the gimbal position
  void sendPoseCommand(const Eigen::Vector3d &desired_position, const double &desired_yaw,const float &desired_gimbal_pitch);
  void goNextPose();

  bool is_valid_;
  double yaw_max_error_;
  double position_max_error_squared_;
  std::list<Waypoint> waypoints_;
  PlannerStatus status_;

  ros::NodeHandle nh_;

  ros::Publisher pose_command_pub_;
  ros::Publisher gimbal_command_pub_;
};


#endif
