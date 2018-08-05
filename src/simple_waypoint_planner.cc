#include <fstream>
#include <visensor_simulator/simple_waypoint_planner.h>
#include <visensor_simulator/simple_waypoint_planner.h>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


static constexpr float kDEG_2_RAD = M_PI / 180.0;

static constexpr double kPlanning_Rate = 5;




/*
status
0: invalid state
1: too far from the initial waypoint
3: executing pathl
4: finished
*/


double quat2yaw(const geometry_msgs::Quaternion & q)
{
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny, cosy);
}

double squared_dist(const geometry_msgs::Point& curr_pos,Waypoint waypoint)
{
  double dx =  curr_pos.x-waypoint.x;
  double dy =  curr_pos.y-waypoint.y;
  double dz =  curr_pos.z-waypoint.z;
  return ((dx*dx)+(dy*dy)+(dz*dz));
}


SimpleWaypointPlanner::SimpleWaypointPlanner():SimpleWaypointPlanner(0.2,2){


}

SimpleWaypointPlanner::SimpleWaypointPlanner(double yaw_max_error, double position_max_error):is_valid_(false),yaw_max_error_(yaw_max_error),position_max_error_squared_(position_max_error*position_max_error){

  pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  gimbal_command_pub_ = nh_.advertise<sensor_msgs::Joy>("command/gimbal_actuators", 1);

  //sample trajectory
  waypoints_.push_back(Waypoint(1,-5,2,0,0));
  waypoints_.push_back(Waypoint(1,5,2,0,0));
  waypoints_.push_back(Waypoint(2,5,2,0,0));
  waypoints_.push_back(Waypoint(2,-5,2,0,0));
  waypoints_.push_back(Waypoint(1,-5,2,0,0));
  status_ = STARTING;
}

bool SimpleWaypointPlanner::loadConfigurationFromFile(const std::string &filepath )
{
  waypoints_.clear();
  status_ = INVALID;
  std::ifstream file(filepath);
  if(file.is_open()){

    double x, y, z, yaw ;
    float gimbal_pitch;
    char eater;//eats commas
    while (file >> x >> eater >> y >> eater >> z >> eater >> yaw >> eater >> gimbal_pitch) {
      waypoints_.push_back(Waypoint(x, y, z, yaw*kDEG_2_RAD,gimbal_pitch));
      if (file.eof()) {
        break;
      }
    }
    if(waypoints_.size() > 0)
    {
      status_ = STARTING;
      return true;
    }
  }
  return false;
}

bool SimpleWaypointPlanner::getNextWaypoint( Eigen::Vector3d &desired_position, double &desired_yaw, float &desired_gimbal_pitch)
{
  if(waypoints_.size() > 0)
  {
    Eigen::Vector3d position(waypoints_.front().x, waypoints_.front().y, waypoints_.front().z);
    desired_position = position;
    desired_yaw =  waypoints_.front().yaw;
    desired_gimbal_pitch = waypoints_.front().gimbal_pitch;
    return true;
  }
  return false;
}

void SimpleWaypointPlanner::goNextPose()
{

  static ros::Time last_time(0);
  if(ros::Time::now() - last_time > ros::Duration(ros::Rate(kPlanning_Rate)) ) {
    last_time = ros::Time::now();
  }else{
    return;
  }

  Eigen::Vector3d desired_position;
  double desired_yaw;
  float desired_gimbal_pitch;
  getNextWaypoint(desired_position,desired_yaw,desired_gimbal_pitch);
  sendPoseCommand(desired_position,desired_yaw,desired_gimbal_pitch);
}

BuiltInPlanner::PlannerStatus SimpleWaypointPlanner::step(const nav_msgs::Odometry &curr_odometry)
{

  if(status_ == STARTING){
    if(reachedNextWaypoint(curr_odometry.pose.pose)){
      status_ = RUNNING;//do not send waypoints yet, the logger need to be started
    }else{
      goNextPose();
    }
  }
  else if (status_ == RUNNING ) {
    {
      if(reachedNextWaypoint(curr_odometry.pose.pose)){
        if(waypoints_.size() > 0){
          waypoints_.pop_front();
        }
      }

      if(waypoints_.size() == 0)
      {
        status_ = COMPLETED;
      }else
      {
        goNextPose();
      }
    }
  }

  return status_;
}


bool SimpleWaypointPlanner::reachedNextWaypoint(const geometry_msgs::Pose& curr_pose)// do not consider the gimbal position
{
  if(waypoints_.size() == 0)
    return true;//nowhere to go

  double position_error_squared = squared_dist(curr_pose.position,waypoints_.front());
  double yaw = quat2yaw(curr_pose.orientation);
  const static double c_2pi = 2*M_PI;
  double yaw_error = std::fmod((std::abs(waypoints_.front().yaw - yaw)+c_2pi),c_2pi);
  //ROS_ERROR_STREAM("DEGUB error :" << position_error_squared << " "<< position_max_error_squared_ << " "<< yaw_error << " "<< yaw_max_error_);
  if(position_error_squared < position_max_error_squared_ &&  ((yaw_error < yaw_max_error_)|| (yaw_error > (c_2pi - yaw_max_error_))))
  {
    return true;
  }
  return false;
}

void SimpleWaypointPlanner::sendPoseCommand(const Eigen::Vector3d &desired_position,const double &desired_yaw,const float &desired_gimbal_pitch)
{
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
  ROS_INFO("Publishing NEW waypoint: %lf :: %lf :: %lf :: %lf :: %f",desired_position.x(), desired_position.y(), desired_position.z(),desired_yaw/kDEG_2_RAD,desired_gimbal_pitch);
  pose_command_pub_.publish(trajectory_msg);
  sensor_msgs::Joy gimbal_msg;
  gimbal_msg.axes= {0.0f,desired_gimbal_pitch,0.0f};
  gimbal_command_pub_.publish(gimbal_msg);
}


