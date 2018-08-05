#ifndef VISENSOR_SIMULATOR_BUILDIN_PLANNER_H
#define VISENSOR_SIMULATOR_BUILDIN_PLANNER_H

#include <nav_msgs/Odometry.h>

class BuiltInPlanner{
public:
  enum PlannerStatus
  {
    INVALID,
    STARTING,
    RUNNING,
    COMPLETED
  };

  virtual PlannerStatus step( const nav_msgs::Odometry& curr_odometry)=0;
  virtual bool loadConfigurationFromFile(const std::string& filepath )=0;
  //virtual bool getNextWaypoint( Eigen::Vector3d &desired_position, double &desired_yaw, float &desired_gimbal_pitch)=0;
};

#endif //VISENSOR_SIMULATOR_BUILDIN_PLANNER_H
