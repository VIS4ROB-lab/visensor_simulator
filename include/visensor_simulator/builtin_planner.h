#ifndef VISENSOR_SIMULATOR_BUILDIN_PLANNER_H
#define VISENSOR_SIMULATOR_BUILDIN_PLANNER_H

class BuiltInPlanner{
public:
 virtual bool step( const geometry_msgs::Pose& curr_pose)=0;
virtual bool getNextWaypoint( Eigen::Vector3d &desired_position, double &desired_yaw, float &desired_gimbal_pitch)=0;


};

#endif //VISENSOR_SIMULATOR_BUILDIN_PLANNER_H
