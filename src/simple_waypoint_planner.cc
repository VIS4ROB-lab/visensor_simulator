#include <fstream>
#include <visensor_simulator/simple_waypoint_planner.h>


static constexpr float kDEG_2_RAD = M_PI / 180.0;




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


     SimpleWaypointPlanner::SimpleWaypointPlanner(double yaw_max_error, double position_max_error):is_valid_(false),yaw_max_error_(yaw_max_error),position_max_error_squared_(position_max_error*position_max_error){

        //sample trajectory
        waypoints_.push_back(Waypoint(1,-5,2,0,0));
        waypoints_.push_back(Waypoint(1,5,2,0,0));
        waypoints_.push_back(Waypoint(2,5,2,0,0));
        waypoints_.push_back(Waypoint(2,-5,2,0,0));
        waypoints_.push_back(Waypoint(1,-5,2,0,0));
        status_ = STARTING;
    }

    bool SimpleWaypointPlanner::loadWaypointsFromFile( std::string filepath )
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

    bool SimpleWaypointPlanner::step( const geometry_msgs::Pose& curr_pose)
    {
        if( status_ == FINISHED)
            return false;

        if(reachedNextWaypoint(curr_pose))
        {
            if(waypoints_.size() > 0)
                waypoints_.pop_front();

            if(waypoints_.size() > 0)
            {
                status_ = RUNNING;
                return true;
            }else
                status_ = FINISHED;
        }

        return false;
    }


    int SimpleWaypointPlanner::getStatus()
    {
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

