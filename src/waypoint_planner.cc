
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

using namespace ros;

const float kDEG_2_RAD = M_PI / 180.0;

struct Waypoint{
    double x;
    double y;
    double z;
    double yaw;
    Waypoint(double x_p, double y_p, double z_p, double yaw_p):x(x_p),y(y_p),z(z_p),yaw(yaw_p){}
};

class WaypointPlanner{
public:
  WaypointPlanner(double yaw_max_error, double position_max_error);
  void run();
  std::list<Waypoint> waypoints_;
  ~WaypointPlanner(){}
private:
  //ROS publishers and subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber pose_command_sub_;
  ros::Subscriber reference_odometry_sub_;

  ros::Publisher pose_command_pub_;  

  ros::Rate loop_rate_;

  double yaw_max_error_;
  double position_max_error_squared_;

  //Callback functions
  void referenceOdometryCallback(const nav_msgs::Odometry& odometry_msg);
};

WaypointPlanner::WaypointPlanner(double yaw_max_error, double position_max_error):
  nh_private_("~"),
  yaw_max_error_(yaw_max_error),
  position_max_error_squared_(position_max_error*position_max_error),
  loop_rate_(10)
{
  reference_odometry_sub_ =  nh_.subscribe("reference_odometry_topic", 10, &WaypointPlanner::referenceOdometryCallback, this);
  pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);  
}

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

void WaypointPlanner::referenceOdometryCallback(const nav_msgs::Odometry &odometry_msg){

    const  geometry_msgs::Point& curr_position = odometry_msg.pose.pose.position;
    double yaw = quat2yaw(odometry_msg.pose.pose.orientation);
    Waypoint & curr_waypoint = waypoints_.front();

    double position_error_squared = squared_dist(curr_position,curr_waypoint);
    const static double c_2pi = 2*M_PI;
    double yaw_error = std::fmod((std::abs(curr_waypoint.yaw - yaw)+c_2pi),c_2pi);

    if(position_error_squared < position_max_error_squared_ &&  yaw_error < yaw_max_error_)
    {
        waypoints_.push_back(waypoints_.front());
        waypoints_.pop_front();
        Waypoint & next_waypoint = waypoints_.front();
        ROS_WARN("publishing NEW waypoint: %lf :: %lf :: %lf :: %lf",next_waypoint.x, next_waypoint.y, next_waypoint.z,next_waypoint.yaw/kDEG_2_RAD);
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
          trajectory_msg.header.stamp = ros::Time::now();
          Eigen::Vector3d desired_position(next_waypoint.x, next_waypoint.y, next_waypoint.z);
          double desired_yaw = next_waypoint.yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
              desired_yaw, &trajectory_msg);
        pose_command_pub_.publish(trajectory_msg);
    }else
      ROS_INFO_STREAM("squared_dist to destination " << position_error_squared << " :  yaw diff" << yaw_error);

    return;
}

void WaypointPlanner::run()
{


    while(nh_.ok()){
      ros::spinOnce();
      loop_rate_.sleep();
    }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_planner");

  ROS_INFO("waypoint_planner started");

  WaypointPlanner planner(0.1,1.0);

  

  ros::NodeHandle nh;
  std::string filename_waypoints;
  if (nh.getParam("/filename_waypoints", filename_waypoints)) {
    ROS_INFO_STREAM("Loading waypoints from: " << filename_waypoints);
    std::ifstream file(filename_waypoints);
    double x, y, z, yaw;
    char eater;//eats commas
    while (file >> x >> eater >> y >> eater >> z >> eater >> yaw) {
      planner.waypoints_.push_back(Waypoint(x, y, z, yaw*kDEG_2_RAD));
      if (file.eof()) {
        break;
      }
    }
    ROS_INFO_STREAM("Loaded " << planner.waypoints_.size() << " waypoints from file.");
  } else {
    planner.waypoints_.push_back(Waypoint(1,-50,2,0));
    planner.waypoints_.push_back(Waypoint(1,50,2,0));
    planner.waypoints_.push_back(Waypoint(2,50,2,0));
    planner.waypoints_.push_back(Waypoint(2,-50,2,0));
    planner.waypoints_.push_back(Waypoint(1,-50,2,0));
    ROS_INFO_STREAM("Loaded " << planner.waypoints_.size() << " default waypoints.");
  }

  planner.run();

  ROS_INFO("shutting down waypoint_planner");

  return 0;
}
