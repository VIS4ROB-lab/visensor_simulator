
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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>

using namespace ros;

const float kDEG_2_RAD = M_PI / 180.0;

struct Waypoint{
    double x;
    double y;
    double z;
    double yaw;
    Waypoint(double x_p, double y_p, double z_p, double yaw_p):x(x_p),y(y_p),z(z_p),yaw(yaw_p){}
};


struct ImuSensorReadings {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuSensorReadings()
        : gyroscopes(),
          accelerometers(),timestamp(0) {

    }

    ImuSensorReadings(ros::Time timestamp_, Eigen::Vector3d gyroscopes_,
                      Eigen::Vector3d accelerometers_)
        : gyroscopes(gyroscopes_),
          accelerometers(accelerometers_),timestamp(timestamp_) {
    }
    Eigen::Vector3d gyroscopes;     ///< Gyroscope measurement.
    Eigen::Vector3d accelerometers; ///< Accelerometer measurement.
    ros::Time timestamp;
};

struct PoseReadings {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    ros::Time timestamp;

};


std::ostream& operator<<(std::ostream& os, const PoseReadings& T)
{
    Eigen::Vector3d p_WS_W = T.position;
    Eigen::Quaterniond q_WS = T.orientation;
    os  << T.timestamp.toNSec() << "," <<  std::scientific
        << std::setprecision(18) << p_WS_W[0] << "," << p_WS_W[1] << ","
        << p_WS_W[2] << "," << q_WS.x() << "," << q_WS.y() << ","
        << q_WS.z() << "," << q_WS.w();
    return os;
}

std::ostream& operator<<(std::ostream& os, const ImuSensorReadings& T)
{
    Eigen::Vector3d acc = T.accelerometers;
    Eigen::Vector3d gyro = T.gyroscopes;
    os  << T.timestamp.toNSec() << "," <<  std::scientific
        << std::setprecision(18) << acc[0] << "," << acc[1] << ","
        << acc[2] << "," << gyro[0] << "," << gyro[1] << ","
        << gyro[2];
    return os;
}


class Logger{
public:
    Logger():is_valid_(false){}
    ~Logger(){stop();}//todo flush files

    bool startLogger( std::string output_folder )
    {
        if( !output_folder.empty() && output_folder.at(output_folder.length()-1) != '/' )
            output_folder+="/";

        std::string imu_filename = output_folder + "imu_data.csv";
        std::string pose_filename = output_folder + "pose_data.csv";
        ROS_INFO_STREAM(" " << imu_filename);
        imu_file.open(imu_filename);
        pose_file.open(pose_filename);

        if(imu_file.is_open() && pose_file.is_open())
        {
            is_valid_ = true;
            imu_file << "test" << std::endl;
        }
        else
        {
            is_valid_ = false;
            imu_file.close();
            pose_file.close();
        }

        imu_data.clear();
        imu_data.reserve(100000);
        pose_data.clear();
        pose_data.reserve(100000);

        return is_valid_;
    }

    bool logPose( const PoseReadings & measurement)
    {
        if(!is_valid_)
            return false;

        pose_data.push_back( measurement );
        return true;
    }

    bool logIMU( const ImuSensorReadings & measurement )
    {
        if(!is_valid_)
            return false;

        imu_data.push_back( measurement );
        return true;
    }

    void stop()
    {
        if(!is_valid_)
            return;

        serializeImu();
        serializePoses();

        is_valid_ = false;
        imu_file.close();
        pose_file.close();

    }

private:
    bool is_valid_;
    std::vector<ImuSensorReadings,Eigen::aligned_allocator<ImuSensorReadings>> imu_data;
    std::vector<PoseReadings,Eigen::aligned_allocator<PoseReadings>> pose_data;

    std::ofstream imu_file;
    std::ofstream pose_file;

    void serializeImu()
    {
        imu_file << "timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z" << std::endl;
        for (const auto& reading : imu_data)
            imu_file << reading << std::endl;
    }


    void serializePoses()
    {
        pose_file << "timestamp, p_x, p_y,p_z, q_x, q_y,q_z, q_w" << std::endl;
        for (const auto& reading : pose_data)
            pose_file << reading << std::endl;
    }



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
    ros::Subscriber imu_sub_;

    ros::Publisher pose_command_pub_;

    ros::Rate loop_rate_;

    Logger logger_;

    double yaw_max_error_;
    double position_max_error_squared_;

    //Callback functions
    void referenceOdometryCallback(const nav_msgs::Odometry& odometry_msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
};

WaypointPlanner::WaypointPlanner(double yaw_max_error, double position_max_error):
    nh_private_("~"),
    yaw_max_error_(yaw_max_error),
    position_max_error_squared_(position_max_error*position_max_error),
    loop_rate_(1)
{
    reference_odometry_sub_ =  nh_.subscribe("imu_frame_odometry_topic", 100, &WaypointPlanner::referenceOdometryCallback, this);
    imu_sub_ =  nh_.subscribe("imu_topic", 1000, &WaypointPlanner::imuCallback, this);

    pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
}


void WaypointPlanner::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    ImuSensorReadings imu_measurement;
    imu_measurement.accelerometers = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                                     msg->linear_acceleration.z);
    imu_measurement.gyroscopes = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                                                 msg->angular_velocity.z);
    imu_measurement.timestamp = msg->header.stamp;

    logger_.logIMU(imu_measurement);

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

    PoseReadings pose;
    pose.position = Eigen::Vector3d(curr_position.x,curr_position.y,curr_position.z);
    pose.orientation = Eigen::Quaterniond(odometry_msg.pose.pose.orientation.w,odometry_msg.pose.pose.orientation.x,odometry_msg.pose.pose.orientation.y,odometry_msg.pose.pose.orientation.z);
    pose.timestamp = odometry_msg.header.stamp;
    logger_.logPose(pose);

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
        ROS_INFO_STREAM("curr goal"<< curr_waypoint.x <<" "<< curr_waypoint.y <<" "<< curr_waypoint.z <<" " <<"\nsquared_dist to destination " << position_error_squared << " :  yaw diff" << yaw_error);

    return;
}

void WaypointPlanner::run()
{
    logger_.startLogger("/home/lucas/tmp/test_logger");
    while(nh_.ok()){
        ros::spinOnce();
        loop_rate_.sleep();
    }
    logger_.stop();
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_backend_node");

    ROS_INFO("ros_backend_node started");

    WaypointPlanner planner(0.1,1.0);

    //param use_embeed planner
    //param outputfolder

    ros::NodeHandle nh;
    std::string filename_waypoints;
    if (nh.getParam("filename_waypoints", filename_waypoints)) {
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
        planner.waypoints_.push_back(Waypoint(1,-5,2,0));
        planner.waypoints_.push_back(Waypoint(1,5,2,0));
        planner.waypoints_.push_back(Waypoint(2,5,2,0));
        planner.waypoints_.push_back(Waypoint(2,-5,2,0));
        planner.waypoints_.push_back(Waypoint(1,-5,2,0));
        ROS_INFO_STREAM("Loaded " << planner.waypoints_.size() << " default waypoints.");
    }

    planner.run();

    ROS_INFO("shutting down ros_backend_node");

    return 0;
}
