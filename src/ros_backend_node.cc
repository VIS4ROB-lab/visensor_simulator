
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
#include <visensor_simulator/logger.h>
#include <visensor_simulator/simple_waypoint_planner.h>
#include <visensor_simulator/ros_backend_node.h>
#include <visensor_simulator/logger.h>

#include <boost/filesystem.hpp>

using namespace ros;

#define STRING_ENUM(value) case RosBackendNode::value: os << "Current state: " << #value; break;


std::ostream& operator<<( std::ostream& os, const RosBackendNode::SimulationState& state )
{
    switch( state )
    {
    STRING_ENUM(NoMission)
            STRING_ENUM(GoingToStart)
            STRING_ENUM(InStartPose)
            STRING_ENUM(StartingRecording)
            STRING_ENUM(InMission)
            STRING_ENUM(StopingRecording)
    }
}


RosBackendNode::RosBackendNode():
    nh_private_("~")
{
    logger_ = new Logger();
    reference_odometry_sub_ =  nh_.subscribe("imu_frame_odometry_topic", 1000, &RosBackendNode::referenceOdometryCallback, this);
    imu_sub_ =  nh_.subscribe("imu_topic", 1000, &RosBackendNode::imuCallback, this);

    pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
    gimbal_command_pub_ = nh_.advertise<sensor_msgs::Joy>("command/gimbal_actuators", 1);

    state_ = NoMission;
}

void RosBackendNode::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    ImuSensorReadings imu_measurement;
    imu_measurement.accelerometers = Eigen::Vector3d(msg->linear_acceleration.x,
                                                     msg->linear_acceleration.y,
                                                     msg->linear_acceleration.z);

    imu_measurement.gyroscopes = Eigen::Vector3d(msg->angular_velocity.x,
                                                 msg->angular_velocity.y,
                                                 msg->angular_velocity.z);

    imu_measurement.timestamp = msg->header.stamp;

    logger_->logIMU(imu_measurement);
}

void RosBackendNode::sendPoseCommand(const Eigen::Vector3d &desired_position,const double &desired_yaw,const float &desired_gimbal_pitch)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    //TODO por de volta ROS_WARN("Publishing NEW waypoint: %lf :: %lf :: %lf :: %lf :: %f",desired_position.x(), desired_position.y(), desired_position.z(),desired_yaw/kDEG_2_RAD,desired_gimbal_pitch);
    pose_command_pub_.publish(trajectory_msg);
    sensor_msgs::Joy gimbal_msg;
    gimbal_msg.axes= {0.0f,desired_gimbal_pitch,0.0f};
    gimbal_command_pub_.publish(gimbal_msg);
}

void RosBackendNode::goNextPose()
{
    Eigen::Vector3d desired_position;
    double desired_yaw;
    float desired_gimbal_pitch;
    simple_planner_.getNextWaypoint(desired_position,desired_yaw,desired_gimbal_pitch);
    sendPoseCommand(desired_position,desired_yaw,desired_gimbal_pitch);
}

void RosBackendNode::setState(RosBackendNode::SimulationState state)
{
    ROS_INFO_STREAM("" << state_ );
    state_ = state;
}

void RosBackendNode::referenceOdometryCallback(const nav_msgs::Odometry &odometry_msg){

    const  geometry_msgs::Point& curr_position = odometry_msg.pose.pose.position;

    PoseReadings pose;
    pose.position = Eigen::Vector3d(curr_position.x,curr_position.y,curr_position.z);
    pose.orientation = Eigen::Quaterniond(odometry_msg.pose.pose.orientation.w
                                          ,odometry_msg.pose.pose.orientation.x
                                          ,odometry_msg.pose.pose.orientation.y
                                          ,odometry_msg.pose.pose.orientation.z);

    pose.timestamp = odometry_msg.header.stamp;
    logger_->logPose(pose);

    if(state_ == InMission ){
        if(simple_planner_.step(odometry_msg.pose.pose))
            goNextPose();
        else
        {
            if(simple_planner_.getStatus() == SimpleWaypointPlanner::FINISHED)
            {
                setState(NoMission);
            }
        }
    }
    else if(state_ == GoingToStart)
    {
        if(simple_planner_.reachedNextWaypoint(odometry_msg.pose.pose))
        {
            setState(NoMission);
        }else{
            static ros::Time last_time(0);
            if(ros::Time::now() - last_time > ros::Duration(1,0) )
            {
                goNextPose();
                last_time = ros::Time::now();
            }
        }
    }else if(state_ == StartingRecording)
    {
        static uint counter = 0;
        if(counter++ > 100)
        {
            setState(NoMission);
        }
    }

    return;
}

void RosBackendNode::run( std::string project_folder )
{

    boost::filesystem::path project_folder_path(project_folder);

    if(!boost::filesystem::exists(project_folder_path))
    {
        ROS_ERROR_STREAM("the project folder does not exists :" << project_folder_path.c_str());
        return;
    }

    boost::filesystem::path waypoints_path = project_folder_path / "waypoints.txt";
    boost::filesystem::path output_folder_path = project_folder_path / "output/1_Rotors";


    if(!boost::filesystem::exists(waypoints_path))
    {
        ROS_ERROR_STREAM("the project folder does not have a waypoints.txt file :" << waypoints_path.c_str());
        return;
    }

    if(!boost::filesystem::exists(output_folder_path))
    {
        if(!boost::filesystem::create_directories(output_folder_path))
        {
            ROS_ERROR_STREAM("the output folder could not be created :" << output_folder_path.c_str());
            return;
        }
    }else{
        ROS_ERROR_STREAM("the output folder already exist - please delete it :" << output_folder_path.c_str());
        return;
    }



    //open waypoints file

    if(!simple_planner_.loadWaypointsFromFile(waypoints_path.c_str())){
        ROS_ERROR("waypoints.txt file is invalid");
        return;
    }

    setState(GoingToStart);

    // send to the start position

    ros::Rate loop_rate(1);
    while(nh_.ok() && state_ != NoMission){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // start to record

    logger_->startLogger(output_folder_path.c_str());

    setState(StartingRecording);

    loop_rate = ros::Rate(100);

    while(nh_.ok()&& state_ != NoMission ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // start vi-slam rotine
    //TODO
    // start mission

    setState(InMission);

    while(nh_.ok()&& state_ != NoMission ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop to record
    setState(StopingRecording);

    logger_->stop();
    // end

}

RosBackendNode::~RosBackendNode(){
    delete logger_;
}
