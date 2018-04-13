
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

#include <sensor_msgs/Imu.h>
#include <visensor_simulator/logger.h>
#include <visensor_simulator/simple_waypoint_planner.h>

using namespace ros;

#define STRING_ENUM(value) case RosBackendNode::value: os << "Current state: " << #value; break;


class RosBackendNode{
public:
    RosBackendNode();
    void run();
    ~RosBackendNode(){}
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

    Logger logger_;
    SimpleWaypointPlanner simple_planner_;

    //Callback functions
    void referenceOdometryCallback(const nav_msgs::Odometry& odometry_msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void sendPoseCommand(const Eigen::Vector3d &desired_position, const double &desired_yaw);
    void goNextPose();

    void setState(SimulationState state);
    SimulationState state_;
};

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
    reference_odometry_sub_ =  nh_.subscribe("imu_frame_odometry_topic", 1000, &RosBackendNode::referenceOdometryCallback, this);
    imu_sub_ =  nh_.subscribe("imu_topic", 1000, &RosBackendNode::imuCallback, this);

    pose_command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

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

    logger_.logIMU(imu_measurement);
}

void RosBackendNode::sendPoseCommand(const Eigen::Vector3d &desired_position,const double &desired_yaw)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    ROS_WARN("Publishing NEW waypoint: %lf :: %lf :: %lf :: %lf",desired_position.x(), desired_position.y(), desired_position.z(),desired_yaw/kDEG_2_RAD);
    pose_command_pub_.publish(trajectory_msg);
}

void RosBackendNode::goNextPose()
{
    Eigen::Vector3d desired_position;
    double desired_yaw;
    simple_planner_.getNextWaypoint(desired_position,desired_yaw);
    sendPoseCommand(desired_position,desired_yaw);
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
    logger_.logPose(pose);

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

void RosBackendNode::run()
{

    nh_private_.setParam("filename_waypoints", "/home/lucas/catkin_ws/src/visensor_simulator/resources/waypoints.txt"); //for debug

    //open waypoints file
    std::string filename_waypoints;
    if (!nh_private_.getParam("filename_waypoints", filename_waypoints)) {
        ROS_ERROR("\"~filename_waypoints\" parameter not provided");
        return;
    }

    if(!simple_planner_.loadWaypointsFromFile(filename_waypoints)){
        ROS_ERROR("\"~filename_waypoints\" parameter is invalid");
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

    logger_.startLogger("/home/lucas/tmp/test_logger");

    setState(StartingRecording);

    loop_rate = ros::Rate(100);

    while(nh_.ok()&& state_ != NoMission ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    //    // start vi-slam rotine
    //TODO
    //    // start mission

    setState(InMission);

    while(nh_.ok()&& state_ != NoMission ){
        ros::spinOnce();
        loop_rate.sleep();
    }

    //    // stop to record
    setState(StopingRecording);

    logger_.stop();
    //    // end

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_backend_node");

    RosBackendNode node;

    ROS_INFO("ros_backend_node started");

    //param use_embeed planner
    //param outputfolder

    node.run();

    ROS_INFO("shutting down ros_backend_node");

    return 0;
}
