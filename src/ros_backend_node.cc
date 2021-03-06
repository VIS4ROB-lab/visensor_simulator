
#include <boost/filesystem.hpp>
#include <fstream>
#include <list>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <visensor_simulator/builtin_planner.h>
#include <visensor_simulator/logger.h>
#include <visensor_simulator/ros_backend_node.h>

using namespace ros;

#define STRING_ENUM(value)             \
  case RosBackendNode::value:          \
    os << "Current state: " << #value; \
    break;

std::ostream& operator<<(
    std::ostream& os, const RosBackendNode::SimulationState& state) {
  switch (state) {
    STRING_ENUM(INVALID)
    STRING_ENUM(WAINTING_PLANNER)
    STRING_ENUM(RECORDING)
    STRING_ENUM(FINISHED)
  }
}

RosBackendNode::RosBackendNode(BuiltInPlanner* planner) : planner_(planner) {
  simulation_state_ = INVALID;

  logger_ = new Logger();
  reference_odometry_sub_ = nh_.subscribe(
      "imu_frame_odometry_topic", 1000,
      &RosBackendNode::referenceOdometryCallback, this);
  imu_sub_ =
      nh_.subscribe("imu_topic", 1000, &RosBackendNode::imuCallback, this);

  if (planner_ != nullptr)
    use_builtin_planner_ = true;
  else
    use_builtin_planner_ = false;
}

void RosBackendNode::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  if (simulation_state_ == RECORDING) {
    ImuSensorReadings imu_measurement;
    imu_measurement.accelerometers = Eigen::Vector3d(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    imu_measurement.gyroscopes = Eigen::Vector3d(
        msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z);

    imu_measurement.timestamp = msg->header.stamp;

    logger_->logIMU(imu_measurement);
  }
}

void RosBackendNode::updateBuiltInPlanner() {
  BuiltInPlanner::PlannerStatus planner_status = planner_->getStatus();

  if (simulation_state_ == WAINTING_PLANNER &&
      planner_status == BuiltInPlanner::RUNNING) {
    // start to record
    logger_->startLogger(output_folder_path_.c_str());
    setState(RECORDING);
  }

  if (simulation_state_ == RECORDING &&
      planner_status == BuiltInPlanner::COMPLETED) {
    setState(FINISHED);
  }
}

void RosBackendNode::setState(RosBackendNode::SimulationState state) {
  ROS_INFO_STREAM(
      "Change state from: " << simulation_state_ << " to: " << state);
  simulation_state_ = state;
}

void RosBackendNode::referenceOdometryCallback(
    const nav_msgs::Odometry& odometry_msg) {
  const geometry_msgs::Point& curr_position = odometry_msg.pose.pose.position;

  if (use_builtin_planner_) {
    updateBuiltInPlanner();
  }

  if (simulation_state_ == RECORDING) {
    PoseReadings pose;
    pose.position =
        Eigen::Vector3d(curr_position.x, curr_position.y, curr_position.z);
    pose.orientation = Eigen::Quaterniond(
        odometry_msg.pose.pose.orientation.w,
        odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y,
        odometry_msg.pose.pose.orientation.z);

    pose.timestamp = odometry_msg.header.stamp;
    logger_->logPose(pose);
  }

  return;
}

void RosBackendNode::run(const std::string& project_folder) {
  boost::filesystem::path project_folder_path(project_folder);

  if (!boost::filesystem::exists(project_folder_path)) {
    ROS_ERROR_STREAM(
        "the project folder does not exists :" << project_folder_path.c_str());
    return;
  }

  boost::filesystem::path output_folder_path =
      project_folder_path / "output/1_InertialPose";

  if (!boost::filesystem::exists(output_folder_path)) {
    if (!boost::filesystem::create_directories(output_folder_path)) {
      ROS_ERROR_STREAM(
          "the output folder could not be created :"
          << output_folder_path.c_str());
      return;
    }
  } else {
    ROS_ERROR_STREAM(
        "the output folder already exist - please delete it :"
        << output_folder_path.c_str());
    return;
  }

  setState(WAINTING_PLANNER);

  output_folder_path_ = std::string(output_folder_path.c_str());

  if (use_builtin_planner_) {
    // open waypoints file
    if (!planner_->loadConfigurationFromFile(project_folder_path.c_str())) {
      ROS_ERROR("Fail to load configuration.");
      return;
    }
  } else {
    logger_->startLogger(output_folder_path_.c_str());
    setState(RECORDING);
    // setup services
  }

  ros::Rate loop_rate = ros::Rate(100);

  while (ros::ok() && simulation_state_ != FINISHED) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // save the log file
  logger_->stop();
  if (simulation_state_ == FINISHED)
    ROS_INFO("Simulation data saved!");
  else if (use_builtin_planner_)
    ROS_WARN("Simulation data saved, but it is INCOMPLETE!");
}

RosBackendNode::~RosBackendNode() {
  delete logger_;
}
