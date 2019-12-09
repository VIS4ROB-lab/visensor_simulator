#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/Joy.h>

namespace gazebo {

//// Default values
static const std::string kDefaultParentFrameId = "gimbal_support";
static const std::string kDefaultChildFrameId = "camera_mount";
static const std::string kDefaulGimbaltLinkName = "firefly/gimbal_support_link";
static const std::string kDefaultCameraLinkName = "firefly/gimbal_yaw";
static const uint kNumActuators = 3;
static const double kDefaultVelocity = 0.3;
static const double kDEG_2_RAD = M_PI / 180.0;

class GazeboPoltergeistGimbalPlugin : public ModelPlugin {
 public:
  GazeboPoltergeistGimbalPlugin() : ModelPlugin() {}
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(
          argc, argv, "gazebo_gimbal_plugin",
          ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    node_handle_.reset(new ros::NodeHandle("gazebo_gimbal_plugin"));

    if (_sdf->HasElement("robot_namespace"))
      namespace_ = _sdf->Get<std::string>("robot_namespace");
    else
      namespace_ = "/gimbal";

    // Store the pointer to the model
    model_ = _parent;

    std::string ref_base_link_str = _sdf->Get<std::string>("ref_base_link");
    gimbal_support_link_ = model_->GetChildLink(ref_base_link_str);

    std::string cam_link_str = _sdf->Get<std::string>("camera_link");
    camera_mount_link_ = model_->GetChildLink(cam_link_str);
    // TODO(lpt) add the transform publisher

    // rpy order roll pitch yaw

    // TODO(lpt) add more error handling for th parameters
    if (_sdf->HasElement("roll_joint_name")) {
      joints_.push_back(
          model_->GetJoint(_sdf->Get<std::string>("roll_joint_name")));
      base_points_.push_back(_sdf->Get<double>("roll_zero"));
      directions_.push_back(_sdf->Get<double>("roll_direction"));
    } else {
      joints_.push_back(nullptr);
      base_points_.push_back(-1e15);
      directions_.push_back(-1e15);
    }

    if (_sdf->HasElement("pitch_joint_name")) {
      joints_.push_back(
          model_->GetJoint(_sdf->Get<std::string>("pitch_joint_name")));
      base_points_.push_back(_sdf->Get<double>("pitch_zero"));
      directions_.push_back(_sdf->Get<double>("pitch_direction"));
    } else {
      joints_.push_back(nullptr);
      base_points_.push_back(-1e15);
      directions_.push_back(-1e15);
    }

    if (_sdf->HasElement("yaw_joint_name")) {
      joints_.push_back(
          model_->GetJoint(_sdf->Get<std::string>("yaw_joint_name")));
      base_points_.push_back(_sdf->Get<double>("yaw_zero"));
      directions_.push_back(_sdf->Get<double>("yaw_direction"));
    } else {
      joints_.push_back(nullptr);
      base_points_.push_back(-1e15);
      directions_.push_back(-1e15);
    }
    goal_points_.assign(kNumActuators, 0);

    max_rotational_velocity_ = kDefaultVelocity;
    if (_sdf->HasElement("gimbal_angular_velocity"))
      max_rotational_velocity_ = _sdf->Get<double>("gimbal_angular_velocity");

    input_sub_ = node_handle_->subscribe<sensor_msgs::Joy>(
        namespace_ + "/command/gimbal_actuators", 10,
        &GazeboPoltergeistGimbalPlugin::onInputCallback, this);

    for (int i = 0; i < kNumActuators; i++) {
      if (joints_[i] != nullptr) {
        joints_[i]->SetPosition(0, base_points_[i] * kDEG_2_RAD);
      }
    }

    previousUpdate_ = 0.0;

    //         Listen to the update event. This event is broadcast every
    //         simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(
            &GazeboPoltergeistGimbalPlugin::OnUpdate, this,
            std::placeholders::_1));

    ROS_WARN("Hello World! gazebo_poltergeist_gimbal model v17");
  }

  void onInputCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    if (msg->axes.size() != 3) {
      ROS_ERROR_STREAM(
          "Gimbal Request need to have 3 axes, Roll: Pitch: Yaw, even that "
          "they are not avaiable");
    }
    ROS_INFO_STREAM(
        "Gimbal Request Roll:" << msg->axes[0] << " Pitch:" << msg->axes[1]
                               << " Yaw:" << msg->axes[2] << " ");
    bool input_valid = true;
    if (joints_[0] != nullptr && (msg->axes[0] > 180 || msg->axes[0] < -179)) {
      ROS_WARN_STREAM(
          "Gimbal Request ignored - invalid ROLL [-179,180]: " << msg->axes[0]);
      input_valid = false;
    }

    if (joints_[1] != nullptr && (msg->axes[1] > 180 || msg->axes[1] < -179)) {
      ROS_WARN_STREAM(
          "Gimbal Request ignored - invalid PITCH [-179,180]: "
          << msg->axes[1]);
      input_valid = false;
    }

    if (joints_[2] != nullptr && (msg->axes[2] > 180 || msg->axes[2] < -179)) {
      ROS_WARN_STREAM(
          "Gimbal Request ignored - invalid YAW [-179,180]: " << msg->axes[2]);
      input_valid = false;
    }

    if (input_valid) {
      for (int i = 0; i < kNumActuators; i++) {
        if (joints_[i] == nullptr)
          continue;
        goal_points_[i] = msg->axes[i];
      }
    }
  }

  // Called by the world update start event
  void OnUpdate(const common::UpdateInfo& _info) {
    for (int i = 0; i < kNumActuators; i++) {
      if (joints_[i] == nullptr)
        continue;

      double curr_position = GetJointPositionOffSeted(i);

      double angle_diff = curr_position - goal_points_[i];

      common::Time delta_time = _info.simTime - previousUpdate_;

      if (delta_time > 0) {
        double update_vel = std::min(
            std::abs(0.9 * kDEG_2_RAD * angle_diff / delta_time.Double()),
            max_rotational_velocity_);
        if (angle_diff > 0) {
          setSpeed(i, -update_vel);
        } else {
          setSpeed(i, update_vel);
        }
      }

      // joints_[i]->SetPosition(0,0.0); Attention: never use setposition
      // function or fmax = 0
      // joints_[i]->SetParam("fmax", 0, 0.0);

      previousUpdate_ = _info.simTime;
    }
  }

  double setSpeed(size_t joint_index, double vel) {
#if GAZEBO_MAJOR_VERSION > 4
    if (joints_[joint_index] != nullptr) {
      joints_[joint_index]->SetParam("fmax", 0, 100.0);
      joints_[joint_index]->SetParam(
          "vel", 0, vel);  // joint motor - only supported by ODE
    }
#else
    if (joints_[joint_index] != nullptr) {
      joints_[joint_index]->SetAttribute("fmax", 0, 100.0);
      joints_[joint_index]->SetAttribute("vel", 0, vel);
    }
#endif
  }

  double GetJointPosition(size_t joint_index) {
    if (joints_[joint_index] != nullptr)
      return ConvertAngle180(
          std::fmod(
              joints_[joint_index]->Position(0) * (180 / M_PI) + 72000000.0, 360.0));

    return -1e16;
  }

  double GetJointPositionOffSeted(size_t joint_index) {
    return ConvertAngle180(
        std::fmod(
            GetJointPosition(joint_index) - base_points_[joint_index] + 360.0,
            360.0));
  }

  double ConvertAngle180(double angle)  //[-180,180]
  {
    return std::fmod(angle + 180.0, 360.0) - 180.0;  // 0->0 , 10->10, 350->-10
  }

  double ConvertAngle360(double angle)  //[0,360]
  {
    return std::fmod(angle + 360.0, 360.0);  // 0 -> 0 , 10 -> 10, -10 -> 350
  }

  double test_math() {
    double joint_index = 1;
    ROS_INFO_STREAM(
        " 1: " << joints_[joint_index]->Position(0) * (180 / M_PI) << " 2: "
               << std::fmod(
                      joints_[joint_index]->Position(0) * (180 / M_PI) + 72000000.0,
                      360.0)
               << " 3: " << GetJointPosition(1) << " 4: "
               << GetJointPosition(joint_index) - base_points_[joint_index] +
                      360.0
               << " 5: " << std::fmod(
                                GetJointPosition(joint_index) -
                                    base_points_[joint_index] + 360.0,
                                360.0)
               << " 6: " << GetJointPositionOffSeted(1));
  }

  // Pointer to the model
 private:
  /// STATE

  double max_rotational_velocity_;

  std::string namespace_;

  common::Time previousUpdate_;

  /// GAZEBO
  ///

  physics::ModelPtr model_;

  std::unique_ptr<ros::NodeHandle> node_handle_;

  physics::LinkPtr gimbal_support_link_;
  physics::LinkPtr camera_mount_link_;

  std::vector<double> base_points_;
  std::vector<double> goal_points_;
  std::vector<double> directions_;
  std::vector<physics::JointPtr> joints_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  /// ROS

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> ros_node_;

  /// \brief A ROS subscriber
  ros::Subscriber input_sub_;

  ros::Publisher status_pub_;                   // TODO
  ros::Publisher transform_gimbal_camera_pub_;  // TODO
  ros::Publisher gimbal_camera_tf_pub_;         // TODO
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboPoltergeistGimbalPlugin)
}
