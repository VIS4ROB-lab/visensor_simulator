
#ifndef VISENSOR_SIMULATOR_LOGGER_H
#define VISENSOR_SIMULATOR_LOGGER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <ros/time.h>
#include <vector>

struct ImuSensorReadings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuSensorReadings() : gyroscopes(), accelerometers(), timestamp(0) {}

  ImuSensorReadings(
      ros::Time timestamp_, Eigen::Vector3d gyroscopes_,
      Eigen::Vector3d accelerometers_)
      : gyroscopes(gyroscopes_),
        accelerometers(accelerometers_),
        timestamp(timestamp_) {}
  Eigen::Vector3d gyroscopes;      ///< Gyroscope measurement.
  Eigen::Vector3d accelerometers;  ///< Accelerometer measurement.
  ros::Time timestamp;
};

struct PoseReadings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  ros::Time timestamp;
};

std::ostream& operator<<(std::ostream& os, const PoseReadings& T) {
  Eigen::Vector3d p_WS_W = T.position;
  Eigen::Quaterniond q_WS = T.orientation;
  os << T.timestamp.toNSec() << "," << std::scientific << std::setprecision(18)
     << p_WS_W[0] << "," << p_WS_W[1] << "," << p_WS_W[2] << "," << q_WS.x()
     << "," << q_WS.y() << "," << q_WS.z() << "," << q_WS.w();
  return os;
}

std::ostream& operator<<(std::ostream& os, const ImuSensorReadings& T) {
  Eigen::Vector3d acc = T.accelerometers;
  Eigen::Vector3d gyro = T.gyroscopes;
  os << T.timestamp.toNSec() << "," << std::scientific << std::setprecision(18)
     << acc[0] << "," << acc[1] << "," << acc[2] << "," << gyro[0] << ","
     << gyro[1] << "," << gyro[2];
  return os;
}

class Logger {
 public:
  Logger() : is_valid_(false) {}
  ~Logger() {
    stop();
  }

  bool startLogger(std::string output_folder) {
    if (!output_folder.empty() &&
        output_folder.at(output_folder.length() - 1) != '/')
      output_folder += "/";

    std::string imu_filename = output_folder + "imu_data.csv";
    std::string pose_filename = output_folder + "pose_data.csv";
    // ROS_INFO_STREAM(" " << imu_filename);
    imu_file.open(imu_filename);
    pose_file.open(pose_filename);

    if (imu_file.is_open() && pose_file.is_open()) {
      is_valid_ = true;
    } else {
      is_valid_ = false;
      imu_file.close();
      pose_file.close();
    }

    imu_file << "timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z" << std::endl << std::flush;
    pose_file << "timestamp, p_x, p_y,p_z, q_x, q_y,q_z, q_w" << std::endl << std::flush;


    imu_data.clear();
    imu_data.reserve(100000);
    pose_data.clear();
    pose_data.reserve(100000);

    return is_valid_;
  }

  bool logPose(const PoseReadings& measurement) {
    if (!is_valid_)
      return false;

    pose_data.push_back(measurement);
    return true;
  }

  bool logIMU(const ImuSensorReadings& measurement) {
    if (!is_valid_)
      return false;

    imu_data.push_back(measurement);
    return true;
  }

  void stop() {
    if (!is_valid_)
      return;

    serializeImu();
    serializePoses();

    is_valid_ = false;
    imu_file.close();
    pose_file.close();
  }

 private:
  bool is_valid_;
  std::vector<ImuSensorReadings, Eigen::aligned_allocator<ImuSensorReadings>>
      imu_data;
  std::vector<PoseReadings, Eigen::aligned_allocator<PoseReadings>> pose_data;

  std::ofstream imu_file;
  std::ofstream pose_file;

  void serializeImu() {
    
    for (const auto& reading : imu_data)
      imu_file << reading << std::endl;
  }

  void serializePoses() {
    
    for (const auto& reading : pose_data)
      pose_file << reading << std::endl;
  }
};

#endif
