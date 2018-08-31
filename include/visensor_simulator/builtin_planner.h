#ifndef VISENSOR_SIMULATOR_BUILTIN_PLANNER_H
#define VISENSOR_SIMULATOR_BUILTIN_PLANNER_H

#include <nav_msgs/Odometry.h>
#include <string>

class BuiltInPlanner {
 public:
  enum PlannerStatus { INVALID, STARTING, RUNNING, COMPLETED };

  virtual PlannerStatus getStatus() = 0;
  virtual bool loadConfigurationFromFile(const std::string& project_folder_path) = 0;
};

#endif  // VISENSOR_SIMULATOR_BUILTIN_PLANNER_H
