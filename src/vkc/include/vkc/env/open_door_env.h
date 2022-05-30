#ifndef VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H
#define VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>

#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/object/objects.h>

#include <string>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

namespace vkc {
/**
 * @brief Load single ur5e husky for door opening task.
 */
class OpenDoorEnv : public VKCEnvBasic {
 public:
  OpenDoorEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps);

  ~OpenDoorEnv() = default;

  bool createEnvironment() override;
};

}  // namespace vkc

#endif  // VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H