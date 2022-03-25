#ifndef VIRTUAL_KINEMATIC_CHAIN_ARENA_ENV_H
#define VIRTUAL_KINEMATIC_CHAIN_ARENA_ENV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <vkc/env/vkc_env_basic.h>
#include <string>


namespace vkc
{
/**
 * @brief Load robot model for multiple tasks.
 */
class ArenaEnv : public VKCEnvBasic
{
public:
  ArenaEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps);

  ~ArenaEnv() = default;

  bool createEnvironment() override;

private:
  int steps_;
};

}  // namespace vkc

#endif  // VIRTUAL_KINEMATIC_CHAIN_ARENA_ENV_H
