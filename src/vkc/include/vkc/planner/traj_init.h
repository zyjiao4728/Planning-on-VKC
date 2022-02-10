#ifndef VKC_TRAJ_INIT_H
#define VKC_TRAJ_INIT_H

#include <tesseract_collision/core/common.h>
#include <tesseract_common/macros.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ros/ros.h>
#include <AStar.hpp>

#include <stdlib.h>
#include <time.h>
#include <vkc/env/vkc_env_basic.h>
#include <cmath>

namespace vkc
{
  struct MapInfo
  {
    int map_x;
    int map_y;
    double step_size;
    int grid_size_x;
    int grid_size_y;

    MapInfo(int x, int y, double step) : map_x(x), map_y(y), step_size(step)
    {
      grid_size_x = int(map_x / step_size) + 1;
      grid_size_y = int(map_y / step_size) + 1;
    }
  };

  bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager, std::string link_name,
                   Eigen::Isometry3d &tf, tesseract_collision::ContactResultMap &contact_results);

  void initBaseTrajectory(VKCEnvBasic &env, std::vector<LinkDesiredPose> &base_pose, MapInfo &map);

  void initFinalJointSeed(std::unordered_map<std::string, int> &joint_name_idx,
                          std::vector<JointDesiredPose> &joint_objectives, Eigen::VectorXd &seed);

  bool checkJointLimit(Eigen::VectorXd &sol, const Eigen::MatrixX2d &joint_limit, size_t num_joint);

  double interpolate(std::vector<LinkDesiredPose> base_pose, std::vector<double> remap, int i, bool x);

  trajopt::TrajArray initTrajectory(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
                                    std::vector<JointDesiredPose> &joint_objectives, MapInfo map,
                                    trajopt::TrajArray &init_traj, int n_steps, const std::string& robot);

  std::vector<double> initIK(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
                             std::vector<JointDesiredPose> &joint_objectives, MapInfo map, int n_steps);

  // trajopt::TrajArray initTrajectoryByOMPL(VKCEnvBasic &env, ActionBase::Ptr act, int n_steps);
} // namespace vkc

#endif // VKC_TRAJ_INIT_H
