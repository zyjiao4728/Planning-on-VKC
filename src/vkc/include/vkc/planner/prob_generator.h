#ifndef VKC_PROB_GENERATOR_H
#define VKC_PROB_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <vkc/env/vkc_env_basic.h>

#include <iostream>
#include <string>
#include <vector>

namespace vkc
{
class ProbGenerator
{
public:
  ProbGenerator();

public:
  trajopt::TrajOptProb::Ptr genProb(VKCEnvBasic &env, ActionBase::Ptr action, int n_steps);
  trajopt::TrajOptProb::Ptr genPickProb(VKCEnvBasic &env, PickAction::Ptr act, int n_steps);
  // trajopt::TrajOptProb::Ptr genPickProbOMPL(VKCEnvBasic &env, PickAction::Ptr act, int n_steps);
  trajopt::TrajOptProb::Ptr genPlaceProb(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps);
  // trajopt::TrajOptProb::Ptr genPlaceProbOMPL(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps);
  trajopt::TrajOptProb::Ptr genGotoProb(VKCEnvBasic &env, GotoAction::Ptr act, int n_steps);
  trajopt::TrajOptProb::Ptr genUseProb(VKCEnvBasic &env, UseAction::Ptr act, int n_steps);

  trajopt::ProblemConstructionInfo genProbTest(VKCEnvBasic &env, ActionBase::Ptr action, int n_steps);
  trajopt::ProblemConstructionInfo genGotoProb_test(VKCEnvBasic &env, GotoAction::Ptr act, int n_steps);
  trajopt::ProblemConstructionInfo genPickProb_test(VKCEnvBasic &env, PickAction::Ptr act, int n_steps);
  trajopt::ProblemConstructionInfo genPlaceProb_test(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps);
  trajopt::ProblemConstructionInfo genUseProb_test(VKCEnvBasic &env, UseAction::Ptr act, int n_steps);

  int initProbInfo(trajopt::ProblemConstructionInfo &pci, tesseract_monitoring::EnvironmentMonitor::Ptr tesseract, int n_steps,
                   std::string manip);
protected:


  bool validateGroupID(tesseract_monitoring::EnvironmentMonitor::Ptr tesseract, const std::string &group_id);

  Eigen::Vector4d getQuatFromIso(Eigen::Isometry3d iso);

  void addJointTerm(trajopt::ProblemConstructionInfo &pci, int joint_num);

  void addCollisionTerm(trajopt::ProblemConstructionInfo &pci, double margin, double coeff);

  void addTargetTerm(trajopt::ProblemConstructionInfo &pci, LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                     Eigen::Vector3d rot_coeff);

  void addTargetTerm(trajopt::ProblemConstructionInfo &pci, JointDesiredPose &joint_pose, int joint_num, double coeff);

  void addTargetCost(trajopt::ProblemConstructionInfo &pci, LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                     Eigen::Vector3d rot_coeff);

private:
  std::unordered_map<std::string, int> planned_joints;
};

}  // namespace vkc

#endif
