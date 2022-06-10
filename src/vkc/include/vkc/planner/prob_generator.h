#ifndef VKC_PROB_GENERATOR_H
#define VKC_PROB_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_process_managers/core/process_planning_request.h>
// #include <tesseract_process_managers/core/default_process_planners.h>
#include <tesseract_environment/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <vkc/env/vkc_env_basic.h>

#include <iostream>
#include <string>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <vector>

namespace vkc {

class ProbGenerator {
 public:
  ProbGenerator();

 public:
  tesseract_planning::PlannerRequest genRequest(VKCEnvBasic &env,
                                                ActionBase::Ptr action,
                                                int n_steps, int n_iter);

  tesseract_planning::PlannerRequest genRequest(
      VKCEnvBasic &env, ActionBase::Ptr action,
      tesseract_planning::Waypoint waypoint, int n_steps, int n_iter);

  tesseract_planning::MixedWaypoint genMixedWaypoint(VKCEnvBasic &env,
                                                     ActionBase::Ptr act);

  tesseract_planning::MixedWaypoint genPickMixedWaypoint(VKCEnvBasic &env,
                                                         PickAction::Ptr act);

  tesseract_planning::MixedWaypoint genPlaceMixedWaypoint(VKCEnvBasic &env,
                                                          PlaceAction::Ptr act);

  tesseract_planning::MixedWaypoint genGotoMixedWaypoint(VKCEnvBasic &env,
                                                         GotoAction::Ptr act);

  tesseract_planning::PlannerRequest genPickProb(VKCEnvBasic &env,
                                                 PickAction::Ptr act,
                                                 int n_steps, int n_iter);
  // trajopt::TrajOptProb::Ptr genPickProbOMPL(VKCEnvBasic &env, PickAction::Ptr
  // act, int n_steps);

  tesseract_planning::PlannerRequest genPlaceProb(VKCEnvBasic &env,
                                                  PlaceAction::Ptr act,
                                                  int n_steps, int n_iter);
  // trajopt::TrajOptProb::Ptr genPlaceProbOMPL(VKCEnvBasic &env,
  // PlaceAction::Ptr act, int n_steps);
  tesseract_planning::PlannerRequest genGotoProb(VKCEnvBasic &env,
                                                 GotoAction::Ptr act,
                                                 int n_steps);
  tesseract_planning::PlannerRequest genUseProb(VKCEnvBasic &env,
                                                UseAction::Ptr act,
                                                int n_steps);

  trajopt::ProblemConstructionInfo genProbTest(VKCEnvBasic &env,
                                               ActionBase::Ptr action,
                                               int n_steps);
  trajopt::ProblemConstructionInfo genGotoProb_test(VKCEnvBasic &env,
                                                    GotoAction::Ptr act,
                                                    int n_steps);
  trajopt::ProblemConstructionInfo genPickProb_test(VKCEnvBasic &env,
                                                    PickAction::Ptr act,
                                                    int n_steps);
  trajopt::ProblemConstructionInfo genPlaceProb_test(VKCEnvBasic &env,
                                                     PlaceAction::Ptr act,
                                                     int n_steps);
  trajopt::ProblemConstructionInfo genUseProb_test(VKCEnvBasic &env,
                                                   UseAction::Ptr act,
                                                   int n_steps);

  // int initProbInfo(trajopt::ProblemConstructionInfo &pci,
  // tesseract_environment::Environment::Ptr tesseract, int n_steps,
  //                  std::string manip);

 protected:
  bool validateGroupID(tesseract_environment::Environment::Ptr tesseract,
                       const std::string &group_id);

  Eigen::Vector4d getQuatFromIso(Eigen::Isometry3d iso);

  tesseract_planning::ProfileDictionary::Ptr genPlannerProfiles_(
      VKCEnvBasic &env, tesseract_planning::ManipulatorInfo manip,
      double collision_margin, double collision_coeff,
      Eigen::Vector3d pos_coeff, Eigen::Vector3d rot_coeff);

  void setJointPlanProfile(
      tesseract_planning::TrajOptDefaultPlanProfile::Ptr profile,
      Eigen::VectorXd joint_coeff);

  void setCartPlanProfile(
      tesseract_planning::TrajOptDefaultPlanProfile::Ptr profile,
      Eigen::Vector3d pos_coeff, Eigen::Vector3d rot_coeff);

  void setCompositeProfile(
      tesseract_planning::TrajOptDefaultCompositeProfile::Ptr profile,
      double margin, double coeff, Eigen::Index num_joints);

  void setStartInstruction(tesseract_planning::CompositeInstruction &program,
                           ros::V_string joint_names,
                           Eigen::VectorXd joint_values);

  void addCartWaypoint(tesseract_planning::CompositeInstruction &program,
                       Eigen::Isometry3d pose, std::string description);

  void addSolverProfile(tesseract_planning::ProfileDictionary::Ptr profiles,
                        int n_iter);

  void addJointTerm(trajopt::ProblemConstructionInfo &pci, int joint_num);

  void addCollisionTerm(trajopt::ProblemConstructionInfo &pci, double margin,
                        double coeff);

  void addTargetTerm(trajopt::ProblemConstructionInfo &pci,
                     LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                     Eigen::Vector3d rot_coeff);

  void addTargetTerm(trajopt::ProblemConstructionInfo &pci,
                     JointDesiredPose &joint_pose, int joint_num, double coeff);

  void addTargetCost(trajopt::ProblemConstructionInfo &pci,
                     LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                     Eigen::Vector3d rot_coeff);

 private:
  std::unordered_map<std::string, int> planned_joints;
};

}  // namespace vkc

#endif
