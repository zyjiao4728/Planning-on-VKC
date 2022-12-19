#ifndef VKC_PROB_TRANSLATOR_H
#define VKC_PROB_TRANSLATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <vkc/planner/huskey_ik_define.h>
#include <vkc/planner/prob_generator.h>
#include <vkc/planner/prob_translator_utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <vkc/planner/prob_ompl_define.h>

// #include <tesseract_motion_planners/ompl/chain_ompl_interface.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

#include <iostream>
#include <string>
#include <vector>

namespace vkc {
struct OmplPlanParameters {
  int inv_attp_max = 100;
  OMPLPlanners::Planners planner = OMPLPlanners::Planners::RRT_Connect;
  Husky_IK::Option ik_option = Husky_IK::Option::Full;
  // tesseract_motion_planners::OmplPlanParameters plan_params;
};

class ProbTranslator {
 public:
  ProbTranslator(OmplPlanParameters params);

 public:
  /**
   * @brief Get the start and goal states and save them internally
   * @param env tesseract environment before planning
   * @param manipulator string of manipulator name
   */
  bool getStartAndGoalState(VKCEnvBasic &env, std::string manipulator);

  /**
   * @brief Copy the start state
   * @param env tesseract environment before planning
   * @param manipulator string of manipulator name
   */
  tesseract_planning::JointWaypointPoly setupStartWaypoint(VKCEnvBasic &env,
                                                       std::string manipulator);

  /**
   * @brief Find the goal state of the manipulator, will use IK if link
   * objective is given
   * @param env tesseract environment before planning
   * @param manipulator string of manipulator name
   */
  tesseract_planning::JointWaypointPoly setupGoalWaypoint(VKCEnvBasic &env,
                                                      std::string manipulator);

  /**
   * @brief Set up planning parameters
   * @param env tesseract environment before planning
   */
  void setupParams(VKCEnvBasic &env, vkc::ActionBase::Ptr act);

  /**
   * @brief Translate problems into OMPL tasks
   * @param env tesseract environment before planning
   * @param act action
   */
  bool transProb(VKCEnvBasic &env, vkc::ActionBase::Ptr act);

  /**
   * @brief Use this planner for OMPL
   * @param option Planner enum, only RRT related planners are implemented
   */
  void insertPlanners(OMPLPlanners::Planners option);

  /**
   * @brief Use this planner for OMPL
   * @param option Planner enum, only RRT related planners are implemented
   */
  bool solveProblem(tesseract_planning::PlannerResponse &response,
                    std::vector<std::vector<double>> &res_traj);

  tesseract_kinematics::IKSolutions inverseKinematics(
      tesseract_kinematics::KinematicGroup::UPtr &kin_,
      const Eigen::Isometry3d &pose, Eigen::VectorXd &seed);

  bool collisionFreeInverseKinematics(VKCEnvBasic &env, std::string manipulator,
                                      Eigen::VectorXd &solutions,
                                      const Eigen::Isometry3d &pose,
                                      Eigen::VectorXd &seed);

  void getJointNameIndexMap(
      tesseract_kinematics::KinematicGroup::UPtr &kin,
      std::unordered_map<std::string, int> &joint_name_idx);

  void genRandState(tesseract_kinematics::KinematicGroup::UPtr &kin,
                    Eigen::VectorXd &seed);

  tesseract_kinematics::KinematicGroup::ConstPtr getKinematics();

 protected:
  bool transPickProb(VKCEnvBasic &env, vkc::PickAction::Ptr act);

  bool transPlaceProb(VKCEnvBasic &env, vkc::PlaceAction::Ptr act);

  bool transGotoProb(VKCEnvBasic &env, vkc::GotoAction::Ptr act);

  bool transUseProb(VKCEnvBasic &env, vkc::UseAction::Ptr act);

  // wanglei@2021-10-28 to check whether solution satisfy joints limits
  bool checkJointLimits(const Eigen::VectorXd &solution,
                        const tesseract_common::KinematicLimits limits);

  // struct BaseBias
  // {
  //   double reach_length;
  //   double clearance = 0.4;
  //   double span = M_PI;
  //   double bias = 0;
  // };

  // /**
  // * @brief Set sampling parameters for base positions
  // * @param reach_length how long arm reach is
  // * @param span spread of angle
  // * @param clearance closest
  // * @param reach_length how long arm reach is
  // */
  // void setBaseBias(double reach_length, double clearance, double span, double
  // bias); void setBaseBias(BaseBias bb);

  // Eigen::VectorXd genBaseGoal(VKCEnvBasic &env, std::vector<LinkDesiredPose>
  // &link_objectives, BaseBias &bias);

  // Eigen::VectorXd initArmGoal(VKCEnvBasic &env, std::vector<LinkDesiredPose>
  // &link_objectives,
  //                             std::vector<JointDesiredPose>
  //                             &joint_objectives, Eigen::VectorXd base_pose);
  // void overwriteGoal(std::vector<double> goal);
  // void overwriteLimit(bool option);
  // void setIKOption(Husky_IK::Option option);
  // Eigen::MatrixX2d processLimit(Eigen::MatrixX2d init_limit);
  // std::vector<double> IKhelper(VKCEnvBasic &env, std::vector<LinkDesiredPose>
  // &link_objectives,
  //                              std::vector<JointDesiredPose>
  //                              &joint_objectives, MapInfo map, int n_steps,
  //                              Eigen::MatrixX2d &joint_limits, InitInfo
  //                              init_type);
  // std::vector<double> BaseFirstIK(VKCEnvBasic &env,
  // std::vector<LinkDesiredPose> &link_objectives,
  //                                 std::vector<JointDesiredPose>
  //                                 &joint_objectives, BaseBias &bias);
  // void randBase(Eigen::VectorXd &goal_pose, Eigen::VectorXd &res, BaseBias
  // &bias); TrajOptProb::Ptr moveArm(VKCEnvBasic &env, int n_steps,
  // Eigen::Vector3d p_goal, Eigen::Quaterniond q_goal, Eigen::VectorXd
  // base_pose); Eigen::VectorXd optArmGoal(VKCEnvBasic &env,
  // std::vector<LinkDesiredPose> &link_objectives,
  //                            std::vector<JointDesiredPose> &joint_objectives,
  //                            Eigen::VectorXd base_pose);
  // bool generateIK(VKCEnvBasic &env, int n_steps, Husky_IK::Option ik_option);

 private:
  // tesseract_motion_planners::ChainOmplInterface::Ptr coi_;
  tesseract_kinematics::KinematicGroup::ConstPtr kin;
  tesseract_planning::JointWaypointPoly start_waypoint;
  tesseract_planning::JointWaypointPoly goal_waypoint;
  tesseract_environment::Environment::Ptr tesseract_;
  // tesseract_planning::OmplPlanParameters params_;
  Husky_IK::Option ik_option_;
  int inv_attp_max_;
  OMPLPlanners::Planners planner_;

  // buffered pose information
  std::unordered_map<std::string, Eigen::Isometry3d> l_obj;
  std::unordered_map<std::string, double> j_obj;

  // BaseBias base_bias;
  // tesseract_kinematics::ForwardKinematics::ConstPtr kin_base;
  // tesseract_kinematics::ForwardKinematics::ConstPtr kin_arm;
  // std::vector<double> fixed_goal;
  // Eigen::Matrix2d limits;
};

}  // namespace vkc

#endif