#ifndef VKC_LONG_HORIZON_H
#define VKC_LONG_HORIZON_H

#include <tesseract_common/macros.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <vkc/action/action_base.h>
#include <vkc/env/vkc_env_basic.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

namespace vkc {

struct IKSetWithCost {
  tesseract_kinematics::IKSolutions ik_set;
  double cost;
  IKSetWithCost(tesseract_kinematics::IKSolutions ik_set, double cost)
      : ik_set{std::move(ik_set)}, cost{cost} {}
  friend bool operator<(IKSetWithCost const &ik1, IKSetWithCost const &ik2) {
    return ik1.cost < ik2.cost;
  }
  friend bool operator>(IKSetWithCost const &ik1, IKSetWithCost const &ik2) {
    return ik1.cost > ik2.cost;
  }
};

class LongHorizonSeedGenerator {
 public:
  LongHorizonSeedGenerator(int n_steps, int n_iter, size_t window_size,
                           size_t robot_vkc_length);
  /**
   * @brief generate long horizon seeds inside actions
   *
   */
  void generate(VKCEnvBasic &env, std::vector<ActionBase::Ptr> &actions);

  std::vector<tesseract_kinematics::IKSolutions> getOrderedIKSet(
      const Eigen::VectorXd current_state,
      const std::vector<tesseract_kinematics::IKSolutions> &act_iks,
      const std::vector<ActionBase::Ptr> actions);

  tesseract_kinematics::IKSolutions kmeans(
      const tesseract_kinematics::IKSolutions &act_iks, int k);

  double getIKSetCost(const tesseract_kinematics::IKSolutions &act_ik_set,
                      const std::vector<Eigen::VectorXd> cost_coeffs);

  std::vector<std::vector<Eigen::VectorXd>> getValidIKSets(
      std::vector<std::vector<Eigen::VectorXd>> &act_iks,
      const std::vector<ActionBase::Ptr> &actions);

  void getValidIKSetsHelper_(
      std::vector<std::vector<Eigen::VectorXd>> &accum,
      std::vector<Eigen::VectorXd> stack,
      std::vector<std::vector<Eigen::VectorXd>> sequences, int index,
      const std::vector<ActionBase::Ptr> &actions);

  void setMapInfo(int x, int y, double resolution);

  void initAstarMap(ActionBase::Ptr action,
                    tesseract_collision::DiscreteContactManager::Ptr
                        discrete_contact_manager);

  bool astarChecking(ActionBase::Ptr action, Eigen::VectorXd start,
                     Eigen::VectorXd end);
  int n_steps;
  int n_iter;
  size_t window_size;
  tesseract_planning::MapInfo map_;
  size_t robot_vkc_length_;
};
std::vector<tesseract_kinematics::IKSolutions> CartesianProduct(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks);
void CartesianRecurse_(std::vector<tesseract_kinematics::IKSolutions> &accum,
                       tesseract_kinematics::IKSolutions stack,
                       std::vector<tesseract_kinematics::IKSolutions> sequences,
                       int index);
};  // namespace vkc

#endif