#ifndef VKC_LONG_HORIZON_H
#define VKC_LONG_HORIZON_H

#include <tesseract_common/macros.h>
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
  LongHorizonSeedGenerator(int n_steps, int n_iter, size_t window_size);
  /**
   * @brief generate long horizon seeds inside actions
   *
   */
  void generate(VKCEnvBasic &env, std::vector<ActionBase::Ptr> &actions);

  std::vector<tesseract_kinematics::IKSolutions> getOrderedIKSet(
      const Eigen::VectorXd current_state,
      const std::vector<tesseract_kinematics::IKSolutions> &act_iks,
      const Eigen::VectorXd cost_coeff);

  tesseract_kinematics::IKSolutions kmeans(
      const tesseract_kinematics::IKSolutions &act_iks, int k);

  double getIKSetCost(const Eigen::VectorXd current_state,
                      const tesseract_kinematics::IKSolutions &act_ik_set,
                      const Eigen::VectorXd cost_coeff);

  int n_steps;
  int n_iter;
  size_t window_size;
};
std::vector<tesseract_kinematics::IKSolutions> CartesianProduct(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks);
void CartesianRecurse_(std::vector<tesseract_kinematics::IKSolutions> &accum,
                       tesseract_kinematics::IKSolutions stack,
                       std::vector<tesseract_kinematics::IKSolutions> sequences,
                       int index);
};  // namespace vkc

#endif