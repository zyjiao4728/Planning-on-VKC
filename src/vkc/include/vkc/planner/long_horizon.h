#ifndef VKC_LONG_HORIZON_H
#define VKC_LONG_HORIZON_H

#include <tesseract_common/macros.h>
#include <vkc/action/action_base.h>
#include <vkc/env/vkc_env_basic.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

namespace vkc {
class LongHorizonSeedGenerator {
 public:
  LongHorizonSeedGenerator(int n_steps, int n_iter, size_t window_size);
  /**
   * @brief generate long horizon seeds inside actions
   *
   */
  void generate(VKCEnvBasic &env, std::vector<ActionBase::Ptr> &actions);

  tesseract_kinematics::IKSolutions getBestIKSet(
      const std::vector<tesseract_kinematics::IKSolutions> &act_iks);

  tesseract_kinematics::IKSolutions kmeans(
      const tesseract_kinematics::IKSolutions &act_iks, int k);
  double getIKSetCost(const tesseract_kinematics::IKSolutions &act_ik_set);

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