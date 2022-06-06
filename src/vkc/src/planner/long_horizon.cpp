#include <tesseract_command_language/utils/utils.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>

namespace vkc {

LongHorizonSeedGenerator::LongHorizonSeedGenerator() {}

void LongHorizonSeedGenerator::generate(VKCEnvBasic &env,
                                        std::vector<ActionBase::Ptr> &actions,
                                        int n_steps, int n_iter,
                                        size_t window_size) {
  ProbGenerator prob_generator;
  std::shared_ptr<VKCEnvBasic> cloned_env = std::move(env.clone());
  std::vector<ActionBase::Ptr> sub_actions(
      actions.begin(), actions.begin() + std::min(window_size, actions.size()));
  for (auto &action : sub_actions) {
    // std::cout << action->seed.empty() << std::endl;
    auto request =
        prob_generator.genRequest(*cloned_env, action, n_steps,
                                  n_iter);  // TODO: refactor generation process
    action->seed = request.seed;
    tesseract_common::JointTrajectory seed_traj =
        toJointTrajectory(action->seed);
    cloned_env->updateEnv(seed_traj.back().joint_names,
                          seed_traj.back().position, action);
  }
  std::cout << env.getEndEffectorLink() << std::endl;
  std::cout << cloned_env->getEndEffectorLink() << std::endl;
  return;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::getIK(ManipulatorInfo manip){
    auto limits = ;
}

}  // namespace vkc