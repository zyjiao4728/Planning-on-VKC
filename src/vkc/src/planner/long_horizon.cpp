#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>

namespace vkc {

LongHorizonSeedGenerator::LongHorizonSeedGenerator(int n_steps, int n_iter,
                                                   size_t window_size)
    : n_steps(n_steps), n_iter(n_iter), window_size(window_size) {}

void LongHorizonSeedGenerator::generate(VKCEnvBasic &raw_vkc_env,
                                        std::vector<ActionBase::Ptr> &actions) {
  ProbGenerator prob_generator;
  std::shared_ptr<VKCEnvBasic> vkc_env = std::move(raw_vkc_env.clone());
  auto env = vkc_env->getVKCEnv()->getTesseract();
  vkc_env->updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  std::vector<ActionBase::Ptr> sub_actions(
      actions.begin(), actions.begin() + std::min(window_size, actions.size()));
  for (auto &action : sub_actions) {
    // std::cout << action->seed.empty() << std::endl;
    tesseract_kinematics::KinematicGroup::Ptr kin_group =
        std::move(env->getKinematicGroup(action->getManipulatorID()));

    // auto ik_result = tesseract_planning::getIKWithHeuristic();
    // auto filtered_ik_result =
    //     tesseract_planning::filterCollisionIK(env, kin_group, ik_result);
    // vkc_env->updateEnv(seed_traj.back().joint_names,
    // seed_traj.back().position,
    //                action);
  }
  std::cout << vkc_env->getEndEffectorLink() << std::endl;
  return;
}

std::vector<tesseract_kinematics::IKSolutions>
LongHorizonSeedGenerator::getBestIKSet(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks) {}

}  // namespace vkc