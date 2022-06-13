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
  std::vector<tesseract_kinematics::IKSolutions> act_iks;
  for (auto &action : sub_actions) {
    tesseract_kinematics::KinematicGroup::Ptr kin_group =
        std::move(env->getKinematicGroup(action->getManipulatorID()));
    auto wp = prob_generator.genMixedWaypoint(*vkc_env, action);
    std::cout << "mixed waypoint generated" << std::endl;
    Eigen::VectorXd coeff(kin_group->getJointNames().size());
    coeff.setOnes();
    coeff(0) = 0;
    coeff(1) = 0;
    std::cout << coeff << std::endl;
    auto ik_result = tesseract_planning::getIKWithOrder(
        kin_group, wp, "world",
        env->getCurrentJointValues(kin_group->getJointNames()), coeff);
    CONSOLE_BRIDGE_logDebug("long horizon ik_result num: %ld",
                            ik_result.size());
    auto filtered_ik_result =
        tesseract_planning::filterCollisionIK(env, kin_group, ik_result);
    act_iks.push_back(filtered_ik_result);
    vkc_env->updateEnv(kin_group->getJointNames(), filtered_ik_result.at(0),
                       action);
  }
  auto ik_set = getBestIKSet(act_iks);
  assert(ik_set.size() == sub_actions.size());
  for (int i = 0; i < sub_actions.size(); i++) {
    sub_actions[i]->joint_candidate = ik_set[i];
  }

  return;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::getBestIKSet(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks) {
  std::vector<Eigen::VectorXd> result;
  for (auto &act_ik : act_iks) {
    result.push_back(act_ik[0]);
  }
  return result;
}

}  // namespace vkc