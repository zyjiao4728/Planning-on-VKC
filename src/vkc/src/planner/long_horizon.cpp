#include <KMeansRexCore.h>
#include <fmt/ranges.h>
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
  CONSOLE_BRIDGE_logDebug("generating long horizon seed");
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
    CONSOLE_BRIDGE_logDebug("mixed waypoint generated");
    Eigen::VectorXd coeff(kin_group->getJointNames().size());
    coeff.setOnes();
    coeff(0) = 0;
    coeff(1) = 0;
    // CONSOLE_BRIDGE_logDebug(
    //     fmt::format("{}", kin_group->getJointNames()).c_str());
    // std::cout
    //     << env->getCurrentJointValues(kin_group->getJointNames()).transpose()
    //     << std::endl;
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
    CONSOLE_BRIDGE_logDebug("udpate env success, processing next action...");
  }
  std::cout << "getting best ik set" << std::endl;
  auto ik_set = getBestIKSet(act_iks);
  assert(ik_set.size() == sub_actions.size());
  for (int i = 0; i < sub_actions.size(); i++) {
    sub_actions[i]->joint_candidate = ik_set[i];
  }

  return;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::getBestIKSet(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks) {
  std::vector<Eigen::VectorXd> best_ik_set;
  std::vector<tesseract_kinematics::IKSolutions> set_input;
  for (auto &act_ik : act_iks) {
    auto filtered_iks = kmeans(act_ik, 50);
    CONSOLE_BRIDGE_logDebug("filtered iks length after kmeans: %d",
                            filtered_iks[0].size());
    set_input.push_back(filtered_iks);
  }
  std::reverse(set_input.begin(), set_input.end());
  auto sets = CartesianProduct(set_input);
  double lowest_cost = 10000;
  for (auto &ik_set : sets) {
    double cost = getIKSetCost(ik_set);
    if (cost < lowest_cost) {
      lowest_cost = cost;
      best_ik_set = ik_set;
    }
  }
  for (const auto &ik : best_ik_set) {
    std::cout << ik.transpose() << std::endl;
  }
  std::cout << "get ik set success" << std::endl;
  // exit(0);
  return best_ik_set;
}

double LongHorizonSeedGenerator::getIKSetCost(
    const std::vector<Eigen::VectorXd> &act_ik_set) {
  double cost = 0.0;
  for (int i = 0; i < act_ik_set.size() - 1; i++) {
    cost +=
        (act_ik_set[i].head(9) - act_ik_set[i + 1].head(9)).array().abs().sum();
  }
  return cost;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::kmeans(
    const tesseract_kinematics::IKSolutions &act_iks, int k) {
  const Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");
  int K = std::min(int(act_iks.size() * 0.6), k);
  int n_features = act_iks[0].rows();
  int n_iters = 400;
  int seed = 42;
  int n_examples_ttl = act_iks.size();
  Eigen::ArrayXXd data_all(n_examples_ttl, n_features);

  for (int i = 0; i < act_iks.size(); i++) {
    data_all.row(i) = act_iks[i].transpose();
  }
  assert(data_all.rows() == act_iks.size());

  // Display the data
  // std::cout << "data_all\n";
  // std::cout << data_all.format(CleanFmt) << "\n";

  Eigen::ArrayXXd mu = Eigen::ArrayXXd::Zero(K, n_features);
  Eigen::ArrayXd z = Eigen::ArrayXd::Zero(n_examples_ttl);

  std::cout << "running kmeans ... \n";
  RunKMeans(data_all.data(), n_examples_ttl, n_features, K, n_iters, seed,
            "plusplus", mu.data(), z.data());
  std::cout << "done.\n";

  // std::cout << "estimated clusters:\n";
  // std::cout << mu.format(CleanFmt) << "\n";

  // get closest ik for each cluster
  tesseract_kinematics::IKSolutions closest;
  closest.resize(K);
  Eigen::ArrayXd best_dist = z;
  best_dist.fill(1000);

  for (int i = 0; i < data_all.rows(); i++) {
    double dist_ = (data_all.row(i) - mu.row(z(i))).matrix().norm();
    if (dist_ < best_dist(z(i))) {
      best_dist(z(i)) = dist_;
      closest[z(i)] = data_all.row(i).transpose();
    }
  }

  // CONSOLE_BRIDGE_logDebug("closest found");

  return closest;
}

void CartesianRecurse_(std::vector<tesseract_kinematics::IKSolutions> &accum,
                       tesseract_kinematics::IKSolutions stack,
                       std::vector<tesseract_kinematics::IKSolutions> sequences,
                       int index) {
  tesseract_kinematics::IKSolutions sequence = sequences[index];
  for (auto &i : sequence) {
    stack.push_back(i);
    if (index == 0)
      accum.push_back(stack);
    else
      CartesianRecurse_(accum, stack, sequences, index - 1);
    stack.pop_back();
  }
}

std::vector<tesseract_kinematics::IKSolutions> CartesianProduct(
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks) {
  std::vector<tesseract_kinematics::IKSolutions> accum;
  tesseract_kinematics::IKSolutions stack;
  if (act_iks.size() > 0)
    CartesianRecurse_(accum, stack, act_iks, act_iks.size() - 1);
  return accum;
}
}  // namespace vkc