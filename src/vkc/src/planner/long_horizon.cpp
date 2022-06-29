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
  std::vector<ActionBase::Ptr> sub_actions(
      actions.begin(), actions.begin() + std::min(window_size, actions.size()));
  if (sub_actions.size() <= 1) {
    CONSOLE_BRIDGE_logDebug(
        "sub actions length <= 1, removing joint candidates");
    if (sub_actions.size()) sub_actions[0]->joint_candidate = Eigen::VectorXd();
    return;
  } 
  ProbGenerator prob_generator;
  std::shared_ptr<VKCEnvBasic> vkc_env = std::move(raw_vkc_env.clone());
  auto env = vkc_env->getVKCEnv()->getTesseract();
  vkc_env->updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  auto current_state = env->getCurrentJointValues();

  std::vector<tesseract_kinematics::IKSolutions> act_iks;
  for (auto &action : sub_actions) {
    tesseract_kinematics::KinematicGroup::Ptr kin_group =
        std::move(env->getKinematicGroup(action->getManipulatorID()));
    auto wp = prob_generator.genMixedWaypoint(*vkc_env, action);
    CONSOLE_BRIDGE_logDebug("mixed waypoint generated");
    Eigen::VectorXd coeff(kin_group->getJointNames().size());
    coeff.setOnes();
    coeff(2) = 0;
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
  Eigen::VectorXd coeff(9);
  coeff.setOnes();
  coeff(2) = 0;
  auto ik_set = getBestIKSet(current_state, act_iks, coeff);
  assert(ik_set.size() == sub_actions.size());
  for (int i = 0; i < sub_actions.size(); i++) {
    ik_set[i](2) = current_state(2);
    sub_actions[i]->joint_candidate = ik_set[i];
  }

  return;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::getBestIKSet(
    const Eigen::VectorXd current_state,
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks,
    const Eigen::VectorXd cost_coeff) {
  std::vector<Eigen::VectorXd> best_ik_set;
  std::vector<tesseract_kinematics::IKSolutions> set_input;
  for (auto &act_ik : act_iks) {
    auto filtered_iks = kmeans(act_ik, 100);
    // CONSOLE_BRIDGE_logDebug("filtered iks length after kmeans: %d",
    //                         filtered_iks[0].size());
    set_input.push_back(filtered_iks);
  }
  std::reverse(set_input.begin(), set_input.end());
  auto sets = CartesianProduct(set_input);
  double lowest_cost = 10000;
  for (const auto &ik_set : sets) {
    double cost = getIKSetCost(current_state, ik_set, cost_coeff);
    if (cost < lowest_cost) {
      lowest_cost = cost;
      best_ik_set = ik_set;
    }
  }
  std::cout << "best ik set\n";
  for (const auto &ik : best_ik_set) {
    std::cout << ik.transpose() << std::endl;
  }
  std::cout << "best ik set cost: " << lowest_cost << std::endl;
  // exit(0);
  return best_ik_set;
}

double LongHorizonSeedGenerator::getIKSetCost(
    const Eigen::VectorXd current_state,
    const std::vector<Eigen::VectorXd> &act_ik_set,
    const Eigen::VectorXd cost_coeff) {
  const int coeff_length = cost_coeff.size();
  assert(act_ik_set.size() > 0 && act_ik_set[0].size() >= coeff_length);
  double cost =
      (act_ik_set[0].head(coeff_length) - current_state.head(coeff_length))
          .cwiseProduct(cost_coeff)
          .array()
          .abs()
          .sum();
  for (int i = 0; i < act_ik_set.size() - 1; i++) {
    assert(act_ik_set[i].size() >= coeff_length);
    cost += (act_ik_set[i].head(coeff_length) -
             act_ik_set[i + 1].head(coeff_length))
                .cwiseProduct(cost_coeff)
                .array()
                .abs()
                .sum();
  }
  return cost;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::kmeans(
    const tesseract_kinematics::IKSolutions &act_iks, int k) {
  const Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");
  int K = std::min(int(act_iks.size() * 0.8), k);
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

  std::cout << "running kmeans with k: " << K << std::endl;
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