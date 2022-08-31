#include <KMeansRexCore.h>
#include <fmt/ranges.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>

#include <queue>

namespace vkc {

LongHorizonSeedGenerator::LongHorizonSeedGenerator(int n_steps, int n_iter,
                                                   size_t window_size)
    : n_steps(n_steps), n_iter(n_iter), window_size(window_size) {}

void LongHorizonSeedGenerator::generate(VKCEnvBasic &raw_vkc_env,
                                        std::vector<ActionBase::Ptr> &actions) {
  CONSOLE_BRIDGE_logDebug("\n\rgenerating long horizon seed\n");
  std::vector<ActionBase::Ptr> sub_actions(
      actions.begin(), actions.begin() + std::min(window_size, actions.size()));
  std::string origin_ee = raw_vkc_env.getEndEffectorLink();
  // if (sub_actions.size() <= 1) {
  //   CONSOLE_BRIDGE_logDebug(
  //       "sub actions length <= 1, removing joint candidates");
  //   if (sub_actions.size()) sub_actions[0]->joint_candidate =
  //   Eigen::VectorXd(); return;
  // }

  ProbGenerator prob_generator;
  VKCEnvBasic::Ptr vkc_env = std::move(raw_vkc_env.clone());
  auto env = vkc_env->getVKCEnv()->getTesseract();
  std::cout << fmt::format("{}", env->getGroupNames()).c_str() << std::endl;
  auto current_state = env->getCurrentJointValues();

  // get action iks
  tesseract_kinematics::IKSolutions filtered_ik_result;
  std::vector<tesseract_kinematics::IKSolutions> act_iks;
  for (auto &action : sub_actions) {
    CONSOLE_BRIDGE_logDebug("processing action %s",
                            action->getActionName().c_str());
    tesseract_kinematics::KinematicGroup::Ptr kin_group =
        std::move(env->getKinematicGroup(action->getManipulatorID()));
    // fmt::print("kin group joint names: {}", kin_group->getJointNames());
    auto wp = prob_generator.genMixedWaypoint(*vkc_env, action);
    CONSOLE_BRIDGE_logDebug("mixed waypoint for long horizon seed generated");
    filtered_ik_result.clear();

    // get collision free ik for action
    if (action->joint_candidates.size()) {
      filtered_ik_result = action->joint_candidates;
    } else {
      auto ik_result = tesseract_planning::getIKs(kin_group, wp, "world");

      CONSOLE_BRIDGE_logDebug("long horizon ik_result num: %ld",
                              ik_result.size());
      filtered_ik_result =
          tesseract_planning::filterCollisionIK(env, kin_group, ik_result);
      CONSOLE_BRIDGE_logDebug("ik after filtering collision: %ld",
                              filtered_ik_result.size());
      action->joint_candidates = filtered_ik_result;
    }
    act_iks.push_back(filtered_ik_result);

    vkc_env->updateEnv(kin_group->getJointNames(), filtered_ik_result.at(0),
                       action);
    CONSOLE_BRIDGE_logDebug(
        "long horizon udpate env success, processing next action...");
    auto kg = vkc_env->getVKCEnv()->getTesseract()->getKinematicGroup(
        action->getManipulatorID());
    // std::cout << "get kg success" << std::endl;
    // fmt::print("kin group joints after update: {}\n", kg->getJointNames());
  }

  std::cout << "getting ordered ik set...";
  Eigen::VectorXd coeff(9);
  coeff.setOnes();
  // coeff(2) = 0;
  auto ik_sets = getOrderedIKSet(current_state, act_iks, coeff);
  std::cout << "done." << std::endl;
  assert(ik_sets.front().size() == sub_actions.size());
  sub_actions.front()->joint_candidates.clear();
  for (auto ik_set : ik_sets) {
    if (std::find(sub_actions.front()->joint_candidates.begin(),
                  sub_actions.front()->joint_candidates.end(),
                  ik_set.front()) ==
        sub_actions.front()->joint_candidates.end()) {
      sub_actions.front()->joint_candidates.push_back(ik_set.front());
    }
  }
  std::cout << sub_actions.front()->joint_candidates.size() << std::endl;
  CONSOLE_BRIDGE_logInform("generating long horizon seed success");
  raw_vkc_env.setEndEffector(origin_ee);
  // const tesseract_srdf::KinematicsInformation kin_info =
  //     raw_vkc_env.getVKCEnv()
  //         ->getTesseract()
  //         ->getKinematicsInformation()
  //         .clone();
  // auto cmd = std::make_shared<AddKinematicsInformationCommand>(kin_info);
  // raw_vkc_env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  raw_vkc_env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  return;
}

std::vector<tesseract_kinematics::IKSolutions>
LongHorizonSeedGenerator::getOrderedIKSet(
    const Eigen::VectorXd current_state,
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks,
    const Eigen::VectorXd cost_coeff) {
  std::priority_queue<IKSetWithCost, std::vector<IKSetWithCost>,
                      std::greater<IKSetWithCost>>
      ik_set_queue;
  std::vector<tesseract_kinematics::IKSolutions> set_input;
  for (auto &act_ik : act_iks) {
    auto filtered_iks = kmeans(act_ik, 100);
    // CONSOLE_BRIDGE_logDebug("filtered iks length after kmeans: %d",
    //                         filtered_iks[0].size());
    set_input.push_back(filtered_iks);
  }
  // used push back before, so we need to reverse the ik sequence back
  std::reverse(set_input.begin(), set_input.end());
  auto sets = CartesianProduct(set_input);
  double lowest_cost = 10000;
  for (const auto &ik_set : sets) {
    double cost = getIKSetCost(current_state, ik_set, cost_coeff);
    if (cost > 0) ik_set_queue.emplace(ik_set, cost);
  }
  std::vector<tesseract_kinematics::IKSolutions> sets_result;

  while (ik_set_queue.size() > 0) {
    sets_result.push_back(ik_set_queue.top().ik_set);
    // std::cout << ik_set_queue.top().cost << std::endl;
    ik_set_queue.pop();
  }
  return sets_result;
}

double LongHorizonSeedGenerator::getIKSetCost(
    const Eigen::VectorXd current_state,
    const std::vector<Eigen::VectorXd> &act_ik_set,
    const Eigen::VectorXd cost_coeff) {
  const int coeff_length = cost_coeff.size();
  assert(current_state.size() >= cost_coeff.size());
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
  // std::cout << data_all.format(CleanFmt) << "\n";

  Eigen::ArrayXXd mu = Eigen::ArrayXXd::Zero(K, n_features);
  Eigen::ArrayXd z = Eigen::ArrayXd::Zero(n_examples_ttl);

  if (K < 10){
    CONSOLE_BRIDGE_logWarn("K: %d, maybe too small", K);
  }
  // CONSOLE_BRIDGE_logDebug("running kmeans with k: %d", K);
  RunKMeans(data_all.data(), n_examples_ttl, n_features, K, n_iters, seed,
            "plusplus", mu.data(), z.data());
  // std::cout << "done.\n";

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