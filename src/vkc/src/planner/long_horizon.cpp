#include <KMeansRexCore.h>
#include <fmt/ranges.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>

#include <AStar.hpp>
#include <queue>

const int K = 20;

namespace vkc {

LongHorizonSeedGenerator::LongHorizonSeedGenerator(int n_iter,
                                                   size_t window_size,
                                                   size_t robot_vkc_length)
    : n_iter(n_iter),
      window_size(window_size),
      robot_vkc_length_(robot_vkc_length) {
  map_ = tesseract_planning::MapInfo();
}

void LongHorizonSeedGenerator::generate(VKCEnvBasic &raw_vkc_env,
                                        std::vector<ActionBase::Ptr> &actions) {
  CONSOLE_BRIDGE_logDebug("\n=====generating long horizon seed=====\n");
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
  // std::cout << fmt::format("get group names: {}",
  // env->getGroupNames()).c_str()
  //           << std::endl;
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
      filtered_ik_result = tesseract_planning::getIKs(
          env, kin_group,
          env->getCurrentJointValues(kin_group->getJointNames()), wp, "world");

      CONSOLE_BRIDGE_logDebug("long horizon ik_result num: %ld",
                              filtered_ik_result.size());
      // filtered_ik_result =
      //     tesseract_planning::filterCollisionIK(env, kin_group, ik_result);
      // CONSOLE_BRIDGE_logDebug("ik after filtering collision: %ld",
      //                         filtered_ik_result.size());
      action->joint_candidates = filtered_ik_result;
    }

    // check action astar map
    if (!action->astar_init) {
      tesseract_collision::DiscreteContactManager::Ptr
          discrete_contact_manager = std::move(vkc_env->getVKCEnv()
                                                   ->getTesseractNonInverse()
                                                   ->getDiscreteContactManager()
                                                   ->clone());
      CONSOLE_BRIDGE_logDebug("initializing astar generator...");
      if (action->getActionType() == ActionType::PlaceAction) {
        auto place_action = std::static_pointer_cast<PlaceAction>(action);
        auto detach_object =
            vkc_env->getAttachLocation(place_action->getDetachedObject());
        if (detach_object->fixed_base) {
          CONSOLE_BRIDGE_logDebug(
              "place action with fixed base object found, disabling collision: "
              "%s for object to avoid astar problems",
              detach_object->link_name_.c_str());
          auto scene_graph_ =
              vkc_env->getVKCEnv()->getTesseractNonInverse()->getSceneGraph();
          auto outbound_joints =
              scene_graph_->getOutboundJoints(detach_object->base_link_);
          discrete_contact_manager->disableCollisionObject(
              detach_object->base_link_);
          while (outbound_joints.size()) {
            for (int i = outbound_joints.size() - 1; i >= 0; i--) {
              discrete_contact_manager->disableCollisionObject(
                  outbound_joints[i]->child_link_name);
              auto child_outbound_joints = scene_graph_->getOutboundJoints(
                  outbound_joints[i]->child_link_name);
              outbound_joints.insert(outbound_joints.end(),
                                     child_outbound_joints.begin(),
                                     child_outbound_joints.end());
              outbound_joints.erase(outbound_joints.begin() + i);
            }
          }
        }
      }
      setupAstarGenerator(action->astar_generator, discrete_contact_manager,
                          map_, "base_link",
                          env->getLinkTransform("base_link").translation()[2]);
      // initAstarMap(action, discrete_contact_manager);
      // std::cout << "door joint value: "
      //           << vkc_env->getVKCEnv()
      //                  ->getTesseractNonInverse()
      //                  ->getCurrentJointValues(
      //                      {"fridge_0001_dof_rootd_Aa002_r_joint"})
      //           << std::endl;
      CONSOLE_BRIDGE_logDebug("init astar map success");
    }
    act_iks.push_back(filtered_ik_result);

    vkc_env->updateEnv(kin_group->getJointNames(), filtered_ik_result.at(0),
                       action);

    if (action->getActionType() == ActionType::PlaceAction &&
        action->getJointObjectives().size()) {
      CONSOLE_BRIDGE_logDebug(
          "place action with joint objectives found, updating specific "
          "joints...");
      for (auto &jo : action->getJointObjectives()) {
        Eigen::VectorXd v;
        v.resize(1);
        v[0] = jo.second;
        CONSOLE_BRIDGE_logDebug("updating %s with %f", jo.first.c_str(),
                                jo.second);
        vkc_env->getVKCEnv()->getTesseract()->setState({jo.first}, v);
      }
    }

    std::stringstream ss;
    ss << filtered_ik_result.at(0).transpose();
    CONSOLE_BRIDGE_logDebug(
        "long horizon udpate env with %s success, processing next action...",
        ss.str().c_str());
  }

  CONSOLE_BRIDGE_logDebug("getting ordered ik set...");

  auto ik_sets = getOrderedIKSet(current_state, act_iks, sub_actions);
  std::cout << "done." << std::endl;
  assert(ik_sets.size() != 0);
  assert(ik_sets.front().size() == sub_actions.size());
  sub_actions.front()->joint_candidates.clear();
  for (auto ik_set : ik_sets) {
    bool is_diff = true;
    for (auto ik : sub_actions.front()->joint_candidates) {
      if ((ik_set.front() - ik).matrix().norm() < 0.05) {
        is_diff = false;
        break;
      }
    }
    if (is_diff) {
      sub_actions.front()->joint_candidates.push_back(ik_set.front());
    }
    // if (std::find(sub_actions.front()->joint_candidates.begin(),
    //               sub_actions.front()->joint_candidates.end(),
    //               ik_set.front()) ==
    //     sub_actions.front()->joint_candidates.end()) {
    //   // ik not exist for this action
    //   sub_actions.front()->joint_candidates.push_back(ik_set.front());
    // }
  }
  std::cout << "first 10 ik candidates: " << std::endl;
  for (int i = 0;
       i < min(10, (int)sub_actions.front()->joint_candidates.size()); i++) {
    std::cout << sub_actions.front()->joint_candidates[i].transpose()
              << std::endl;
  }
  CONSOLE_BRIDGE_logDebug("candidates for first action: %d",
                          sub_actions.front()->joint_candidates.size());
  CONSOLE_BRIDGE_logInform("generating long horizon seed success");
  raw_vkc_env.setEndEffector(origin_ee);
  raw_vkc_env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  return;
}

std::vector<tesseract_kinematics::IKSolutions>
LongHorizonSeedGenerator::getOrderedIKSet(
    const Eigen::VectorXd current_state,
    const std::vector<tesseract_kinematics::IKSolutions> &act_iks,
    const std::vector<ActionBase::Ptr> actions) {
  std::priority_queue<IKSetWithCost, std::vector<IKSetWithCost>,
                      std::greater<IKSetWithCost>>
      ik_set_queue;
  std::vector<tesseract_kinematics::IKSolutions> set_input;

  set_input.insert(set_input.begin(), {current_state});

  for (auto &act_ik : act_iks) {
    auto filtered_iks = kmeans(act_ik, K);
    // auto filtered_iks = act_ik;
    CONSOLE_BRIDGE_logDebug("filtered iks after kmeans: %d - %d/%d",
                            filtered_iks[0].size(), filtered_iks.size(),
                            act_ik.size());
    set_input.push_back(filtered_iks);
  }
  // used push back before, so we need to reverse the ik sequence back
  // std::reverse(set_input.begin(), set_input.end());
  // auto sets = CartesianProduct(set_input);
  // no need to reverse for new function
  // std::cout << set_input.size() << std::endl;
  auto sets = getValidIKSets(set_input, actions);
  // std::cout << sets.front().size() << std::endl;
  if (sets.size() == 0)
    throw std::runtime_error("no valid sets found for given ik set input.");
  double lowest_cost = 10000;

  // generate cost coeffs
  std::vector<Eigen::VectorXd> cost_coeffs;
  for (const auto &act : actions) {
    if (act->getIKCostCoeff().size())
      cost_coeffs.push_back(act->getIKCostCoeff());
    else {
      Eigen::VectorXd coeff_;
      coeff_.setOnes(robot_vkc_length_);
      coeff_[3] = 5.;
      coeff_[0] = 2.;
      coeff_[1] = 2.;
      coeff_[2] = 0.5;
      cost_coeffs.push_back(coeff_);
    }
  }

  for (const auto &ik_set : sets) {
    double cost = getIKSetCost(ik_set, cost_coeffs);
    if (cost > 0) ik_set_queue.emplace(ik_set, cost);
  }

  std::vector<tesseract_kinematics::IKSolutions> sets_result;

  while (ik_set_queue.size() > 0) {
    auto set = ik_set_queue.top().ik_set;
    set.erase(set.begin());
    sets_result.push_back(set);  // small to large
    // std::cout << ik_set_queue.top().cost << std::endl;
    ik_set_queue.pop();
  }
  return sets_result;
}

double LongHorizonSeedGenerator::getIKSetCost(
    const std::vector<Eigen::VectorXd> &act_ik_set,
    const std::vector<Eigen::VectorXd> cost_coeffs) {
  assert(cost_coeffs.size() > 0);
  assert(act_ik_set.size() == cost_coeffs.size() + 1);
  std::stringstream ss;
  double cost = 0;
  for (int i = 0; i < act_ik_set.size() - 1; i++) {
    assert(cost_coeffs[i].size() >= robot_vkc_length_);
    cost += (act_ik_set[i].head(robot_vkc_length_) -
             act_ik_set[i + 1].head(robot_vkc_length_))
                .cwiseProduct(cost_coeffs[i].head(robot_vkc_length_))
                .array()
                .abs()
                .sum();
  }
  return cost;
}

std::vector<std::vector<Eigen::VectorXd>>
LongHorizonSeedGenerator::getValidIKSets(
    std::vector<std::vector<Eigen::VectorXd>> &act_iks,
    const std::vector<ActionBase::Ptr> &actions) {
  std::vector<std::vector<Eigen::VectorXd>> accum;
  std::vector<Eigen::VectorXd> stack;
  CONSOLE_BRIDGE_logDebug("getting valid ik sets...");
  if (act_iks.size() > 0)
    getValidIKSetsHelper_(accum, stack, act_iks, 0, actions);
  CONSOLE_BRIDGE_logDebug("valid ik sets: %ld", accum.size());
  return accum;
}

void LongHorizonSeedGenerator::getValidIKSetsHelper_(
    std::vector<std::vector<Eigen::VectorXd>> &accum,
    std::vector<Eigen::VectorXd> stack,
    std::vector<std::vector<Eigen::VectorXd>> sequences, int index,
    const std::vector<ActionBase::Ptr> &actions) {
  tesseract_kinematics::IKSolutions sequence = sequences[index];
  for (auto &ik : sequence) {
    if (index > 0 && !astarChecking(actions[index - 1], stack.back(), ik))
      continue;
    stack.push_back(ik);
    if (index == sequences.size() - 1)
      accum.push_back(stack);
    else
      getValidIKSetsHelper_(accum, stack, sequences, index + 1, actions);
    stack.pop_back();
  }
}

void LongHorizonSeedGenerator::setMapInfo(int x, int y, double resolution) {
  map_ = tesseract_planning::MapInfo(x, y, resolution);
}

void LongHorizonSeedGenerator::initAstarMap(
    ActionBase::Ptr action,
    tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager) {
  action->astar_generator.setWorldSize({map_.grid_size_x, map_.grid_size_y});
  action->astar_generator.setHeuristic(AStar::Heuristic::euclidean);
  action->astar_generator.setDiagonalMovement(true);

  // add collisions
  tesseract_collision::ContactResultMap contact_results;
  Eigen::Isometry3d base_tf;
  for (int x = 0; x < map_.grid_size_x; ++x) {
    for (int y = 0; y < map_.grid_size_y; ++y) {
      base_tf.setIdentity();
      contact_results.clear();
      base_tf.translation() =
          Eigen::Vector3d(-map_.map_x / 2.0 + x * map_.step_size,
                          -map_.map_y / 2.0 + y * map_.step_size, 0.02);
      if (!tesseract_planning::isEmptyCell(discrete_contact_manager,
                                           "base_link", base_tf,
                                           contact_results) /*&&
          (!(x == base_x && y == base_y) && !(x == end_x && y == end_y))*/) {
        // std::cout << "o";
        // std::cout << x << ":\t" << y << std::endl;
        action->astar_generator.addCollision({x, y});
        // } else if (x == base_x && y == base_y) {
        // std::cout << "S";
        // } else if (x == end_x && y == end_y) {
        // std::cout << "G";
      } else {
        // std::cout << "+";
      }
    }
    // std::cout << std::endl;
  }
  action->astar_generator.printMap({0, 0}, {0, 0});
  action->astar_init = true;
}

bool LongHorizonSeedGenerator::astarChecking(ActionBase::Ptr action,
                                             Eigen::VectorXd start,
                                             Eigen::VectorXd end) {
  if (start.size() == 0 || end.size() == 0) return false;
  assert(start.size() > 0);
  assert(end.size() > 0);
  int base_x = int(round((start[0] + map_.map_x / 2.0) / map_.step_size));
  int base_y = int(round((start[1] + map_.map_y / 2.0) / map_.step_size));

  int end_x = int(round((end[0] + map_.map_x / 2.0) / map_.step_size));
  int end_y = int(round((end[1] + map_.map_y / 2.0) / map_.step_size));
  bool start_collision =
      action->astar_generator.detectCollision({base_x, base_y});
  bool end_collision = action->astar_generator.detectCollision({end_x, end_y});
  // action->astar_generator.printMap({base_x, base_y}, {end_x, end_y});
  action->astar_generator.removeCollision({base_x, base_y});
  action->astar_generator.removeCollision({end_x, end_y});
  auto path =
      action->astar_generator.findPath({base_x, base_y}, {end_x, end_y});
  bool solution_found = true;
  if (path.front().x != end_x || path.front().y != end_y) {
    // cannot find valid astar path
    solution_found = false;
  }
  if (start_collision) action->astar_generator.addCollision({base_x, base_y});
  if (end_collision) action->astar_generator.addCollision({end_x, end_y});
  // std::cout << solution_found << "\t";
  return solution_found;
}

tesseract_kinematics::IKSolutions LongHorizonSeedGenerator::kmeans(
    const tesseract_kinematics::IKSolutions &act_iks, int k) {
  const Eigen::IOFormat CleanFmt(3, 0, " ", "\n", "[", "]");
  if (act_iks.size() < k) {
    return act_iks;
  }
  int K = std::min(int(act_iks.size()), k);
  int n_features = 3;
  // int n_features = act_iks[0].rows();
  int n_iters = 400;
  int seed = 42;
  int n_examples_ttl = act_iks.size();
  Eigen::ArrayXXd data_all(n_examples_ttl, act_iks[0].rows());

  for (int i = 0; i < act_iks.size(); i++) {
    data_all.row(i) = act_iks[i].transpose();
  }
  assert(data_all.rows() == act_iks.size());

  // Display the data
  // std::cout << data_all.format(CleanFmt) << "\n";

  Eigen::ArrayXXd mu = Eigen::ArrayXXd::Zero(K, n_features);
  Eigen::ArrayXd z = Eigen::ArrayXd::Zero(n_examples_ttl);

  if (K < 10) {
    CONSOLE_BRIDGE_logWarn("K: %d, maybe too small", K);
  }
  // CONSOLE_BRIDGE_logDebug("running kmeans with k: %d", K);
  RunKMeans(data_all.data(), n_examples_ttl, n_features, K, n_iters, seed,
            "plusplus", mu.data(), z.data());
  // std::cout << "done.\n";

  // std::cout << "estimated clusters:\n";
  // std::cout << mu.format(CleanFmt) << "\n";

  // get closest ik for each cluster
  CONSOLE_BRIDGE_logDebug("getting closest ik for each cluster...");
  tesseract_kinematics::IKSolutions closest;
  closest.resize(K);
  Eigen::ArrayXd best_dist = z;  // z: label for datas
  best_dist.fill(10000);

  for (int i = 0; i < data_all.rows(); i++) {
    double dist_ =
        (data_all.row(i).head(n_features) - mu.row(z(i))).matrix().norm();
    if (dist_ < best_dist(z(i))) {
      best_dist(z(i)) = dist_;
      // if (!data_all.row(i).size()) continue;
      closest[z(i)] = data_all.row(i).transpose();
    }
  }

  for (int i = 0; i < closest.size(); i++) {
    if (closest[i].size() == 0) {
      CONSOLE_BRIDGE_logWarn("size of ik %d in closest is zero!", i);
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