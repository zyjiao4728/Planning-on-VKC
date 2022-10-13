
#include <fmt/ranges.h>
#include <tesseract_motion_planners/3mo/3mo_motion_planner.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <vkc/planner/traj_init.h>
//#include "vkc/planner/prob_translator.h"

#define UNUSED(x) (void)(x)
const std::string DEFAULT_VKC_GROUP_ID = "vkc";

using namespace tesseract_planning;
namespace vkc {

void generateLongHorizonSeeds(VKCEnvBasic &env,
                              std::vector<ActionBase::Ptr> actions,
                              size_t window_size) {
  auto cloned_env = std::move(env.clone());
  auto sub_actions = {actions.begin(),
                      actions.begin() + std::min(window_size, actions.size())};
  for (auto &action : sub_actions) {
  }
}

bool isEmptyCell(
    tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
    std::string link_name, Eigen::Isometry3d &tf,
    tesseract_collision::ContactResultMap &contact_results) {
  discrete_contact_manager->setCollisionObjectsTransform(link_name, tf);
  discrete_contact_manager->contactTest(
      contact_results, tesseract_collision::ContactTestType::ALL);
  for (auto &collision : contact_results) {
    // std::cout << collision.first.first << ":\t" << collision.first.second <<
    // std::endl;
    if (collision.first.first == "base_link" ||
        collision.first.second == "base_link") {
      return false;
    }
  }
  return true;
}

void initBaseTrajectory(VKCEnvBasic &env,
                        std::vector<LinkDesiredPose> &base_pose, MapInfo &map) {
  int map_x = map.map_x;
  int map_y = map.map_y;
  double step_size = map.step_size;
  std::string base_link_name = "base_link";

  Eigen::Isometry3d base_start =
      env.getVKCEnv()->getTesseract()->getLinkTransform(base_link_name);
  Eigen::Isometry3d base_end = base_pose.back().tf;

  int base_x =
      int(round((base_start.translation()[0] + map_x / 2.0) / step_size));
  int base_y =
      int(round((base_start.translation()[1] + map_y / 2.0) / step_size));

  int end_x = int(round((base_end.translation()[0] + map_x / 2.0) / step_size));
  int end_y = int(round((base_end.translation()[1] + map_y / 2.0) / step_size));

  // std::cout << base_x << ":\t" << base_y << std::endl;
  // std::cout << end_x << ":\t" << end_y << std::endl;

  Eigen::Isometry3d base_tf;
  tesseract_collision::ContactResultMap contact_results;
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_ =
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->clone();

  // discrete_contact_manager_->setContactDistanceThreshold(0.05);

  std::vector<std::string> link_names =
      env.getVKCEnv()->getTesseract()->getKinematicGroup("vkc")->getLinkNames();

  for (auto &link_name : link_names) {
    if (link_name != base_link_name && link_name != "world") {
      if (!discrete_contact_manager_->removeCollisionObject(link_name)) {
        // ROS_WARN("Unable to remove collision object: %s", link_name.c_str());
      }
    }
  }

  AStar::Generator astar_generator;
  astar_generator.setWorldSize({map.grid_size_x, map.grid_size_y});
  astar_generator.setHeuristic(AStar::Heuristic::euclidean);
  astar_generator.setDiagonalMovement(true);

  for (int x = 0; x < map.grid_size_x; ++x) {
    for (int y = 0; y < map.grid_size_y; ++y) {
      base_tf.setIdentity();
      contact_results.clear();
      base_tf.translation() = Eigen::Vector3d(
          -map_x / 2.0 + x * step_size, -map_y / 2.0 + y * step_size, 0.13);
      if (!isEmptyCell(discrete_contact_manager_, base_link_name, base_tf,
                       contact_results) &&
          (!(x == base_x && y == base_y) && !(x == end_x && y == end_y))) {
        // std::cout << "o";
        // std::cout << x << ":\t" << y << std::endl;
        astar_generator.addCollision({x, y});
      } else if (x == base_x && y == base_y) {
        // std::cout << "S";
      } else if (x == end_x && y == end_y) {
        // std::cout << "G";
      } else {
        // std::cout << "+";
      }
    }
    // std::cout << "" << std::endl;
  }

  base_pose.clear();

  auto path = astar_generator.findPath({base_x, base_y}, {end_x, end_y});
  // auto path = astar_generator.findPath({ base_x, base_y }, {
  // int(round((4.17791 + map_x / 2.0) / step_size)),  int(round((-0.325272 +
  // map_y / 2.0) / step_size)) });
  for (auto &coordinate : path) {
    // std::cout << -map_x / 2.0 + coordinate.x * step_size << ":\t" << -map_y
    // / 2.0 + coordinate.y * step_size
    //           << std::endl;
    // std::cout << coordinate.x << ":\t" << coordinate.y << std::endl;
    Eigen::Isometry3d base_target;
    base_target.setIdentity();
    base_target.translation() =
        Eigen::Vector3d(-map_x / 2.0 + coordinate.x * step_size,
                        -map_y / 2.0 + coordinate.y * step_size, 0.13);
    base_pose.push_back(LinkDesiredPose(base_link_name, base_target));
    // std::cout << -map_x/2.0 + coordinate.x * step_size << " " << -map_y/2.0 +
    // coordinate.y * step_size << "\n";
  }
  return;
}

void initFinalJointSeed(std::unordered_map<std::string, int> &joint_name_idx,
                        std::vector<JointDesiredPose> &joint_objectives,
                        Eigen::VectorXd &seed) {
  if (joint_objectives.size() <= 0) {
    return;
  }

  for (auto &joint_obj : joint_objectives) {
    if (joint_name_idx.find(joint_obj.joint_name) != joint_name_idx.end()) {
      if (joint_name_idx.at(joint_obj.joint_name) < 0 ||
          joint_name_idx.at(joint_obj.joint_name) > seed.size()) {
        ROS_WARN("[%s]joint_name: %s, joint_index: %d, seed size: %ld",
                 __func__, joint_obj.joint_name.c_str(),
                 joint_name_idx.at(joint_obj.joint_name), seed.size());
      }

      seed[joint_name_idx.at(joint_obj.joint_name)] = joint_obj.joint_angle;
      // joint_name_idx[joint_obj.joint_name] = -1;
    } else {
      ROS_ERROR("[%s]joint_name: %s, joint_index: %d, seed size: %ld", __func__,
                joint_obj.joint_name.c_str(),
                joint_name_idx.at(joint_obj.joint_name), seed.size());
    }
  }
}

bool checkJointLimit(Eigen::VectorXd &sol, const Eigen::MatrixX2d &joint_limit,
                     size_t num_joint) {
  UNUSED(joint_limit);
  UNUSED(num_joint);

  // std::cout << joint_limit << std::endl;
  for (int i = 3; i < 6; ++i) {
    // if (i == 2)
    // {
    //   if (sol(i) < joint_limit(i, 0) - 0.15 || sol(i) > joint_limit(i, 1) +
    //   0.15)
    //     return false;
    // }
    // if (sol(i) < joint_limit(i, 0) + 0.1 || sol(i) > joint_limit(i, 1) - 0.1)
    //   return false;
    // if (i == 3 || i == 4 || i == 5 || i == 6 || i == 7)
    if (i >= 3) {
      // if (sol(i) < -3.14 + 0.1 || sol(i) > 3.14 - 0.1)
      if (sol(i) < joint_limit(i, 0) + 0.1 ||
          sol(i) > joint_limit(i, 1) - 0.1) {
        return false;
      }
    }
  }
  return true;
}

double interpolate(std::vector<LinkDesiredPose> base_pose,
                   std::vector<double> remap, unsigned int i, bool x) {
  double idx = remap[i];
  double idx_1 = 0;
  double rem = modf(idx, &idx_1);
  double idx_2 = idx_1 + 1;

  size_t int_idex_1 = size_t(idx_1);
  size_t int_idx_2 = size_t(idx_2);

  // ROS_WARN("remap[i]: %f, index_1: %f~%d, , index_2: %f~%d",
  //          idx, idx_1, (int)idx_1, idx_2, (int)idx_2);
  double waypts_len = pow(pow(base_pose[int_idx_2].tf.translation()[0] -
                                  base_pose[int_idex_1].tf.translation()[0],
                              2.0) +
                              pow(base_pose[int_idx_2].tf.translation()[1] -
                                      base_pose[int_idex_1].tf.translation()[1],
                                  2.0),
                          0.5);
  double grad_x = (base_pose[int_idx_2].tf.translation()[0] -
                   base_pose[int_idex_1].tf.translation()[0]) /
                  waypts_len;
  double grad_y = (base_pose[int_idx_2].tf.translation()[1] -
                   base_pose[int_idex_1].tf.translation()[1]) /
                  waypts_len;

  // if (x) {
  //   std::cout << "idx:\t" << idx << ",\tidx_1:\t" << int_idex_1 <<
  //   ",\tidx_2:\t" << int_idx_2 << ",\trem:\t" << rem << std::endl; std::cout
  //   << base_pose[int_idex_1].tf.translation()[0] + grad_x * rem * waypts_len
  //   << std::endl;
  // }
  if (x)
    return base_pose[int_idex_1].tf.translation()[0] +
           grad_x * rem * waypts_len;
  else
    return base_pose[int_idex_1].tf.translation()[1] +
           grad_y * rem * waypts_len;
}

std::vector<double> initIK(VKCEnvBasic &env,
                           std::vector<LinkDesiredPose> &link_objectives,
                           std::vector<JointDesiredPose> &joint_objectives,
                           MapInfo map, int n_steps) {
  UNUSED(n_steps);

  // srand(time(NULL));

  int max_iter = 1000;

  tesseract_kinematics::KinematicGroup::UPtr vkc_kin_group =
      env.getVKCEnv()->getTesseract()->getKinematicGroup(DEFAULT_VKC_GROUP_ID);

  tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->clone();

  for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects()) {
    disc_cont_mgr_->enableCollisionObject(active_link);
  }

  tesseract_collision::ContactResultMap contact_results;
  std::vector<LinkDesiredPose> base_pose;
  Eigen::VectorXd sol;
  Eigen::VectorXd seed;
  sol.resize(vkc_kin_group->numJoints());
  seed.resize(vkc_kin_group->numJoints());
  sol.setZero();
  seed.setZero();

  std::vector<std::string> joint_names;
  joint_names = vkc_kin_group->getJointNames();

  int idx = 0;
  std::unordered_map<std::string, int> joint_name_idx;
  for (auto &jnt : joint_names) {
    // std::cout << jnt << std::endl;
    joint_name_idx[jnt] = idx;
    ++idx;
  }

  // std::cout <<
  // inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName()
  // << std::endl;

  int inv_iter = 0;
  bool inv_suc = false;
  long unsigned int satisfy_collision = 1;
  bool satisfy_limit = false;
  bool desired_base_pose = false;

  if (link_objectives.size() > 0) {
    for (auto &link_obj : link_objectives) {
      if (link_obj.link_name == "base_link") {
        base_pose.clear();
        base_pose.push_back(link_obj);
        desired_base_pose = true;
        initBaseTrajectory(env, base_pose, map);
      } else if (link_obj.link_name ==
                 vkc_kin_group->getAllPossibleTipLinkNames()[0]) {
        initFinalJointSeed(joint_name_idx, joint_objectives, seed);

        Eigen::MatrixX2d joint_limits = vkc_kin_group->getLimits().joint_limits;
        // std::cout << joint_limits << std::endl;

        while (satisfy_collision > 0 || inv_iter == 0 || !inv_suc ||
               !satisfy_limit) {
          ++inv_iter;

          for (auto &jnt : joint_name_idx) {
            if (jnt.second >= 0)  // jnt.second != 2 &&
            {
              if (jnt.second == 4 || jnt.second == 6 || jnt.second == 7) {
                seed[jnt.second] = rand() % (int(joint_limits(jnt.second, 1))) -
                                   int(0.5 * joint_limits(jnt.second, 1));
              } else {
                seed[jnt.second] =
                    rand() % (2 * int(joint_limits(jnt.second, 1))) -
                    int(joint_limits(jnt.second, 1));
              }
            }
          }

          auto ik_input = new tesseract_kinematics::KinGroupIKInput(
              link_obj.tf, "world", link_obj.link_name);

          tesseract_kinematics::IKSolutions sol =
              vkc_kin_group->calcInvKin(*ik_input, seed);
          inv_suc = !sol.empty();

          auto env_state =
              env.getVKCEnv()->getTesseract()->getState(joint_names, sol[0]);
          contact_results.clear();
          disc_cont_mgr_->setCollisionObjectsTransform(
              env_state.link_transforms);

          disc_cont_mgr_->contactTest(
              contact_results, tesseract_collision::ContactTestType::ALL);

          satisfy_collision = contact_results.size();

          satisfy_limit =
              checkJointLimit(sol[0], vkc_kin_group->getLimits().joint_limits,
                              size_t(vkc_kin_group->numJoints()));

          if (inv_iter > max_iter - 1) {
            ROS_WARN("[%s]Exceed max inv kin iter!", __func__);
            break;
          }
        }
        // std::cout << "Iteration:\t" << inv_iter << std::endl;
        base_pose.clear();
        Eigen::Isometry3d base_final_pose;
        base_final_pose.setIdentity();
        base_final_pose.translation() = Eigen::Vector3d(sol(0), sol(1), 0.13);
        base_pose.push_back(LinkDesiredPose("base_link", base_final_pose));
        initBaseTrajectory(env, base_pose, map);
      } else {
        ROS_WARN(
            "Unsupported trajectory initialization for given link desired "
            "pose:\t%s",
            link_obj.link_name.c_str());
      }
    }
  }

  // std::cout << "this is IK solution" << std::endl;
  // std::cout << sol << std::endl;
  std::vector<double> res(sol.data(), sol.data() + sol.size());
  return res;
}

trajopt::TrajArray initTrajectory(
    VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
    std::vector<JointDesiredPose> &joint_objectives, MapInfo map,
    trajopt::TrajArray &init_traj, int n_steps, const std::string &robot) {
  // srand(time(NULL));
  ROS_WARN(
      "[%s]init trajectory, link_objectives size: %ld, joint_objective size: "
      "%ld",
      __func__, link_objectives.size(), joint_objectives.size());

  for (auto &objective : link_objectives) {
    std::cout << __func__ << ": link objective" << std::endl
              << "\tname: " << objective.link_name << std::endl
              << "\ttransform: " << std::endl
              << objective.tf.linear() << std::endl
              << "\ttranslation: " << std::endl
              << objective.tf.translation().transpose() << std::endl;
  }

  tesseract_kinematics::KinematicGroup::UPtr robot_kin_group =
      env.getVKCEnv()->getTesseract()->getKinematicGroup(robot);
  // tesseract_kinematics::InverseKinematics::Ptr inv_kin =
  // env.getVKCEnv()->getTesseract()->getInvKinematicsManager()->getInvKinematicSolver(robot);
  tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->clone();

  for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects()) {
    disc_cont_mgr_->enableCollisionObject(active_link);
  }

  tesseract_collision::ContactResultMap contact_results;
  std::vector<LinkDesiredPose> base_pose;
  tesseract_kinematics::IKSolutions sol;
  Eigen::VectorXd seed;
  // sol.resize(robot_kin_group->numJoints());
  seed.resize(robot_kin_group->numJoints());
  sol.clear();
  seed.setZero();

  std::unordered_map<std::string, int> joint_name_idx;
  int idx = 0;

  std::vector<std::string> joint_names;
  joint_names = robot_kin_group->getJointNames();

  for (auto &jnt : joint_names) {
    // std::cout << jnt << std::endl;
    joint_name_idx[jnt] = idx;
    ++idx;
  }

  // std::cout << inv_kin->getLimits() << std::endl;

  int inv_iter = 0;
  bool inv_suc = false;
  size_t satisfy_collision = 1;
  bool satisfy_limit = false;
  bool desired_base_pose = false;

  Eigen::VectorXd best_sol;
  size_t last_collision_count = 100000;

  if (link_objectives.size() > 0) {
    int max_iter = 500;
    for (auto &link_obj : link_objectives) {
      if (link_obj.link_name == "base_link") {
        ROS_WARN("[%s]init trajectory for base link...", __func__);
        base_pose.clear();
        base_pose.push_back(link_obj);
        desired_base_pose = true;
        initBaseTrajectory(env, base_pose, map);
        initFinalJointSeed(joint_name_idx, joint_objectives, sol[0]);
      } else if (link_obj.link_name ==
                 robot_kin_group->getAllPossibleTipLinkNames()[0]) {
        initFinalJointSeed(joint_name_idx, joint_objectives, seed);
        Eigen::MatrixX2d joint_limits =
            robot_kin_group->getLimits().joint_limits;
        std::cout << "joint limits: " << std::endl
                  << joint_limits.transpose() << std::endl;

        Eigen::VectorXd sol_cand;  // candiated solution of current best
        unsigned int last_sol_collision_cnt = 0;
        bool last_sol_inv = false;
        bool last_limit_stisfy = false;

        while (satisfy_collision > 0 || !inv_suc || !satisfy_limit) {
          ROS_WARN("[%s]iterating count: %d!", __func__, inv_iter);
          if (inv_iter++ >= max_iter) {
            ROS_WARN("[%s]Exceed max inv kin iter: %d!", __func__, max_iter);
            break;
          }

          for (auto &jnt : joint_name_idx) {
            // wanglei @2021-11-26
            // wanglei @2021-11-26
            // wanglei @2021-11-26
            // theory: seed = joint_lower_limit + double_rand *
            // (joint_upper_limit - joint_lower_limit) that's seed s.t.
            // [joint_lower_limit + 0.1, joint_upper_limit - 0.1]
            seed[jnt.second] =
                joint_limits(jnt.second, 0) + 0.1 +
                (double(rand()) /
                 double((RAND_MAX))) *  // get a float value ranging in [0, 1]
                                        // with the highest precision
                    (joint_limits(jnt.second, 1) - joint_limits(jnt.second, 0) -
                     0.2);  // range of current joint limits
          }
          auto ik_input = new tesseract_kinematics::KinGroupIKInput(
              link_obj.tf, "world", link_obj.link_name);

          sol = robot_kin_group->calcInvKin(*ik_input, seed);
          inv_suc = !sol.empty();
          if (!inv_suc) {
            ROS_WARN("[%s]no inverse kinematics solution is found!", __func__);
            continue;
          }

          // check limits voilation
          satisfy_limit =
              checkJointLimit(sol[0], robot_kin_group->getLimits().joint_limits,
                              robot_kin_group->numJoints());
          if (!satisfy_limit) {
            ROS_WARN("[%s]inverse kinematics solution voilates limits!",
                     __func__);
            continue;
          }

          // check collision
          contact_results.clear();
          auto env_state =
              env.getVKCEnv()->getTesseract()->getState(joint_names, sol[0]);
          disc_cont_mgr_->setCollisionObjectsTransform(
              env_state.link_transforms);
          disc_cont_mgr_->contactTest(
              contact_results, tesseract_collision::ContactTestType::ALL);
          satisfy_collision = contact_results.size();

          if (satisfy_collision < last_collision_count) {
            ROS_WARN(
                "[%s]best solutioni updated! current collision: %ld, last "
                "collision: %ld",
                __func__, satisfy_collision, last_collision_count);
            best_sol = sol[0];
            last_collision_count = satisfy_collision;
          }

          if (satisfy_collision > 0) {
            ROS_WARN("[%s]totoal %ld collision points detected:", __func__,
                     satisfy_collision);
            for (auto &collision : contact_results) {
              std::cout << "\t" << collision.first.first << " -->|<-- "
                        << collision.first.second << std::endl;
            }
          }

          std::cout << "sol: " << sol[0].transpose() << std::endl;
        }

        // we choose the best among the all the failure cases, and this includes
        // the case of last success
        if (best_sol.size() > 0) {
          sol[0] = best_sol;
        }

        base_pose.clear();
        Eigen::Isometry3d base_final_pose;
        base_final_pose.setIdentity();
        if (inv_suc && satisfy_limit && (0 == satisfy_collision)) {
          ROS_INFO("[%s]init trajectory sucessfully!", __func__);
          // the third argument here is the z-offset of base frame referred to
          // world frame
          base_final_pose.translation() =
              Eigen::Vector3d(sol[0](0), sol[0](1), 0.13);
        } else {
          bool init_base_position = false;
          int idx = 0;
          std::vector<std::string> base_joints(
              {"base_y_base_x", "base_theta_base_y"});
          Eigen::VectorXd base_values;
          base_values.resize(2);
          base_values.setZero();
          while (!init_base_position && idx++ < 100) {
            init_base_position = true;

            double r = (rand() % 100) / 100.0 * 0.1;  // origional value: 0.7
            double a = (rand() % 100) / 100.0 * 6.18;

            base_values[0] = link_obj.tf.translation()[0] + r * cos(a);
            base_values[1] = link_obj.tf.translation()[1] - r * sin(a);

            auto env_state = env.getVKCEnv()->getTesseract()->getState(
                base_joints, base_values);
            contact_results.clear();
            disc_cont_mgr_->setCollisionObjectsTransform(
                env_state.link_transforms);
            disc_cont_mgr_->contactTest(
                contact_results, tesseract_collision::ContactTestType::ALL);
            for (auto &collision : contact_results) {
              if (collision.first.first == "base_link" ||
                  collision.first.second == "base_link") {
                init_base_position = false;
                break;
              }
            }
            contact_results.clear();
          }
          base_final_pose.translation() =
              Eigen::Vector3d(base_values[0], base_values[1], 0.13);
        }

        base_pose.push_back(LinkDesiredPose("base_link", base_final_pose));
        initBaseTrajectory(env, base_pose, map);
      } else {
        ROS_WARN(
            "Unsupported trajectory initialization for given tip link: %s, "
            "actural tip link: %s",
            link_obj.link_name.c_str(),
            robot_kin_group->getAllPossibleTipLinkNames()[0].c_str());
        return init_traj;
      }
    }
  } else {
    // TODO: check this case and provide an safe solution
    ROS_WARN(
        "@%s:%u: this case shouldnot happed, this means an invalid goal being "
        "set.",
        __func__, __LINE__);
    return init_traj;
  }

  std::vector<double> nsteps_remap;
  std::cout << "solution: " << std::endl;
  for (int j = 0; j < robot_kin_group->numJoints(); ++j) {
    std::cout << sol[j] << " ";
  }
  std::cout << std::endl;

  for (int i = 0; i < n_steps; ++i) {
    nsteps_remap.push_back(i / 1.0 / (n_steps - 1) * (base_pose.size() - 1));
  }
  std::reverse(base_pose.begin(), base_pose.end());
  for (int i = 0; i < n_steps; ++i) {
    if (i == 0) continue;

    init_traj(i, 0) = interpolate(base_pose, nsteps_remap, i, true);
    init_traj(i, 1) = interpolate(base_pose, nsteps_remap, i, false);

    if (std::isnan(init_traj(i, 0)) || std::isnan(init_traj(i, 1))) {
      init_traj(i, 0) = init_traj(i - 1, 0);
      init_traj(i, 1) = init_traj(i - 1, 1);
    }

    for (int j = 2; j < robot_kin_group->numJoints(); ++j) {
      if ((sol[0][j] - init_traj(0, j)) > M_PI) {
        sol[0][j] -= 2 * M_PI;
      } else if ((sol[0][j] - init_traj(0, j)) < -M_PI) {
        sol[0][j] += 2 * M_PI;
      }
      init_traj(i, j) =
          init_traj(0, j) + (sol[0][j] - init_traj(0, j)) * i / (n_steps - 1);
    }

    if (i == n_steps - 1) {
      init_traj(i, 0) = base_pose.back().tf.translation()[0];
      init_traj(i, 1) = base_pose.back().tf.translation()[1];
    }
  }

  // std::cout << "Initial trajectory:" << std::endl;
  // std::cout << init_traj << std::endl;
  return init_traj;
}

CompositeInstruction generateMixedSeed(
    const CompositeInstruction &instructions,
    const tesseract_scene_graph::SceneState &current_state,
    const tesseract_environment::Environment::ConstPtr &env, int min_steps,
    const std::pair<std::string, std::string> base_joint,
    const Eigen::VectorXd &ik_cost_coeff) {
  // srand(time(NULL));
  CONSOLE_BRIDGE_logDebug("generating mixed seed");
  PlannerRequest request;
  request.instructions = instructions;
  request.env_state = current_state;
  request.env = env;
  PlannerResponse response;

  tesseract_planning::MMMOMotionPlanner planner;
  auto profile = std::make_shared<MMMOPlannerPlanProfile>(
      min_steps, 5 * M_PI / 180, 0.1, 5 * M_PI / 180);
  profile->setMapInfo(10, 10, 0.2);
  profile->setBaseJoint(base_joint);
  CONSOLE_BRIDGE_logDebug(
      fmt::format("current joint names: {}", env->getActiveJointNames())
          .c_str());

  if (ik_cost_coeff.size()) {
    std::stringstream ss;
    ss << ik_cost_coeff.transpose();
    CONSOLE_BRIDGE_logInform("setting ik cost coeff: %s", ss.str().c_str());
    profile->cost_coeff = ik_cost_coeff;
  }

  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<MMMOPlannerPlanProfile>(
      planner.getName(), instructions.getProfile(), profile);
  auto flat = flattenProgram(instructions);
  for (const auto &i : flat) {
    if (i.get().isMoveInstruction())
      profiles->addProfile<MMMOPlannerPlanProfile>(
          planner.getName(), i.get().as<MoveInstructionPoly>().getProfile(),
          profile);
  }
  request.profiles = profiles;
  planner.solve(request, response);
  // response.results.print("mixed seed: ");
  CONSOLE_BRIDGE_logDebug("mixed seed generation success");
  return response.results;
}

}  // namespace vkc
