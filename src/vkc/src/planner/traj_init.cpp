
#include <vkc/planner/traj_init.h>
//#include "vkc/planner/prob_translator.h"

#define UNUSED(x) (void)(x)
const std::string DEFAULT_VKC_GROUP_ID = "vkc";

namespace vkc
{

  bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager, std::string link_name,
                   Eigen::Isometry3d &tf, tesseract_collision::ContactResultMap &contact_results)
  {
    discrete_contact_manager->setCollisionObjectsTransform(link_name, tf);
    discrete_contact_manager->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);
    for (auto &collision : contact_results)
    {
      // std::cout << collision.first.first << ":\t" << collision.first.second << std::endl;
      if (collision.first.first == "base_link" || collision.first.second == "base_link")
      {
        return false;
      }
    }
    return true;
  }

  void initBaseTrajectory(VKCEnvBasic &env, std::vector<LinkDesiredPose> &base_pose, MapInfo &map)
  {
    int map_x = map.map_x;
    int map_y = map.map_y;
    double step_size = map.step_size;
    std::string base_link_name = "base_link";

    Eigen::Isometry3d base_start = env.getVKCEnv()->getTesseract()->getEnvironment()->getLinkTransform(base_link_name);
    Eigen::Isometry3d base_end = base_pose.back().tf;

    int base_x = int(round((base_start.translation()[0] + map_x / 2.0) / step_size));
    int base_y = int(round((base_start.translation()[1] + map_y / 2.0) / step_size));

    int end_x = int(round((base_end.translation()[0] + map_x / 2.0) / step_size));
    int end_y = int(round((base_end.translation()[1] + map_y / 2.0) / step_size));

    // std::cout << base_x << ":\t" << base_y << std::endl;
    // std::cout << end_x << ":\t" << end_y << std::endl;

    Eigen::Isometry3d base_tf;
    tesseract_collision::ContactResultMap contact_results;
    tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_ =
        env.getVKCEnv()->getTesseract()->getEnvironment()->getDiscreteContactManager()->clone();

    // discrete_contact_manager_->setContactDistanceThreshold(0.05);

    std::vector<std::string> link_names =
        env.getVKCEnv()->getTesseract()->getFwdKinematicsManagerConst()->getFwdKinematicSolver("vkc")->getLinkNames();

    for (auto &link_name : link_names)
    {
      if (link_name != base_link_name && link_name != "world")
      {
        if (!discrete_contact_manager_->removeCollisionObject(link_name))
        {
          // ROS_WARN("Unable to remove collision object: %s", link_name.c_str());
        }
      }
    }

    AStar::Generator astar_generator;
    astar_generator.setWorldSize({map.grid_size_x, map.grid_size_y});
    astar_generator.setHeuristic(AStar::Heuristic::euclidean);
    astar_generator.setDiagonalMovement(true);

    for (int x = 0; x < map.grid_size_x; ++x)
    {
      for (int y = 0; y < map.grid_size_y; ++y)
      {
        base_tf.setIdentity();
        contact_results.clear();
        base_tf.translation() = Eigen::Vector3d(-map_x / 2.0 + x * step_size, -map_y / 2.0 + y * step_size, 0.13);
        if (!isEmptyCell(discrete_contact_manager_, base_link_name, base_tf, contact_results) && (!(x == base_x && y == base_y) && !(x == end_x && y == end_y)))
        {
          // std::cout << "o";
          // std::cout << x << ":\t" << y << std::endl;
          astar_generator.addCollision({x, y});
        }
        else if (x == base_x && y == base_y)
        {
          // std::cout << "S";
        }
        else if (x == end_x && y == end_y)
        {
          // std::cout << "G";
        }
        else
        {
          // std::cout << "+";
        }
      }
      // std::cout << "" << std::endl;
    }

    base_pose.clear();

    auto path = astar_generator.findPath({base_x, base_y}, {end_x, end_y});
    // auto path = astar_generator.findPath({ base_x, base_y }, { int(round((4.17791 + map_x / 2.0) / step_size)),  int(round((-0.325272 + map_y / 2.0) / step_size)) });
    for (auto &coordinate : path)
    {
      // std::cout << -map_x / 2.0 + coordinate.x * step_size << ":\t" << -map_y / 2.0 + coordinate.y * step_size
      //           << std::endl;
      // std::cout << coordinate.x << ":\t" << coordinate.y << std::endl;
      Eigen::Isometry3d base_target;
      base_target.setIdentity();
      base_target.translation() =
          Eigen::Vector3d(-map_x / 2.0 + coordinate.x * step_size, -map_y / 2.0 + coordinate.y * step_size, 0.13);
      base_pose.push_back(LinkDesiredPose(base_link_name, base_target));
      // std::cout << -map_x/2.0 + coordinate.x * step_size << " " << -map_y/2.0 + coordinate.y * step_size << "\n";
    }
    return;
  }

  void initFinalJointSeed(std::unordered_map<std::string, int> &joint_name_idx,
                          std::vector<JointDesiredPose> &joint_objectives, Eigen::VectorXd &seed)
  {
    if (joint_objectives.size() > 0)
    {
      for (auto &joint_obj : joint_objectives)
      {
        if (joint_name_idx.find(joint_obj.joint_name) != joint_name_idx.end())
        {
          if (joint_name_idx.at(joint_obj.joint_name) < 0 || joint_name_idx.at(joint_obj.joint_name) > seed.size())
          {
            ROS_WARN("joint_name: %s, joint_index: %d, seed size: %d",
                     joint_obj.joint_name.c_str(),
                     joint_name_idx.at(joint_obj.joint_name),
                     seed.size());
          }

          seed[joint_name_idx.at(joint_obj.joint_name)] = joint_obj.joint_angle;
          // joint_name_idx[joint_obj.joint_name] = -1;
        }
        else
        {
            ROS_ERROR("joint_name: %s, joint_index: %d, seed size: %d",
                     joint_obj.joint_name.c_str(),
                     joint_name_idx.at(joint_obj.joint_name),
                     seed.size());
        }
      }
    }
  }

  bool checkJointLimit(Eigen::VectorXd &sol, const Eigen::MatrixX2d &joint_limit, size_t num_joint)
  {
    UNUSED(joint_limit);
    UNUSED(num_joint);

    // std::cout << joint_limit << std::endl;
    for (int i = 3; i < 6; ++i)
    {
      // if (i == 2)
      // {
      //   if (sol(i) < joint_limit(i, 0) - 0.15 || sol(i) > joint_limit(i, 1) + 0.15)
      //     return false;
      // }
      // if (sol(i) < joint_limit(i, 0) + 0.1 || sol(i) > joint_limit(i, 1) - 0.1)
      //   return false;
      if (i == 3 || i == 4 || i == 5 || i == 6 || i == 7)
      {
        if (sol(i) < -3.14 + 0.1 || sol(i) > 3.14 - 0.1)
        {
          return false;
        }
      }
    }
    return true;
  }

  double interpolate(std::vector<LinkDesiredPose> base_pose, std::vector<double> remap, int i, bool x)
  {
    double idx = remap[i];
    double idx_1 = 0;
    double rem = modf(idx, &idx_1);
    double idx_2 = idx_1 + 1;

    size_t int_idex_1 = size_t(idx_1);
    size_t int_idx_2 = size_t(idx_2);

    // ROS_WARN("remap[i]: %f, index_1: %f~%d, , index_2: %f~%d",
    //          idx, idx_1, (int)idx_1, idx_2, (int)idx_2);
    double waypts_len = pow(pow(base_pose[int_idx_2].tf.translation()[0] - base_pose[int_idex_1].tf.translation()[0], 2.0) +
                                pow(base_pose[int_idx_2].tf.translation()[1] - base_pose[int_idex_1].tf.translation()[1], 2.0),
                            0.5);
    double grad_x = (base_pose[int_idx_2].tf.translation()[0] - base_pose[int_idex_1].tf.translation()[0]) / waypts_len;
    double grad_y = (base_pose[int_idx_2].tf.translation()[1] - base_pose[int_idex_1].tf.translation()[1]) / waypts_len;

    // if (x) {
    //   std::cout << "idx:\t" << idx << ",\tidx_1:\t" << int_idex_1 << ",\tidx_2:\t" << int_idx_2 << ",\trem:\t" << rem << std::endl;
    //   std::cout << base_pose[int_idex_1].tf.translation()[0] + grad_x * rem * waypts_len << std::endl;
    // }
    if (x)
      return base_pose[int_idex_1].tf.translation()[0] + grad_x * rem * waypts_len;
    else
      return base_pose[int_idex_1].tf.translation()[1] + grad_y * rem * waypts_len;
  }

  std::vector<double> initIK(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
                             std::vector<JointDesiredPose> &joint_objectives, MapInfo map, int n_steps)
  {
    UNUSED(n_steps);

    // srand(time(NULL));

    int max_iter = 1000;

    tesseract::InverseKinematicsManager::Ptr inv_kin_mgr = env.getVKCEnv()->getTesseract()->getInvKinematicsManager();
    tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
        env.getVKCEnv()->getTesseract()->getEnvironment()->getDiscreteContactManager()->clone();

    for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects())
    {
      disc_cont_mgr_->enableCollisionObject(active_link);
    }

    tesseract_collision::ContactResultMap contact_results;
    std::vector<LinkDesiredPose> base_pose;
    Eigen::VectorXd sol;
    Eigen::VectorXd seed;
    sol.resize(inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());
    seed.resize(inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());
    sol.setZero();
    seed.setZero();

    std::vector<std::string> joint_names;
    joint_names = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getJointNames();

    int idx = 0;
    std::unordered_map<std::string, int> joint_name_idx;
    for (auto &jnt: joint_names)
    {
      // std::cout << jnt << std::endl;
      joint_name_idx[jnt] = idx;
      ++idx;
    }


    // std::cout << inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName() << std::endl;

    int inv_iter = 0;
    bool inv_suc = false;
    int satisfy_collision = 1;
    bool satisfy_limit = false;
    bool desired_base_pose = false;

    if (link_objectives.size() > 0)
    {
      for (auto &link_obj : link_objectives)
      {
        if (link_obj.link_name == "base_link")
        {
          base_pose.clear();
          base_pose.push_back(link_obj);
          desired_base_pose = true;
          initBaseTrajectory(env, base_pose, map);
        }
        else if (link_obj.link_name == inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName())
        {
          initFinalJointSeed(joint_name_idx, joint_objectives, seed);

          Eigen::MatrixX2d joint_limits = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getLimits();
          // std::cout << joint_limits << std::endl;

          while (satisfy_collision > 0 || inv_iter == 0 || !inv_suc || !satisfy_limit)
          {
            ++inv_iter;

            for (auto &jnt : joint_name_idx)
            {
              if (jnt.second >= 0) // jnt.second != 2 &&
              {
                if (jnt.second == 4 || jnt.second == 6 || jnt.second == 7)
                {
                  seed[jnt.second] = rand() % (int(joint_limits(jnt.second, 1))) - int(0.5 * joint_limits(jnt.second, 1));
                }
                else
                {
                  seed[jnt.second] = rand() % (2 * int(joint_limits(jnt.second, 1))) - int(joint_limits(jnt.second, 1));
                }
              }
            }
            inv_suc = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->calcInvKin(sol, link_obj.tf, seed);

            tesseract_environment::EnvState::Ptr env_state =
                env.getVKCEnv()->getTesseract()->getEnvironment()->getState(joint_names, sol);
            contact_results.clear();
            disc_cont_mgr_->setCollisionObjectsTransform(env_state->transforms);

            disc_cont_mgr_->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);

            satisfy_collision = contact_results.size();

            satisfy_limit = checkJointLimit(sol, inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getLimits(),
                                            inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());

            if (inv_iter > max_iter - 1)
            {
              ROS_WARN("Exceed max inv kin iter!");
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
        }
        else
        {
          ROS_WARN("Unsupported trajectory initialization for given link desired pose:\t%s", link_obj.link_name.c_str());
        }
      }
    }

    // std::cout << "this is IK solution" << std::endl;
    // std::cout << sol << std::endl;
    std::vector<double> res(sol.data(), sol.data() + sol.size());
    return res;
  }

  trajopt::TrajArray initTrajectory(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
                                    std::vector<JointDesiredPose> &joint_objectives, MapInfo map,
                                    trajopt::TrajArray &init_traj, int n_steps)
  {
    // srand(time(NULL));

    int max_iter = 1000;

    tesseract::InverseKinematicsManager::Ptr inv_kin_mgr = env.getVKCEnv()->getTesseract()->getInvKinematicsManager();
    tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
        env.getVKCEnv()->getTesseract()->getEnvironment()->getDiscreteContactManager()->clone();

    for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects())
    {
      disc_cont_mgr_->enableCollisionObject(active_link);
    }

    tesseract_collision::ContactResultMap contact_results;
    std::vector<LinkDesiredPose> base_pose;
    Eigen::VectorXd sol;
    Eigen::VectorXd seed;
    sol.resize(inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());
    seed.resize(inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());
    sol.setZero();
    seed.setZero();

    std::unordered_map<std::string, int> joint_name_idx;
    int idx = 0;

    std::vector<std::string> joint_names;
    joint_names = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getJointNames();

    for (auto &jnt : joint_names)
    {
      // std::cout << jnt << std::endl;
      joint_name_idx[jnt] = idx;
      ++idx;
    }

    // std::cout << inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getLimits() << std::endl;

    int inv_iter = 0;
    bool inv_suc = false;
    size_t satisfy_collision = 1;
    bool satisfy_limit = false;
    bool desired_base_pose = false;

    if (link_objectives.size() > 0)
    {
      for (auto &link_obj : link_objectives)
      {

        if (link_obj.link_name == "base_link")
        {
          base_pose.clear();
          base_pose.push_back(link_obj);
          desired_base_pose = true;
          initBaseTrajectory(env, base_pose, map);
          initFinalJointSeed(joint_name_idx, joint_objectives, sol);
        }
        else if (link_obj.link_name == inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName())
        {

          initFinalJointSeed(joint_name_idx, joint_objectives, seed);

          Eigen::MatrixX2d joint_limits = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getLimits();
          // std::cout << joint_limits << std::endl;

          while (satisfy_collision > 0 || inv_iter == 0 || !inv_suc || !satisfy_limit)
          {
            ++inv_iter;

            for (auto &jnt : joint_name_idx)
            {
              if (jnt.second >= 0) // jnt.second != 2 &&
              {
                if (jnt.second == 4 || jnt.second == 6 || jnt.second == 7)
                {
                  seed[jnt.second] = rand() % (int(joint_limits(jnt.second, 1))) - int(0.5 * joint_limits(jnt.second, 1));
                }
                else
                {
                  seed[jnt.second] = rand() % (2 * int(joint_limits(jnt.second, 1))) - int(joint_limits(jnt.second, 1));
                }
              }
            }
            inv_suc = inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->calcInvKin(sol, link_obj.tf, seed);

            tesseract_environment::EnvState::Ptr env_state =
                env.getVKCEnv()->getTesseract()->getEnvironment()->getState(joint_names, sol);
            contact_results.clear();
            disc_cont_mgr_->setCollisionObjectsTransform(env_state->transforms);

            disc_cont_mgr_->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);

            satisfy_collision = contact_results.size();
            satisfy_limit = checkJointLimit(sol, inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getLimits(),
                                            inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints());

            std::cout << satisfy_collision << ", " << inv_suc << ", " << satisfy_limit << std::endl;
            if (satisfy_collision > 0)
            {
              for (auto &collision : contact_results)
              {
                std::cout << collision.first.first << " and " << collision.first.second << " are in collision" << std::endl;
              }
              std::cout << "======================================================" << std::endl;
            }
            // std::cout << "sol: " << sol.transpose() << std::endl;
            if (inv_iter > max_iter - 1)
            {
              ROS_WARN("Exceed max inv kin iter!");
              break;
            }
          }
          // std::cout << "Iteration:\t" << inv_iter << std::endl;
          base_pose.clear();
          Eigen::Isometry3d base_final_pose;
          base_final_pose.setIdentity();
          if (inv_suc && (satisfy_collision == 0) && (satisfy_limit == 1))
          {
            // the third argument here is the z-offset of base frame referred to world frame
            base_final_pose.translation() = Eigen::Vector3d(sol(0), sol(1), 0.13);
          }
          else
          {
            bool init_base_position = false;
            int idx = 0;
            std::vector<std::string> base_joints({"base_y_base_x", "base_theta_base_y"});
            std::vector<double> base_values({0, 0});
            while (!init_base_position && idx < 100)
            {
              std::cout << idx << " ";
              idx += 1;
              init_base_position = true;

              double r = (rand() % 100) / 100.0 * 0.7;
              double a = (rand() % 100) / 100.0 * 6.18;

              base_values[0] = link_obj.tf.translation()[0] + r * cos(a);
              base_values[1] = link_obj.tf.translation()[1] - r * sin(a);

              tesseract_environment::EnvState::Ptr env_state =
                  env.getVKCEnv()->getTesseract()->getEnvironment()->getState(base_joints, base_values);
              contact_results.clear();
              disc_cont_mgr_->setCollisionObjectsTransform(env_state->transforms);
              disc_cont_mgr_->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);
              for (auto &collision : contact_results)
              {
                if (collision.first.first == "base_link" || collision.first.second == "base_link")
                {
                  init_base_position = false;
                  break;
                }
              }
              contact_results.clear();
            }
            base_final_pose.translation() = Eigen::Vector3d(base_values[0], base_values[1], 0.13);
          }
          base_pose.push_back(LinkDesiredPose("base_link", base_final_pose));
          initBaseTrajectory(env, base_pose, map);
        }
        else
        {
          ROS_WARN("Unsupported trajectory initialization for given link desired pose:\t%s, tiplink: %s",
                   link_obj.link_name.c_str(), inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName().c_str());
        }
      }
    }
    else
    {
      // TODO: check this case and provide an safe solution
      ROS_WARN("@%s:%u: this case shouldnot happed, this means an invalid goal being set.",
               __func__, __LINE__);
      return init_traj;
    }

    std::vector<double> nsteps_remap;

    for (int j = 0; j < inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints(); ++j)
    {
      std::cout << sol[j] << ", ";
    }

    for (int i = 0; i < n_steps; ++i)
    {
      nsteps_remap.push_back(i / 1.0 / (n_steps - 1) * (base_pose.size() - 1));
    }
    std::reverse(base_pose.begin(), base_pose.end());
    for (int i = 0; i < n_steps; ++i)
    {
      if (i == 0)
        continue;

      init_traj(i, 0) = interpolate(base_pose, nsteps_remap, i, true);
      init_traj(i, 1) = interpolate(base_pose, nsteps_remap, i, false);

      if (std::isnan(init_traj(i, 0)) || std::isnan(init_traj(i, 1)))
      {
        init_traj(i, 0) = init_traj(i - 1, 0);
        init_traj(i, 1) = init_traj(i - 1, 1);
      }

      for (int j = 2; j < inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->numJoints(); ++j)
      {
        if ((sol[j] - init_traj(0, j)) > M_PI)
        {
          sol[j] -= 2 * M_PI;
        }
        else if ((sol[j] - init_traj(0, j)) < -M_PI)
        {
          sol[j] += 2 * M_PI;
        }
        init_traj(i, j) = init_traj(0, j) + (sol[j] - init_traj(0, j)) * i / (n_steps - 1);
      }

      if (i == n_steps - 1)
      {
        init_traj(i, 0) = base_pose.back().tf.translation()[0];
        init_traj(i, 1) = base_pose.back().tf.translation()[1];
      }
    }
    //std::cout << "Initial trajectory:" << std::endl;
    //std::cout << init_traj << std::endl;
    return init_traj;
  }



    // trajopt::TrajArray initTrajectoryByOMPL(VKCEnvBasic &env, ActionBase::Ptr act, int n_steps)
    // {
    //     vkc::OmplPlanParameters param;
    //     param.planner = OMPLPlanners::Planners::RRT_Connect;
    //     param.ik_option = Husky_IK::Option::Full;

    //     param.inv_attp_max = 100;
    //     param.plan_params.n_steps = n_steps;
    //     param.plan_params.planning_time = 5.0;
    //     param.plan_params.simplify = true;

    //     ProbTranslator prob_translator(param);
    //     prob_translator.transProb(env, act);

    //     tesseract_motion_planners::PlannerResponse response;
    //     std::vector<std::vector<double>> res_traj;
    //     prob_translator.solveProblem(response, res_traj);

    //     trajopt::TrajArray init_traj;
    //     if (0 < response.status_code)
    //     {
    //         init_traj = response.trajectory;
    //     }
    //     else
    //     {
    //         std::cout << "[" << __func__ << "]failed to init trajectory" << std::endl;
    //     }

    //     return init_traj;
    // }
} // namespace vkc
