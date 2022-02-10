#include <vkc/planner/prob_translator.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;

namespace vkc
{
  ProbTranslator::ProbTranslator(OmplPlanParameters params)
  {
    inv_attp_max_ = params.inv_attp_max;
    params_ = params.plan_params;
    ik_option_ = params.ik_option;
    planner_ = params.planner;
    // srand((unsigned int)time(NULL));   // if this expression is commented on, we use the default random seed 1
  }

  tesseract_motion_planners::JointWaypoint::Ptr ProbTranslator::setupStartWaypoint(VKCEnvBasic &env, std::string manipulator)
  {
    auto kin_ = env.getVKCEnv()->getTesseract()->getFwdKinematicsManager()->getFwdKinematicSolver(manipulator);
    // getFwdKinematicsManager(manipulator);

    EnvState::ConstPtr current_state = env.getVKCEnv()->getTesseractEnvironment()->getCurrentState();
    Eigen::VectorXd start_pos;
    start_pos.resize(kin_->numJoints());
    int joint_num = 0;
    // JointWaypoint(std::vector<double> joint_positions, std::vector<std::string> joint_names)

    for (const auto &j : kin_->getJointNames())
    {
      start_pos[joint_num] = current_state->joints.at(j);
      ++joint_num;
    }

    if(!checkJointLimits(start_pos, kin_->getLimits()))
    {
      ROS_ERROR("[%s]Oho, start waypoint is invalid!", __func__);
    }

    return std::make_shared<tesseract_motion_planners::JointWaypoint>(start_pos, kin_->getJointNames());
  }

  tesseract_kinematics::ForwardKinematics::ConstPtr ProbTranslator::getKinematics()
  {
    return kin;
  }

  tesseract_motion_planners::JointWaypoint::Ptr ProbTranslator::setupGoalWaypoint(VKCEnvBasic &env, std::string manipulator)
  {
    auto kin_ = env.getVKCEnv()->getTesseract()->getFwdKinematicsManager()->getFwdKinematicSolver(manipulator);
    if (l_obj.size())
    {
      if (j_obj.size())
      {
        ROS_ERROR("2 types of objectives detected, using link objectives only");
      }
      Eigen::VectorXd solutions;
      auto seed = (start_waypoint == nullptr) ? Eigen::VectorXd::Zero(kin_->getJointNames().size()) : start_waypoint->getPositions();
      auto pose = l_obj[0].tf;
      if (collisionFreeInverseKinematics(env, manipulator, solutions, pose, seed))
      {
        return std::make_shared<tesseract_motion_planners::JointWaypoint>(solutions, kin_->getJointNames());
      }
      ROS_ERROR("Failed to find collision free solution!");
    }
    else
    {
      if (kin_->numJoints() == j_obj.size())
      {
        Eigen::VectorXd solutions = Eigen::VectorXd::Zero(kin_->numJoints());
        int idx = 0;
        for (auto j : j_obj)
        {
          solutions(idx) = j.joint_angle;
          idx++;
        }

        return std::make_shared<tesseract_motion_planners::JointWaypoint>(solutions, kin_->getJointNames());
      }
      ROS_ERROR("Joint objectives detected size mismatched");
    }

    return nullptr;
  }

  void ProbTranslator::setupParams(VKCEnvBasic &env, vkc::ActionBase::Ptr act)
  {

    std::string name = "ompl";

    this->tesseract_ = env.getVKCEnv()->getTesseract();

    this->kin = env.getVKCEnv()->getTesseract()->getFwdKinematicsManager()->getFwdKinematicSolver(act->getManipulatorID());

    // // there has got to be a better way to do this
    // this->kin_base = env.getVKCEnv()->getTesseract()->getFwdKinematicsManager()->getFwdKinematicSolver("base");
    // this->kin_arm = env.getVKCEnv()->getTesseract()->getFwdKinematicsManager()->getFwdKinematicSolver("arm");

    return;
  }

  bool ProbTranslator::solveProblem(PlannerResponse &response, std::vector<std::vector<double>> &res_traj)
  {
    ROS_INFO("[%s]OMPL starts planning...", __func__);
    if (nullptr == start_waypoint || nullptr == goal_waypoint)
    {
      response.status_code = 0;
      ROS_WARN("[%s]invalid %s %s, OMPL terminate the motion plan!",
               __func__, 
               nullptr == start_waypoint ? "start state" : "",
               nullptr == goal_waypoint ? "goal state" : "");
      return false;
    }
    

    boost::optional<ompl::geometric::PathGeometric> maybe_path = coi_->plan(start_waypoint->getPositions(), goal_waypoint->getPositions(), params_);

    if (maybe_path)
    {
      response.trajectory = vkc::toTrajArray(*maybe_path);
      response.joint_names = kin->getJointNames();
      response.status_code = 1;

      ompl::geometric::PathGeometric path = *maybe_path;

      for (int i = 0; i < response.trajectory.rows(); ++i)
      {
        const double *begin = &response.trajectory.row(i).data()[0];
        res_traj.push_back(std::vector<double>(begin, begin + response.trajectory.cols()));
      }

      return true;
    }
    response.status_code = 0;
    ROS_WARN("No results from OMPL");
    return false;
  }

  bool ProbTranslator::transProb(VKCEnvBasic &env, vkc::ActionBase::Ptr act)
  {
    switch (act->getActionType())
    {
    case ActionType::PickAction:
    {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(act);
      // initFinalPose(env, std::vector<LinkDesiredPose>(), std::vector<JointDesiredPose>(), ActionType::PickAction);
      return transPickProb(env, pick_act);
    }

    case ActionType::GotoAction:
    {
      GotoAction::Ptr goto_act = std::dynamic_pointer_cast<GotoAction>(act);
      return transGotoProb(env, goto_act);
    }

    case ActionType::PlaceAction:
    {
      PlaceAction::Ptr place_act = std::dynamic_pointer_cast<PlaceAction>(act);
      return transPlaceProb(env, place_act);
    }

    case ActionType::UseAction:
    {
      UseAction::Ptr use_act = std::dynamic_pointer_cast<UseAction>(act);
      return transUseProb(env, use_act);
    }

    default:
    {
      ROS_ERROR("Undefined action type.");
      return false;
    }
    }
  }

  void ProbTranslator::insertPlanners(OMPLPlanners::Planners option)
  {
    switch (option)
    {
    case OMPLPlanners::Planners::RRT_Connect:
    {
      this->coi_->ss_->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(this->coi_->spaceInformation()));
    }
    break;
    case OMPLPlanners::Planners::RRT_Star:
    {
      this->coi_->ss_->setPlanner(std::make_shared<ompl::geometric::RRTstar>(this->coi_->spaceInformation()));
    }
    break;

    default:
      this->coi_->ss_->setPlanner(std::make_shared<ompl::geometric::RRT>(this->coi_->spaceInformation()));
      break;
    }

    return;
  }

  bool ProbTranslator::inverseKinematics(tesseract_kinematics::InverseKinematics::Ptr solver, Eigen::VectorXd &solutions, const Eigen::Isometry3d &pose, Eigen::VectorXd &seed)
  {
    return solver->calcInvKin(solutions, pose, seed);
  }

  void ProbTranslator::getJointNameIndexMap(tesseract_kinematics::ForwardKinematics::Ptr kin, std::unordered_map<std::string, int> &joint_name_idx)
  {
    joint_name_idx.clear();
    int idx = 0;
    auto joint_names = kin->getJointNames();
    for (auto &jnt : joint_names)
    {
      // std::cout << jnt << std::endl;
      joint_name_idx[jnt] = idx;
      ++idx;
    }
  }



  bool ProbTranslator::collisionFreeInverseKinematics(VKCEnvBasic &env, std::string manipulator, Eigen::VectorXd &solutions, const Eigen::Isometry3d &pose, Eigen::VectorXd &seed)
  {
    auto inv_kin_mgr = env.getVKCEnv()->getTesseract()->getInvKinematicsManager();
    auto inv_kin_solver = inv_kin_mgr->getInvKinematicSolver(manipulator);
    if (inv_kin_solver)
    {
      ROS_INFO("[%s]it will try %d times to get a collision free waypoint, ik solver name: %s",
               __func__, inv_attp_max_, inv_kin_solver->getSolverName().c_str());

      solutions.resize(inv_kin_mgr->getInvKinematicSolver(manipulator)->numJoints());
      seed.resize(inv_kin_mgr->getInvKinematicSolver(manipulator)->numJoints());

      auto state_og = env.getVKCEnv()->getTesseractEnvironment()->getCurrentState();
      auto kin_ = env.getVKCEnv()->getTesseract()->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
      auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
          env.getVKCEnv()->getTesseractEnvironment()->getSceneGraph(), kin_->getActiveLinkNames(), state_og->transforms);


      auto collision_manager = env.getVKCEnv()->getTesseractEnvironment()->getDiscreteContactManager()->clone();
      collision_manager->setActiveCollisionObjects(adj_map->getActiveLinkNames());

      int attempts = 0;
      bool satisfied = false;
      tesseract_collision::ContactResultMap collisions;
      Eigen::MatrixX2d joint_limits = kin_->getLimits();

      while (attempts < inv_attp_max_)
      {
        ROS_WARN("[%s]%d times try to get a collision free waypoint",
               __func__, attempts);
        // calculate IK and make sure the solution satisfy joint limits
        if (inverseKinematics(inv_kin_solver, solutions, pose, seed) && checkJointLimits(solutions, joint_limits))
        {
          tesseract_environment::EnvState::Ptr state =
              env.getVKCEnv()->getTesseractEnvironment()->getState(kin_->getJointNames(), solutions);

          collision_manager->setCollisionObjectsTransform(state->transforms);

          collisions.clear();
          collision_manager->setContactDistanceThreshold(0.02);  // wanglei@2021-11-12
          collision_manager->contactTest(collisions, tesseract_collision::ContactTestType::FIRST);

          // collisions.size() = zero means no collisioin
          if (0 == collisions.size())
          {
            satisfied = true;
            return true;
          }
          else
          {
            ROS_INFO("[%s]the solution results in at least one collision:\n\t%s <--------> %s",
              __func__, collisions.begin()->second[0].link_names[0].c_str(), 
              collisions.begin()->second[0].link_names[1].c_str());
          }
        }

        genRandState(kin_, seed);
        ++attempts;
      }

      ROS_ERROR("[%s]failed to find a collision free state after %d attempts.", __func__, attempts);
      return false;
    }
    else
    {
      ROS_ERROR("[%s]Unable to find inverse kinematics solver for  %s.", __func__, manipulator.c_str());
      return false;
    }

  }

  void ProbTranslator::genRandState(tesseract_kinematics::ForwardKinematics::Ptr kin, Eigen::VectorXd &seed)
  {
    Eigen::MatrixX2d joint_limits = kin->getLimits();
    std::vector<std::string> joint_names;
    std::unordered_map<std::string, int> joint_name_idx;

    getJointNameIndexMap(kin, joint_name_idx);
    for (auto &jnt : joint_name_idx)
    {
      double f = (double)rand() / RAND_MAX;
      seed[jnt.second] = joint_limits(jnt.second, 0) + f * (joint_limits(jnt.second, 1) - joint_limits(jnt.second, 0));
    }
  }

  bool ProbTranslator::getStartAndGoalState(VKCEnvBasic &env, std::string manipulator)
  {
    start_waypoint = setupStartWaypoint(env, manipulator);
    goal_waypoint = setupGoalWaypoint(env, manipulator);
    if (goal_waypoint == nullptr)
    {
      return false;
    }
    return true;
  }
  // wanglei@2021-10-28 to check whether solution satisfy joints limits
  bool ProbTranslator::checkJointLimits(const Eigen::VectorXd &solution, const Eigen::MatrixX2d& limits)
  {
    for(size_t i = 0; i < solution.size(); ++i)
    {
      if(solution[i] < limits(i, 0) || solution[i] > limits(i, 1))
      {
        ROS_INFO("[%s]joint %d exceeds limits, expected: [%f, %f], actual: %f",
                 __func__, i + 1, limits(i, 0), limits(i, 1), solution[i]);
        return false;
      }
    }
    return true;
  }
  bool ProbTranslator::transPickProb(VKCEnvBasic &env, vkc::PickAction::Ptr act)
  {
    ROS_INFO("Translating Pick Problem");

    setupParams(env, act);
    this->coi_ = std::make_shared<ChainOmplInterface>(env.getVKCEnv()->getTesseractEnvironment(), this->kin);
    this->coi_->setAdjacencyMap();

    insertPlanners(planner_);

    // extract pose information
    BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());
    Eigen::Isometry3d target_pos = attach_location_ptr->world_joint_origin_transform;
    

    this->l_obj = {LinkDesiredPose(env.getEndEffectorLink(), target_pos)};
    this->j_obj.clear();

    return getStartAndGoalState(env, act->getManipulatorID());
  }

  bool ProbTranslator::transPlaceProb(VKCEnvBasic &env, vkc::PlaceAction::Ptr act)
  {

    ROS_INFO("Translating Place Problem");

    setupParams(env, act);
    
    this->coi_ = std::make_shared<ChainOmplInterface>(env.getVKCEnv()->getTesseractEnvironment(), this->kin);
    this->coi_->setAdjacencyMap();

    this->l_obj = act->getLinkObjectives();
    this->j_obj = act->getJointObjectives();

    // get the detach location and feed it to validity checkers -----------------------
    BaseObject::AttachLocation::Ptr detach_location_ptr = env.getAttachLocation(act->getDetachedObject());
    if (detach_location_ptr->fixed_base)
    {
      // std::string fix_link_name = detach_location_ptr->base_link_;
      // std::cout << fix_link_name.c_str() << std::endl;
      // Eigen::Isometry3d fix_link_trans = env.getVKCEnv()
      //                                        ->getTesseract()
      //                                        ->getEnvironment()
      //                                        ->getLinkTransform(detach_location_ptr->base_link_);
      // std::cout << fix_link_trans.matrix() << std::endl;

      // // this->coi_->setConstraintsStd(fix_link_name, fix_link_trans, 0.3);

      // this->coi_->setMotionValidator(std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(
      //     this->coi_->spaceInformation(),
      //     env.getVKCEnv()->getTesseractEnvironment(),
      //     this->kin, fix_link_name, fix_link_trans, 0.3));

      ROS_ERROR("Fixed based method is not valid with OMPL!");
    }
    else
    {
      this->coi_->setMotionValidator(std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(
          this->coi_->spaceInformation(),
          env.getVKCEnv()->getTesseractEnvironment(),
          kin));
    }

    insertPlanners(planner_);

    return getStartAndGoalState(env, act->getManipulatorID());
  }

  bool ProbTranslator::transGotoProb(VKCEnvBasic &env, vkc::GotoAction::Ptr act)
  {
    ROS_INFO("translating go to problem");

    // translate the problem info to ompl
    setupParams(env, act);

    this->coi_ = std::make_shared<ChainOmplInterface>(env.getVKCEnv()->getTesseractEnvironment(), this->kin);

    ompl::base::MotionValidatorPtr mv = std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(
        this->coi_->spaceInformation(),
        env.getVKCEnv()->getTesseractEnvironment(),
        kin);
    this->coi_->setMotionValidator(mv);

    insertPlanners(planner_);

    if (act->getLinkObjectives().size() && act->getJointObjectives().size())
    {
      ROS_ERROR("Both link objectives and jointobjectives found, using link objectives only ");
    }
    if (act->getLinkObjectives().size())
    {
      this->l_obj = act->getLinkObjectives();
    }
    else
    {
      this->j_obj = act->getJointObjectives();
    }

    return getStartAndGoalState(env, act->getManipulatorID());
  }

  bool ProbTranslator::transUseProb(VKCEnvBasic &env, vkc::UseAction::Ptr act)
  {
    UNUSED(env);
    UNUSED(act);
    ROS_ERROR("USE problems are not implemented");
    return false;
  }

  // void ProbTranslator::setBaseBias(double reach_length, double clearance, double span, double bias)
  // {
  //   this->base_bias.reach_length = reach_length;
  //   this->base_bias.clearance = clearance;
  //   this->base_bias.span = span;
  //   this->base_bias.bias = bias;
  // }
  // void ProbTranslator::setBaseBias(BaseBias bb)
  // {
  //   this->base_bias = bb;
  //   return;
  // }

  // bool ProbTranslator::generateIK(VKCEnvBasic &env, Husky_IK::Option ik_option)
  // {

  //   std::vector<double> ewp_std;

  //   switch (ik_option)
  //   {

  //   case Husky_IK::Option::Full:
  //   {
  //     ROS_INFO("Full IK");
  //     ewp_std = IKhelper(env, this->l_obj, this->j_obj, map, n_steps, joint_limits, init_type);
  //     break;
  //   }

  //   case Husky_IK::Option::BaseFirst:
  //   {
  //     ROS_INFO("Base First IK");

  //     ewp_std = BaseFirstIK(env, this->l_obj, this->j_obj, this->base_bias);
  //     break;
  //   }

  //   default:
  //     ewp_std = IKhelper(env, this->l_obj, this->j_obj, map, n_steps, joint_limits, init_type);
  //     break;
  //   }

  //   this->goal_waypoint = ewp_std;

  //   if (goal_waypoint.size() == 0)
  //     return false;
  //   return true;
  // }

  // void ProbTranslator::randBase(Eigen::VectorXd &goal_pose, Eigen::VectorXd &res, BaseBias &bias)
  // {

  //   double len = (double)rand() / RAND_MAX * (bias.reach_length - bias.clearance) + bias.clearance;
  //   double ang = (double)rand() / RAND_MAX * bias.span - bias.span * 0.5 + bias.bias;
  //   // double  = (double)rand() / RAND_MAX * reach_length;
  //   res(0) = cos(ang) * len + goal_pose(0);
  //   res(1) = sin(ang) * len + goal_pose(1);
  //   // res(2) = (double)rand() / RAND_MAX * (2 * M_PI) - M_PI;
  //   res(2) = 0;

  //   // bool init_base_position = false;
  //   // while (!init_base_position) { init_base_position = true; double r = (rand() % 1000) / 1000.0 * 0.8;
  //   // double a = (rand() % 1000) / 1000.0 * 3.14;
  //   // base_values[0] = 3.79 - r*sin(a);
  //   // base_values[1] = -0.4 + r*cos(a);
  //   // env.getVKCEnv()->getTesseractEnvironment()->setState(base_joints, base_values);
  //   // env.getVKCEnv()->getTesseractEnvironment()->getDiscreteContactManager()->contactTest( contact_results, tesseract_collision::ContactTestType::ALL); for (auto &collision : contact_results) { if (collision.first.first == "base_link" || collision.first.second == "base_link") { init_base_position = false; break; } } contact_results.clear(); }
  // }

  // Eigen::VectorXd ProbTranslator::genBaseGoal(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives, BaseBias &bias)
  // {
  //   // std::vector<double> res;
  //   Eigen::VectorXd res;

  //   const int max_tries = 100;
  //   EnvState::ConstPtr current_state = env.getVKCEnv()->getTesseractEnvironment()->getCurrentState();

  //   tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
  //       env.getVKCEnv()->getTesseractEnvironment()->getDiscreteContactManager()->clone();

  //   // enable all collision
  //   // for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects())
  //   // {
  //   //   disc_cont_mgr_->enableCollisionObject(active_link);
  //   // }

  //   // disable arm collision
  //   for (auto &passive_link : kin_arm->getActiveLinkNames())
  //   {
  //     disc_cont_mgr_->disableCollisionObject(passive_link);
  //   }

  //   //-----------------------------------------------
  //   // for (auto &passive_link : kin_base->getActiveLinkNames())
  //   // {
  //   //   std::cout << passive_link.c_str() << std::endl;
  //   // }
  //   // --------------------------------------------------

  //   int tries = 0;
  //   Eigen::VectorXd base_pose = Eigen::VectorXd::Zero(3);
  //   Eigen::VectorXd goal_pose = Eigen::VectorXd::Zero(3);

  //   // find the the end effector goal
  //   for (LinkDesiredPose &ldp : link_objectives)
  //   {
  //     if (ldp.link_name.compare(env.getEndEffectorLink()) == 0)
  //     {
  //       goal_pose = ldp.tf.translation();
  //       // std::cout << "Goal pose is " << std::endl;
  //       // std::cout << ldp.tf.matrix() << std::endl;
  //     }
  //   }

  //   // get the base goal
  //   while (tries < max_tries)
  //   {

  //     randBase(goal_pose, base_pose, bias);

  //     //-----------------------------------------------

  //     // std::cout << base_pose << std::endl;

  //     // --------------------------------------------------

  //     tesseract_environment::EnvState::Ptr env_state =
  //         env.getVKCEnv()->getTesseractEnvironment()->getState(this->kin_base->getJointNames(), base_pose);

  //     contact_results.clear();
  //     disc_cont_mgr_->setCollisionObjectsTransform(env_state->transforms);

  //     disc_cont_mgr_->contactTest(contact_results, tesseract_collision::ContactTestType::FIRST);

  //     if (contact_results.empty())
  //     {
  //       // std::cout << base_pose << std::endl;

  //       // for (int i = 0; i < base_pose.size(); i++)
  //       //   res.push_back(base_pose(i));
  //       // break;

  //       return base_pose;
  //     }
  //     else
  //     {
  //       auto it = contact_results.begin();
  //       while (it != contact_results.end())
  //       {
  //         std::cout << it->first.first.c_str() << " " << it->first.second.c_str() << std::endl;

  //         // it = contact_results.erase(it);

  //         ++it;
  //       }
  //     }

  //     tries++;
  //   }

  //   // std::cout << base_pose << std::endl;
  //   return res;
  // }

  // Eigen::VectorXd ProbTranslator::initArmGoal(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
  //                                             std::vector<JointDesiredPose> &joint_objectives, Eigen::VectorXd base_pose)
  // {
  //   // srand(time(NULL));
  //   std::string arm_name = "arm";

  //   int max_iter = 1000;

  //   std::vector<std::string> total_joint_names;
  //   std::vector<std::string> base_joint_names = kin_base->getJointNames();

  //   env.getVKCEnv()->getTesseractEnvironment()->setState(base_joint_names, base_pose);

  //   tesseract::InverseKinematicsManager::Ptr inv_kin_mgr = env.getVKCEnv()->getTesseract()->getInvKinematicsManager();
  //   tesseract_collision::DiscreteContactManager::Ptr disc_cont_mgr_ =
  //       env.getVKCEnv()->getTesseractEnvironment()->getDiscreteContactManager()->clone();

  //   // for (auto &active_link : disc_cont_mgr_->getActiveCollisionObjects())
  //   // {
  //   //   disc_cont_mgr_->enableCollisionObject(active_link);
  //   // }

  //   // disable base collision
  //   for (auto &passive_link : this->kin_base->getActiveLinkNames())
  //   {
  //     disc_cont_mgr_->enableCollisionObject(passive_link);
  //   }

  //   tesseract_collision::ContactResultMap contact_results;

  //   Eigen::VectorXd sol;
  //   Eigen::VectorXd seed;
  //   sol.resize(inv_kin_mgr->getInvKinematicSolver(arm_name)->numJoints());
  //   seed.resize(inv_kin_mgr->getInvKinematicSolver(arm_name)->numJoints());
  //   sol.setZero();
  //   seed.setZero();

  //   std::unordered_map<std::string, int> joint_name_idx;
  //   int idx = 0;

  //   std::vector<std::string> joint_names;
  //   joint_names = inv_kin_mgr->getInvKinematicSolver(arm_name)->getJointNames();
  //   total_joint_names.reserve(base_joint_names.size() + joint_names.size()); // preallocate memory
  //   total_joint_names.insert(total_joint_names.end(), base_joint_names.begin(), base_joint_names.end());
  //   total_joint_names.insert(total_joint_names.end(), joint_names.begin(), joint_names.end());

  //   for (auto &jnt : joint_names)
  //   {
  //     // std::cout << jnt << std::endl;
  //     joint_name_idx[jnt] = idx;
  //     ++idx;
  //   }

  //   int inv_iter = 0;
  //   bool inv_suc = false;
  //   int satisfy_collision = 1;
  //   bool satisfy_limit = false;

  //   if (link_objectives.size() > 0)
  //   {
  //     for (auto &link_obj : link_objectives)
  //     {
  //       // std::cout << inv_kin_mgr->getInvKinematicSolver(DEFAULT_VKC_GROUP_ID)->getTipLinkName() << std::endl;
  //       // std::cout << link_obj.link_name << std::endl;

  //       if (link_obj.link_name == inv_kin_mgr->getInvKinematicSolver(arm_name)->getTipLinkName())
  //       {

  //         // std::cout << "BREAK "<< std::endl;
  //         // std::cout <<inv_kin_mgr->getInvKinematicSolver(arm_name)->getTipLinkName() << std::endl;

  //         initFinalJointSeed(joint_name_idx, joint_objectives, seed);

  //         Eigen::MatrixX2d joint_limits = inv_kin_mgr->getInvKinematicSolver(arm_name)->getLimits();
  //         // std::cout << joint_limits << std::endl;

  //         while (satisfy_collision > 0 || inv_iter == 0 || !inv_suc || !satisfy_limit)
  //         {
  //           ++inv_iter;

  //           for (auto &jnt : joint_name_idx)
  //           {
  //             if (jnt.second >= 0) // jnt.second != 2 &&
  //             {
  //               if (jnt.second == 1 || jnt.second == 3 || jnt.second == 4)
  //               {
  //                 seed[jnt.second] = rand() % (int(joint_limits(jnt.second, 1))) - int(0.5 * joint_limits(jnt.second, 1));
  //               }
  //               else
  //               {
  //                 seed[jnt.second] = rand() % (2 * int(joint_limits(jnt.second, 1))) - int(joint_limits(jnt.second, 1));
  //               }
  //             }
  //           }

  //           // Eigen::Isometry3d stuff;
  //           // stuff.translation() = Eigen::Vector3d(3.79, -0.4, 0.95);
  //           // stuff.linear() = Eigen::Quaterniond(0,1,0,0).matrix();

  //           inv_suc = inv_kin_mgr->getInvKinematicSolver(arm_name)->calcInvKin(sol, link_obj.tf, seed);

  //           // std::cout << "IK solver name is"<< std::endl;
  //           // std::cout << inv_kin_mgr->getInvKinematicSolver(arm_name)->getSolverName().c_str() << std::endl;
  //           // tesseract_kinematics::KDLInvKinChainLMA::Ptr con_ptr = std::dynamic_pointer_cast<tesseract_kinematics::KDLInvKinChainLMA>(inv_kin_mgr->getInvKinematicSolver(arm_name));
  //           // if (con_ptr->checkInitialized()){
  //           //   ROS_WARN("CON INITED");
  //           // }
  //           // else {
  //           //   ROS_WARN("CON NOT INITED");
  //           // }

  //           // if (!inv_suc) std::cout <<  "IK failed" << std::endl;
  //           // if (inv_suc) std::cout <<  "IK got it" << std::endl;
  //           // std::cout << seed << std::endl;

  //           // if (inv_suc){
  //           //   ROS_WARN("IK worked");
  //           // }

  //           // Eigen::VectorXd jojo(base_pose.size() + sol.size());
  //           // jojo << base_pose, sol;

  //           // std::cout << sol << std::endl;

  //           tesseract_environment::EnvState::Ptr env_state =
  //               env.getVKCEnv()->getTesseractEnvironment()->getState(joint_names, sol);

  //           if (inv_iter < 4)
  //           {
  //             for (std::string nm : kin->getJointNames())
  //             {
  //               std::cout << nm.c_str() << std::endl;
  //               std::cout << env_state->joints.at(nm) << std::endl;
  //             }
  //           }

  //           contact_results.clear();
  //           disc_cont_mgr_->setCollisionObjectsTransform(env_state->transforms);

  //           disc_cont_mgr_->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);

  //           satisfy_collision = contact_results.size();

  //           // if (!satisfy_collision) std::cout <<  "has collision" << std::endl;

  //           satisfy_limit = checkJointLimit(sol, inv_kin_mgr->getInvKinematicSolver(arm_name)->getLimits(),
  //                                           inv_kin_mgr->getInvKinematicSolver(arm_name)->numJoints());

  //           // if (!satisfy_limit) std::cout <<  "exceed limit" << std::endl;

  //           if (inv_iter > max_iter - 1)
  //           {
  //             // ROS_WARN("Exceed max inv kin iter!");

  //             // std::cout << sol << std::endl;

  //             break;
  //           }
  //         }
  //       }
  //       else
  //       {
  //         ROS_WARN("Unsupported trajectory initialization for given link desired pose:\t%s", link_obj.link_name.c_str());
  //       }
  //     }
  //   }
  //   // std::vector<double> vec(sol.data(), sol.data() + sol.size());
  //   return sol;
  // }

  // Eigen::VectorXd ProbTranslator::optArmGoal(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
  //                                            std::vector<JointDesiredPose> &joint_objectives, Eigen::VectorXd base_pose)
  // {
  //   Eigen::VectorXd res;

  //   return res;
  // }

  // TrajOptProb::Ptr ProbTranslator::moveArm(VKCEnvBasic &env, int n_steps, Eigen::Vector3d p_goal, Eigen::Quaterniond q_goal, Eigen::VectorXd base_pose)
  // {
  //   ProblemConstructionInfo a_pci(env.getVKCEnv()->getTesseract());

  //   // Populate Basic Info
  //   a_pci.basic_info.n_steps = n_steps;
  //   a_pci.basic_info.manip = "arm";
  //   a_pci.basic_info.start_fixed = true;
  //   a_pci.basic_info.use_time = false;

  //   // if (!validateGroupID(env.getVKCEnv()->getTesseract(), a_pci.basic_info.manip))
  //   //   exit(1);

  //   // Create Kinematic Object
  //   a_pci.kin = this->kin_arm;

  //   std::vector<std::string> base_joint_names = kin_base->getJointNames();

  //   env.getVKCEnv()->getTesseractEnvironment()->setState(base_joint_names, base_pose);

  //   // Populate Init Info
  //   EnvState::ConstPtr current_state = a_pci.env->getCurrentState();
  //   Eigen::VectorXd start_waypoint;
  //   start_waypoint.resize(a_pci.kin->numJoints());
  //   int joint_num = 0;
  //   for (const auto &j : a_pci.kin->getJointNames())
  //   {
  //     start_waypoint[joint_num] = current_state->joints.at(j);
  //     ++joint_num;
  //     // ROS_INFO("Setting joint initial condition: %s", j.c_str());
  //   }

  //   a_pci.init_info.type = InitInfo::STATIONARY;
  //   a_pci.init_info.data = start_waypoint.transpose().replicate(a_pci.basic_info.n_steps, 1);

  //   // Populate Cost Info
  //   std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  //   jv->coeffs = std::vector<double>(joint_num, 5.0);
  //   jv->targets = std::vector<double>(joint_num, 0.0);
  //   jv->first_step = 0;
  //   jv->last_step = a_pci.basic_info.n_steps - 1;
  //   jv->name = "joint_vel";
  //   jv->term_type = TT_COST;
  //   a_pci.cost_infos.push_back(jv);

  //   std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  //   collision->name = "collision";
  //   collision->term_type = TT_CNT;
  //   collision->continuous = true;
  //   collision->first_step = 0;
  //   collision->last_step = a_pci.basic_info.n_steps - 2;
  //   collision->gap = 1;
  //   collision->info = createSafetyMarginDataVector(a_pci.basic_info.n_steps, 0.0001, 40);
  //   a_pci.cnt_infos.push_back(collision);

  //   std::shared_ptr<CartPoseTermInfo> ee_goal = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  //   ee_goal->term_type = TT_CNT;
  //   ee_goal->name = "ee_goal";
  //   ee_goal->link = "ur_arm_ee_link";
  //   ee_goal->timestep = a_pci.basic_info.n_steps - 1;
  //   ee_goal->xyz = p_goal;
  //   ee_goal->wxyz = Eigen::Vector4d(q_goal.w(), q_goal.x(), q_goal.y(), q_goal.z());
  //   ee_goal->pos_coeffs = Eigen::Vector3d(10, 10, 10);
  //   ee_goal->rot_coeffs = Eigen::Vector3d(10, 10, 10);
  //   a_pci.cnt_infos.push_back(ee_goal);

  //   return ConstructProblem(a_pci);
  // }

  // std::vector<double> ProbTranslator::BaseFirstIK(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
  //                                                 std::vector<JointDesiredPose> &joint_objectives, BaseBias &bias)
  // {

  //   // std::vector<double> res;

  //   // get goal position
  //   Eigen::VectorXd base_pose = genBaseGoal(env, link_objectives, bias);

  //   // set base to goal position
  //   // env.getVKCEnv()->getTesseractEnvironment()->setState(this->kin_base->getJointNames(), base_pose);
  //   // EnvState::ConstPtr state = env.getVKCEnv()->getTesseractEnvironment()->getCurrentState();

  //   // 2.5
  //   // Eigen::VectorXd base_pose = Eigen::VectorXd::Zero(3);
  //   // base_pose << 1.57189,   -0.93617,  -0.537171 ;
  //   std::cout << "Base result is" << std::endl;
  //   std::cout << base_pose << std::endl;

  //   // for (std::string nm : kin->getJointNames()){
  //   //   std::cout << nm.c_str() << std::endl;
  //   //   std::cout << state->joints.at(nm) << std::endl;
  //   // }

  //   // base_pose(0) = 1.8;
  //   // base_pose(1) = -1.5;

  //   // take traj opt to calculate goal state
  //   Eigen::Vector3d p_goal;
  //   Eigen::Quaterniond q_goal;

  //   for (LinkDesiredPose &ldp : link_objectives)
  //   {
  //     if (ldp.link_name.compare(env.getEndEffectorLink()) == 0)
  //     {
  //       p_goal = ldp.tf.translation();
  //       q_goal = ldp.tf.linear();
  //     }
  //   }

  //   std::cout << "Calculating Arm IK" << std::endl;
  //   TrajOptProb::Ptr toptr = moveArm(env, 10, p_goal, q_goal, base_pose);

  //   PlannerResponse response;

  //   solveOptProb(toptr, response, 500);

  //   if (response.status_code < 0)
  //   {
  //     std::cout << "IK Failed" << std::endl;
  //     std::vector<double> mt_res;
  //     return mt_res;
  //   }
  //   std::cout << "IK result is" << std::endl;
  //   std::cout << response.trajectory << std::endl;

  //   Eigen::VectorXd arm_pose = response.trajectory.row(response.trajectory.rows() - 1);

  //   // Eigen::VectorXd arm_pose = initArmGoal(env, link_objectives, joint_objectives, base_pose);

  //   // for (double p : base_pose)
  //   // {
  //   //   std::cout << p << std::endl;
  //   // }

  //   // for (double p : arm_pose)
  //   // {
  //   //   std::cout << p << std::endl;
  //   // }

  //   // std::cout << "finished 3" << std::endl;

  //   env.getVKCEnv()->getTesseractEnvironment()->setState(this->kin->getJointNames(), start_waypoint);

  //   // std::cout << "finished 4" << std::endl;

  //   // res.reserve(base_pose.size() + arm_pose.size()); // preallocate memory
  //   // res.insert(res.end(), base_pose.begin(), base_pose.end());
  //   // res.insert(res.end(), arm_pose.begin(), arm_pose.end());

  //   // for (double p : res)
  //   // {
  //   //   std::cout << p << std::endl;
  //   // }

  //   Eigen::VectorXd pose(base_pose.size() + arm_pose.size());
  //   pose << base_pose, arm_pose;

  //   std::cout << pose << std::endl;
  //   std::vector<double> res(pose.data(), pose.data() + pose.size());

  //   return res;
  // }

  // std::vector<double> ProbTranslator::IKhelper(VKCEnvBasic &env, std::vector<LinkDesiredPose> &link_objectives,
  //                                              std::vector<JointDesiredPose> &joint_objectives, MapInfo map,
  //                                              int n_steps, Eigen::MatrixX2d &joint_limits, InitInfo init_type)
  // {
  //   int inv_attp = 0;
  //   ROS_INFO("Determining Full IK");
  //   std::vector<double> ewp_std = HuskeyIK(env, link_objectives, joint_objectives, map, n_steps, joint_limits);
  //   // while (!(this->coi_->isStateValidStd(ewp_std)))
  //   // {
  //   //   ROS_INFO("Retrying IK");
  //   //   ewp_std = HuskeyIK(env, link_objectives, joint_objectives, map, n_steps, joint_limits);

  //   //   inv_attp++;
  //   //   if (inv_attp > this->inv_attp_max)
  //   //   {
  //   //     ROS_WARN("IK failed for OMPL!");
  //   //     break;
  //   //   }
  //   // }

  //   return ewp_std;
  // }

  // void ProbTranslator::overwriteGoal(std::vector<double> goal)
  // {
  //   ROS_WARN("YOU ARE OVERWRITING THE IK RESULT");
  //   this->fixed_goal = goal;
  // }

  // void ProbTranslator::overwriteLimit(bool option)
  // {
  //   // if (this->fixed_goal.size()){
  //   //   ROS_WARN("using limited limits");
  //   // }
  //   // else{
  //   //   ROS_WARN("no predetermined goals");
  //   // }

  //   this->use_small_limit = option;
  // }

  // void ProbTranslator::setIKOption(Husky_IK::Option option)
  // {
  //   ROS_INFO("Changing IK Option to : %d", option);
  //   ik_option = option;
  // }

  // Eigen::MatrixX2d ProbTranslator::processLimit(Eigen::MatrixX2d init_limit)
  // {

  //   Eigen::MatrixX2d res = init_limit;
  //   int dof = this->kin->getJointNames().size();
  //   for (unsigned i = 0; i < dof; ++i)
  //   {
  //     if (this->start_waypoint[i] > this->fixed_goal[i])
  //     {
  //       res(i, 0) = this->fixed_goal[i] - 0.5;
  //       res(i, 1) = this->start_waypoint[i] + 0.5;
  //     }
  //     else
  //     {
  //       res(i, 1) = this->fixed_goal[i] + 0.5;
  //       res(i, 0) = this->start_waypoint[i] - 0.5;
  //     }

  //     // if (i == 10)
  //     // {
  //     //   res(i, 1) = 0;
  //     //   res(i, 0) = -1.5708;
  //     // }
  //   }

  //   //          -10       10
  //   //      -10       10
  //   //    -3.14     3.14
  //   // -3.14159  3.14159
  //   // -3.14159  3.14159
  //   // -3.14159  3.14159
  //   // -3.14159  3.14159
  //   // -3.14159  3.14159
  //   // -6.28319  6.28319
  //   // -3.14159  3.14159
  //   //  -1.5708       -0

  //   return res;
  // }

} // namespace vkc
