#include <fmt/ranges.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/mixed_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_ik_plan_profile.h>
#include <tesseract_motion_planners/interface_utils.h>
#include <tesseract_motion_planners/ompl/profile/ompl_constrained_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <vkc/planner/prob_generator.h>
#include <vkc/planner/traj_init.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_planning;

namespace vkc {
ProbGenerator::ProbGenerator() {}

PlannerRequest ProbGenerator::genRequest(VKCEnvBasic &env,
                                         ActionBase::Ptr action, int n_steps,
                                         int n_iter) {
  MixedWaypointPoly wp;
  wp = genMixedWaypoint(env, action);
  return genRequest(env, action, wp, n_steps, n_iter);
}

PlannerRequest ProbGenerator::genRequest(VKCEnvBasic &env, ActionBase::Ptr act,
                                         MixedWaypointPoly wp, int n_steps,
                                         int n_iter) {
  CONSOLE_BRIDGE_logDebug("generating planning request...");
  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();
  tesseract_common::ManipulatorInfo manip;
  manip.tcp_frame = env.getEndEffectorLink();
  manip.working_frame = "world";
  manip.manipulator = act->getManipulatorID();
  auto kinematic_group = env_->getKinematicGroup(
      act->getManipulatorID());  // same as joint group for initial
                                 // step(verified)

  // set profiles
  double collision_margin = 0.001;
  double collision_coeff = 10;

  auto pos_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);
  auto rot_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);

  auto profiles = genPlannerProfiles_(env, manip, collision_margin,
                                      collision_coeff, pos_coeff, rot_coeff);
  addSolverProfile(profiles, n_iter);

  CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED,
                               manip);

  // set initial pose
  setStartInstruction(
      program, kinematic_group->getJointNames(),
      env_->getCurrentJointValues(kinematic_group->getJointNames()));
  auto seed_program = program;

  MoveInstruction plan_instruction(wp, MoveInstructionType::FREESPACE,
                                   "DEFAULT");
  plan_instruction.setDescription(
      fmt::format("waypoint for {}", act->getActionName()));
  program.appendMoveInstruction(plan_instruction);

  auto seed_instruction = plan_instruction;
  if (act->joint_candidate.size()) {
    CONSOLE_BRIDGE_logInform(
        "joint candidate found, resetting seed waypoint to joint waypoint...");
    // std::cout << act->joint_candidate.size() << std::endl;
    assert(kinematic_group->getJointNames().size() ==
           act->joint_candidate.size());
    seed_instruction.assignJointWaypoint(JointWaypointPoly{
        JointWaypoint(kinematic_group->getJointNames(), act->joint_candidate)});
  }
  seed_program.appendMoveInstruction(seed_instruction);

  // generate seed
  auto cur_state = env.getVKCEnv()->getTesseract()->getState();

  // CompositeInstruction seed =
  //     generateSeed(program, cur_state, env.getVKCEnv()->getTesseract());
  CompositeInstruction seed =
      act->seed.empty()
          ? generateMixedSeed(seed_program, cur_state,
                              env.getVKCEnv()->getTesseract(), n_steps,
                              act->getBaseJoint(), act->getIKCostCoeff())
          : act->seed;

  ROS_INFO("number of move instructions in pick seed: %ld",
           seed.getMoveInstructionCount());

  // seed.print(fmt::format("{} seed: ", act->getActionName()));

  // compose request
  PlannerRequest request;
  request.name = process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = program;
  request.profiles = profiles;
  request.seed = seed;
  request.env_state = cur_state;
  request.env = env.getVKCEnv()->getTesseract();

  ROS_INFO("%s request generated.", act->getActionName().c_str());

  return request;
}

PlannerRequest ProbGenerator::getOmplRequest(VKCEnvBasic &env,
                                             ActionBase::Ptr action,
                                             int n_steps, int n_iter) {
  auto wp = genMixedWaypoint(env, action);
  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();
  tesseract_common::ManipulatorInfo manip;
  manip.tcp_frame = env.getEndEffectorLink();
  manip.working_frame = "world";
  manip.manipulator = action->getManipulatorID();
  tesseract_kinematics::KinematicGroup::Ptr kin_group =
      std::move(env_->getKinematicGroup(action->getManipulatorID()));

  auto ompl_planner_config = std::make_shared<RRTConnectConfigurator>(); //LazyPRMstar, RRTConnect
  auto profiles = std::make_shared<ProfileDictionary>();
  std::string default_profile = "FREESPACE";
  if (wp.getLinkConstraints().size()) {
    /* generate constrained ompl profile(deprecated)
    // auto ompl_profile = std::make_shared<OMPLConstrainedPlanProfile>();
    // auto constraint =
    //     std::make_shared<LinkConstraint>(kin_group, wp.link_constraints,
    0.2);
    // ompl_profile->constraint = constraint;
    // ompl_profile->planners = {ompl_planner_config, ompl_planner_config};
    // profiles->addProfile<OMPLConstrainedPlanProfile>(
    //     profile_ns::OMPL_DEFAULT_NAMESPACE, "FREESPACE", ompl_profile);
    */
    default_profile = "IK_TRAJ";
    auto ik_profile = std::make_shared<MMMOPlannerIKPlanProfile>();
    ik_profile->min_steps = n_steps;
    ik_profile->local_joint_origin_transform =
        env.getAttachLocation("attach_door_north_handle_link")
            ->local_joint_origin_transform;
    ik_profile->attach_location_link =
        env.getAttachLocation("attach_door_north_handle_link")->link_name_;

    profiles->addProfile<MMMOPlannerIKPlanProfile>(
        profile_ns::MMMO_DEFAULT_NAMESPACE, default_profile, ik_profile);
  } else {
    auto ompl_profile = std::make_shared<OMPLDefaultPlanProfile>();
    ompl_profile->planning_time = 10.;
    ompl_profile->planners = {ompl_planner_config, ompl_planner_config};
    profiles->addProfile<OMPLDefaultPlanProfile>(
        profile_ns::OMPL_DEFAULT_NAMESPACE, default_profile, ompl_profile);
  }

  CONSOLE_BRIDGE_logDebug("generating program");
  CompositeInstruction program(default_profile,
                               CompositeInstructionOrder::ORDERED, manip);
  setStartInstruction(program, kin_group->getJointNames(),
                      env_->getCurrentJointValues(kin_group->getJointNames()));
  MoveInstruction plan_instruction(wp, MoveInstructionType::FREESPACE,
                                   default_profile);
  if (action->joint_candidate.size()) {
    CONSOLE_BRIDGE_logDebug(
        "joint candidate found, resetting seed waypoint to joint waypoint...");
    plan_instruction.assignJointWaypoint(JointWaypointPoly{
        JointWaypoint(kin_group->getJointNames(), action->joint_candidate)});
  }
  plan_instruction.setDescription(
      fmt::format("waypoint for {}", action->getActionName()));
  auto seed_program = program;
  auto seed_instrcution = plan_instruction;
  seed_instrcution.assignJointWaypoint(JointWaypointPoly{
      JointWaypoint(kin_group->getJointNames(),
                    env_->getCurrentJointValues(kin_group->getJointNames()))});
  seed_program.appendMoveInstruction(seed_instrcution);

  program.appendMoveInstruction(plan_instruction);
  auto cur_state = env.getVKCEnv()->getTesseract()->getState();

  CompositeInstruction seed =
      generateSeed(seed_program, cur_state, env.getVKCEnv()->getTesseract(),
                   0.087, 0.15, 0.087, n_steps);

  PlannerRequest request;
  request.name = wp.getLinkConstraints().size()
                     ? "3MO_IK_TRAJ"
                     : process_planner_names::OMPL_PLANNER_NAME;
  request.instructions = program;
  request.profiles = profiles;
  request.env_state = cur_state;
  request.seed = seed;
  request.env = env.getVKCEnv()->getTesseract();

  ROS_INFO("%s ompl request generated.", action->getActionName().c_str());

  return request;
}

MixedWaypointPoly ProbGenerator::genMixedWaypoint(VKCEnvBasic &env,
                                                  ActionBase::Ptr action) {
  switch (action->getActionType()) {
    case ActionType::PickAction: {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      // initFinalPose(env, std::vector<LinkDesiredPose>(),
      // std::vector<JointDesiredPose>(), ActionType::PickAction);
      return genPickMixedWaypoint(env, pick_act);
    }

    case ActionType::GotoAction: {
      GotoAction::Ptr goto_act = std::dynamic_pointer_cast<GotoAction>(action);
      return genGotoMixedWaypoint(env, goto_act);
    }

    case ActionType::PlaceAction: {
      PlaceAction::Ptr place_act =
          std::dynamic_pointer_cast<PlaceAction>(action);
      return genPlaceMixedWaypoint(env, place_act);
    }

      // case ActionType::UseAction:
      // {
      //   UseAction::Ptr use_act =
      //   std::dynamic_pointer_cast<UseAction>(action); return genUseProb(env,
      //   use_act, n_steps);
      // }

    default: {
      ROS_ERROR("Undefined action type.");
    }
  }
  throw std::logic_error("action error");
}

MixedWaypointPoly ProbGenerator::genPickMixedWaypoint(VKCEnvBasic &env,
                                                      PickAction::Ptr act) {
  CONSOLE_BRIDGE_logDebug("generating pick mixed waypoint");
  std::cout << "act manipulator id: " << act->getManipulatorID() << std::endl;
  std::cout << fmt::format("group joint names: {}",
                           env.getVKCEnv()->getTesseract()->getGroupJointNames(
                               act->getManipulatorID()))
            << std::endl;
  auto kin_group = env.getVKCEnv()->getTesseract()->getKinematicGroup(
      act->getManipulatorID());
  // std::cout << fmt::format("{}", kin_group->getJointNames()) << std::endl;
  MixedWaypointPoly waypoint{MixedWaypoint(kin_group->getJointNames())};
  BaseObject::AttachLocation::ConstPtr attach_location_ptr =
      env.getAttachLocation(act->getAttachedObject());
  // std::cout << "test" << std::endl << act->getAttachedObject() << std::endl;
  // CONSOLE_BRIDGE_logDebug("attack location: %s",
  //                         attach_location_ptr->link_name_.c_str());
  Eigen::Isometry3d pick_pose_world_transform =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  // std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
  //           << std::endl;
  // std::cout << pick_pose_world_transform.translation() << std::endl;
  // std::cout << pick_pose_world_transform.linear() << std::endl;
  // std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
  //           << std::endl;

  waypoint.addLinkTarget(env.getEndEffectorLink(), pick_pose_world_transform);
  return waypoint;
}

MixedWaypointPoly ProbGenerator::genPlaceMixedWaypoint(VKCEnvBasic &env,
                                                       PlaceAction::Ptr act) {
  auto kin_group = env.getVKCEnv()->getTesseract()->getKinematicGroup(
      act->getManipulatorID());
  MixedWaypointPoly waypoint{MixedWaypoint(kin_group->getJointNames())};

  BaseObject::AttachLocation::ConstPtr detach_location_ptr =
      env.getAttachLocation(act->getDetachedObject());
  if (detach_location_ptr == nullptr)
    throw std::runtime_error("detach location ptr is null");
  if (detach_location_ptr->fixed_base) {
    auto detach_pose = env.getVKCEnv()->getTesseract()->getLinkTransform(
        detach_location_ptr->base_link_);
    // waypoint.addLinkTarget(manip.tcp_frame, detach_pose);
    // addCartWaypoint(program, detach_pose, "place object(fixed base)");
    act->addLinkObjectives(
        LinkDesiredPose(detach_location_ptr->base_link_, detach_pose));
    waypoint.addLinkConstraint(detach_location_ptr->base_link_, detach_pose);
  }

  for (auto jo : act->getJointObjectives()) {
    // std::cout << jo.joint_angle << std::endl;
    waypoint.addJointTarget(jo.joint_name, jo.joint_angle);
  }

  for (auto lo : act->getLinkObjectives()) {
    waypoint.addLinkTarget(lo.link_name, lo.tf);
    // addCartWaypoint(program, lo.tf, "place object");
  }
  return waypoint;
}

MixedWaypointPoly ProbGenerator::genGotoMixedWaypoint(VKCEnvBasic &env,
                                                      GotoAction::Ptr act) {
  auto kin_group = env.getVKCEnv()->getTesseract()->getKinematicGroup(
      act->getManipulatorID());
  MixedWaypoint waypoint(kin_group->getJointNames());

  for (auto jo : act->getJointObjectives()) {
    waypoint.addJointTarget(jo.joint_name, jo.joint_angle);
  }

  for (auto lo : act->getLinkObjectives()) {
    waypoint.addLinkTarget(lo.link_name, lo.tf);
    std::cout << lo.tf.linear() << std::endl;
    std::cout << lo.tf.translation() << std::endl;
  }
  return waypoint;
}

bool ProbGenerator::validateGroupID(Environment::Ptr tesseract,
                                    const std::string &group_id) {
  bool isfound_group = false;

  for (const auto &name_ : tesseract->getGroupNames()) {
    if (name_ == group_id) {
      isfound_group = true;
      ROS_INFO("Found group %s.", group_id.c_str());
    }
  }

  if (!isfound_group) {
    ROS_ERROR("Cannot find group %s.", group_id.c_str());
  }

  return isfound_group;
}

Eigen::Vector4d ProbGenerator::getQuatFromIso(Eigen::Isometry3d iso) {
  return Eigen::Vector4d(Eigen::Quaterniond(iso.rotation()).w(),
                         Eigen::Quaterniond(iso.rotation()).x(),
                         Eigen::Quaterniond(iso.rotation()).y(),
                         Eigen::Quaterniond(iso.rotation()).z());
}

ProfileDictionary::Ptr ProbGenerator::genPlannerProfiles_(
    VKCEnvBasic &env, tesseract_common::ManipulatorInfo manip,
    double collision_margin, double collision_coeff, Eigen::Vector3d pos_coeff,
    Eigen::Vector3d rot_coeff) {
  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();
  auto trajopt_composite_profile =
      std::make_shared<TrajOptDefaultCompositeProfile>();
  setCompositeProfile(
      trajopt_composite_profile, collision_margin, collision_coeff,
      (long int)env_->getGroupJointNames(manip.manipulator).size());

  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  setCartPlanProfile(trajopt_plan_profile, pos_coeff, rot_coeff);

  // trajopt_plan_profile->fixed_dofs = {3,4,5};

  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptCompositeProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
      trajopt_composite_profile);
  profiles->addProfile<TrajOptPlanProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_plan_profile);

  return profiles;
}

void ProbGenerator::setJointPlanProfile(TrajOptDefaultPlanProfile::Ptr profile,
                                        Eigen::VectorXd joint_coeff) {
  profile->joint_coeff = joint_coeff;
}

void ProbGenerator::setCartPlanProfile(TrajOptDefaultPlanProfile::Ptr profile,
                                       Eigen::Vector3d pos_coeff,
                                       Eigen::Vector3d rot_coeff) {
  profile->cartesian_coeff.resize(6);
  profile->cartesian_coeff.head<3>() = pos_coeff;
  profile->cartesian_coeff.tail<3>() = rot_coeff;
  profile->term_type = trajopt::TermType::TT_CNT;
}

void ProbGenerator::setCompositeProfile(
    TrajOptDefaultCompositeProfile::Ptr profile, double margin, double coeff,
    Eigen::Index num_joints) {
  profile->collision_constraint_config.enabled = true;
  profile->collision_constraint_config.type =
      trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  profile->collision_constraint_config.safety_margin = margin;
  profile->collision_constraint_config.coeff = coeff;
  profile->collision_cost_config.enabled = false;

  profile->smooth_accelerations = true;
  profile->smooth_velocities = true;
  profile->smooth_jerks = true;

  profile->velocity_coeff.setConstant(num_joints, 5);
  // profile->velocity_coeff[0] = 30;
  // profile->velocity_coeff[1] = 30;

  profile->acceleration_coeff.setConstant(num_joints, 5);
}

void ProbGenerator::addSolverProfile(ProfileDictionary::Ptr profiles,
                                     int n_iter) {
  auto trajopt_solver_profile =
      std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = n_iter;
  trajopt_solver_profile->opt_info.cnt_tolerance = 5e-3;
  // trajopt_solver_profile->opt_info.trust_expand_ratio = 1.2;
  trajopt_solver_profile->opt_info.trust_expand_ratio = 1.5;
  // trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.8;
  trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.5;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-2;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-2;
  trajopt_solver_profile->opt_info.inflate_constraints_individually = true;
  trajopt_solver_profile->opt_info.max_time = 30.;

  profiles->addProfile<TrajOptSolverProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);

  return;
}

void ProbGenerator::setStartInstruction(CompositeInstruction &program,
                                        ros::V_string joint_names,
                                        Eigen::VectorXd joint_values) {
  std::stringstream ss;
  ss << joint_values.transpose();
  CONSOLE_BRIDGE_logDebug("start instruction set with state: %s",
                          ss.str().c_str());
  auto start_waypoint =
      StateWaypointPoly{StateWaypoint(joint_names, joint_values)};
  MoveInstruction start_instruction(start_waypoint, MoveInstructionType::START);
  start_instruction.setDescription("start instruction set by prob generator");
  program.setStartInstruction(start_instruction);
}

void ProbGenerator::addCartWaypoint(CompositeInstruction &program,
                                    Eigen::Isometry3d pose,
                                    std::string description) {
  MoveInstruction pick_plan(CartesianWaypointPoly{CartesianWaypoint(pose)},
                            MoveInstructionType::FREESPACE, "DEFAULT");
  pick_plan.setDescription(description);
  program.appendMoveInstruction(pick_plan);
}

void ProbGenerator::addJointTerm(ProblemConstructionInfo &pci, int joint_num) {
  // the following if expression is added to avoid exception while to allocate
  // an empty vector added by: wanglei@bigai.ai, 2021-11-08
  if (0 == joint_num) {
    ROS_INFO("[%s]joint number: %d", __func__, joint_num);
    return;
  }

  // Populate Cost Info
  std::shared_ptr<JointVelTermInfo> jv_cost =
      std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv_cost->coeffs =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 5);
  jv_cost->coeffs[0] = 30;  //(rand() % 100) / 5 + 30;
  jv_cost->coeffs[1] = jv_cost->coeffs[0];
  jv_cost->targets =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jv_cost->first_step = 0;
  jv_cost->last_step = pci.basic_info.n_steps - 1;
  jv_cost->name = "joint_vel_cost";
  jv_cost->term_type = TT_COST;
  pci.cost_infos.push_back(jv_cost);

  std::shared_ptr<JointVelTermInfo> jv_limit =
      std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv_limit->coeffs =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 100.0);
  jv_limit->targets =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jv_limit->upper_tols =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.5);
  jv_limit->lower_tols =
      std::vector<double>(static_cast<unsigned long int>(joint_num), -0.5);
  jv_limit->first_step = 0;
  jv_limit->last_step = pci.basic_info.n_steps - 1;
  jv_limit->name = "joint_vel_limit";
  jv_limit->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv_limit);

  std::shared_ptr<JointAccTermInfo> ja_cost =
      std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  ja_cost->coeffs =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 5.0);
  ja_cost->targets =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  ja_cost->first_step = 0;
  ja_cost->last_step = pci.basic_info.n_steps - 1;
  ja_cost->name = "joint_acc_cost";
  ja_cost->term_type = TT_COST;
  pci.cost_infos.push_back(ja_cost);

  std::shared_ptr<JointAccTermInfo> ja_limit =
      std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  ja_limit->coeffs =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 100.0);
  ja_limit->targets =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  ja_limit->upper_tols =
      std::vector<double>(static_cast<unsigned long int>(joint_num), 0.3);
  ja_limit->lower_tols =
      std::vector<double>(static_cast<unsigned long int>(joint_num), -0.3);
  ja_limit->first_step = 0;
  ja_limit->last_step = pci.basic_info.n_steps - 1;
  ja_limit->name = "joint_acc_limit";
  ja_limit->term_type = TT_CNT;
  pci.cnt_infos.push_back(ja_limit);
}

void ProbGenerator::addCollisionTerm(ProblemConstructionInfo &pci,
                                     double margin, double coeff) {
  // TODO: use trajopt_utils createCollisionTermInfo
  std::shared_ptr<CollisionTermInfo> collision =
      std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->name = "collision";
  collision->term_type = TT_CNT;
  collision->evaluator_type =
      trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 2;
  // collision->gap = 1;
  collision->info =
      util::createSafetyMarginDataVector(pci.basic_info.n_steps, margin, coeff);
  pci.cnt_infos.push_back(collision);
}

void ProbGenerator::addTargetTerm(ProblemConstructionInfo &pci,
                                  LinkDesiredPose &link_pose,
                                  Eigen::Vector3d pos_coeff,
                                  Eigen::Vector3d rot_coeff) {
  std::shared_ptr<CartPoseTermInfo> Link_Goal =
      std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  Link_Goal->term_type = TT_CNT;
  Link_Goal->name = link_pose.link_name + "_Goal";
  Link_Goal->source_frame = link_pose.link_name;
  Link_Goal->timestep = pci.basic_info.n_steps - 1;
  Link_Goal->target_frame = "world";
  Link_Goal->target_frame_offset = link_pose.tf;
  // Link_Goal->xyz = link_pose.tf.translation();
  // Link_Goal->wxyz = getQuatFromIso(link_pose.tf);
  Link_Goal->pos_coeffs = pos_coeff;
  Link_Goal->rot_coeffs = rot_coeff;
  pci.cnt_infos.push_back(Link_Goal);
}

void ProbGenerator::addTargetTerm(ProblemConstructionInfo &pci,
                                  JointDesiredPose &joint_pose, int joint_num,
                                  double coeff) {
  // // the following if expression is added to avoid exception while to
  // allocate an empty vector
  // // added by: wanglei@bigai.ai, 2021-11-08
  // if (0 == joint_num)
  // {
  //   ROS_INFO("[%s]joint number: %d", __func__, joint_num);
  //   return;
  // }

  // ROS_INFO("[%s]joint number: %d, joint: %s, target value: %f",
  //          __func__, joint_num, joint_pose.joint_name.c_str(),
  //          joint_pose.joint_angle);

  // std::shared_ptr<JointPosTermInfo> jp =
  // std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo); jp->coeffs =
  // std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  // jp->coeffs[planned_joints.at(joint_pose.joint_name)] = coeff;
  // jp->targets = std::vector<double>(static_cast<unsigned long
  // int>(joint_num), 0.0);
  // jp->targets[planned_joints.at(joint_pose.joint_name)] =
  // joint_pose.joint_angle; jp->upper_tols =
  // std::vector<double>(static_cast<unsigned long int>(joint_num), 0.001);
  // jp->lower_tols = std::vector<double>(static_cast<unsigned long
  // int>(joint_num), 0.001); jp->first_step = pci.basic_info.n_steps - 1;
  // jp->last_step = pci.basic_info.n_steps - 1;
  // jp->name = joint_pose.joint_name + "_Goal";
  // jp->term_type = TT_CNT;
  // pci.cnt_infos.push_back(jp);

  // ROS_INFO("[%s]add target finished", __func__);
}

// not used here
void ProbGenerator::addTargetCost(ProblemConstructionInfo &pci,
                                  LinkDesiredPose &link_pose,
                                  Eigen::Vector3d pos_coeff,
                                  Eigen::Vector3d rot_coeff) {
  // std::shared_ptr<CartPoseTermInfo> Link_Goal =
  // std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  // Link_Goal->term_type = TT_COST;
  // Link_Goal->name = link_pose.link_name + "_Goal";
  // Link_Goal->source_frame = link_pose.link_name;
  // Link_Goal->timestep = pci.basic_info.n_steps - 1;
  // Link_Goal->target_frame = "world";
  // Link_Goal->target_frame_offset = link_pose.tf;
  // // Link_Goal->xyz = link_pose.tf.translation();
  // // Link_Goal->wxyz = getQuatFromIso(link_pose.tf);
  // Link_Goal->pos_coeffs = pos_coeff;
  // Link_Goal->rot_coeffs = rot_coeff;
  // pci.cost_infos.push_back(Link_Goal);
}
PlannerRequest ProbGenerator::genPickProb(VKCEnvBasic &env, PickAction::Ptr act,
                                          int n_steps, int n_iter) {
  // // to make sure the attach link exists
  // if (nullptr == env.getAttachLocation(act->getAttachedObject())) {
  //   ROS_ERROR(
  //       "[%s]attach location named %s does not exist, please specified it "
  //       "first.",
  //       __func__, act->getAttachedObject().c_str());
  //   assert(false);
  // }

  // ROS_DEBUG("generating pick problem");

  // Environment::Ptr env_ = env.getVKCEnv()->getTesseract();
  // ManipulatorInfo manip;
  // manip.tcp_frame = env.getEndEffectorLink();
  // manip.working_frame = "world";
  // manip.manipulator = act->getManipulatorID();
  // auto kinematic_group = env_->getKinematicGroup(
  //     act->getManipulatorID());  // same as joint group for initial
  //                                // step(verified)

  // // set profiles
  // double collision_margin = 0.0001;
  // double collision_coeff = 10;

  // auto pos_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);
  // auto rot_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);

  // auto profiles = genPlannerProfiles_(env, manip, collision_margin,
  //                                     collision_coeff, pos_coeff, rot_coeff);
  // addSolverProfile(profiles, n_iter);

  // CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED,
  //                              manip);

  // // set initial pose
  // setStartInstruction(
  //     program, kinematic_group->getJointNames(),
  //     env_->getCurrentJointValues(kinematic_group->getJointNames()));

  // MixedWaypoint waypoint(kinematic_group->getJointNames());

  // // set target pose
  // BaseObject::AttachLocation::ConstPtr attach_location_ptr =
  //     env.getAttachLocation(act->getAttachedObject());
  // Eigen::Isometry3d pick_pose_world_transform =
  //     env.getVKCEnv()->getTesseract()->getLinkTransform(
  //         attach_location_ptr->link_name_) *
  //     attach_location_ptr->local_joint_origin_transform;

  // waypoint.addLinkTarget(env.getEndEffectorLink(),
  // pick_pose_world_transform);

  // PlanInstruction pick_plan(waypoint, PlanInstructionType::FREESPACE,
  //                           "DEFAULT");
  // pick_plan.setDescription("waypoint for pick");
  // program.push_back(pick_plan);

  // // addCartWaypoint(program, pick_pose_world_transform, "pick object");

  // // generate seed
  // auto cur_state = env.getVKCEnv()->getTesseract()->getState();
  // ROS_DEBUG("generating seed");

  // // CompositeInstruction seed =
  // //     generateSeed(program, cur_state, env.getVKCEnv()->getTesseract());
  // CompositeInstruction seed = generateMixedSeed(
  //     program, cur_state, env.getVKCEnv()->getTesseract(), n_steps);

  // ROS_INFO("number of move instructions in pick seed: %ld",
  //          getMoveInstructionCount(seed));
  // ROS_INFO("composing request.");

  // seed.print("pick seed: ");

  // compose request
  PlannerRequest request;
  // request.name = process_planner_names::TRAJOPT_PLANNER_NAME;
  // request.instructions = program;
  // request.profiles = profiles;
  // request.seed = seed;
  // request.env_state = cur_state;
  // request.env = env.getVKCEnv()->getTesseract();

  // ROS_INFO("pick request generated.");

  return request;
}

trajopt::ProblemConstructionInfo ProbGenerator::genPickProb_test(
    VKCEnvBasic &env, PickAction::Ptr act, int n_steps) {
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());
  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID()); addJointTerm(pci, joint_num);
  // addCollisionTerm(pci, 0.01, 50);

  // BaseObject::AttachLocation::Ptr attach_location_ptr =
  // env.getAttachLocation(act->getAttachedObject());

  // std::shared_ptr<CartPoseTermInfo> PickPose =
  // std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  // PickPose->term_type = TT_CNT;
  // PickPose->name = "PickGoal";
  // PickPose->source_frame = env.getEndEffectorLink();
  // PickPose->timestep = pci.basic_info.n_steps - 1;
  // PickPose->target_frame = "world";
  // PickPose->target_frame_offset =
  // attach_location_ptr->world_joint_origin_transform;
  // // PickPose->xyz =
  // attach_location_ptr->world_joint_origin_transform.translation();
  // // PickPose->wxyz =
  // getQuatFromIso(attach_location_ptr->world_joint_origin_transform);
  // PickPose->pos_coeffs = Eigen::Vector3d(100.0, 100.0, 100.0);
  // PickPose->rot_coeffs = Eigen::Vector3d(100.0, 100.0, 100.0);
  // pci.cnt_infos.push_back(PickPose);
  // std::vector<LinkDesiredPose> link_objs;
  // std::vector<JointDesiredPose> joint_objs;
  // Eigen::Isometry3d ee_pose =
  // attach_location_ptr->world_joint_origin_transform;
  // link_objs.push_back(LinkDesiredPose(attach_location_ptr->connection.parent_link_name,
  // ee_pose));

  // pci.init_info.type = InitInfo::GIVEN_TRAJ;
  // pci.init_info.data = initTrajectory(env, link_objs, joint_objs, MapInfo(4,
  // 4, 0.2),
  //                                     pci.init_info.data, n_steps,
  //                                     act->getManipulatorID());

  return pci;
}
}  // namespace vkc
