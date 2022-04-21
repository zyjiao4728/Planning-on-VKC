#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/interface_utils.h>
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
  switch (action->getActionType()) {
    case ActionType::PickAction: {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      // initFinalPose(env, std::vector<LinkDesiredPose>(),
      // std::vector<JointDesiredPose>(), ActionType::PickAction);
      return genPickProb(env, pick_act, n_steps, n_iter);
    }

      // case ActionType::GotoAction:
      // {
      //   GotoAction::Ptr goto_act =
      //   std::dynamic_pointer_cast<GotoAction>(action); return
      //   genGotoProb(env, goto_act, n_steps);
      // }

    case ActionType::PlaceAction: {
      PlaceAction::Ptr place_act =
          std::dynamic_pointer_cast<PlaceAction>(action);
      return genPlaceProb(env, place_act, n_steps, n_iter);
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

// trajopt::ProblemConstructionInfo ProbGenerator::genProbTest(VKCEnvBasic &env,
// ActionBase::Ptr action, int n_steps)
// {
//   switch (action->getActionType())
//   {
//   case ActionType::PickAction:
//   {
//     PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
//     // initFinalPose(env, std::vector<LinkDesiredPose>(),
//     std::vector<JointDesiredPose>(), ActionType::PickAction); return
//     genPickProb_test(env, pick_act, n_steps);
//   }

//   case ActionType::GotoAction:
//   {
//     GotoAction::Ptr goto_act = std::dynamic_pointer_cast<GotoAction>(action);
//     return genGotoProb_test(env, goto_act, n_steps);
//   }

//   case ActionType::PlaceAction:
//   {
//     PlaceAction::Ptr place_act =
//     std::dynamic_pointer_cast<PlaceAction>(action); return
//     genPlaceProb_test(env, place_act, n_steps);
//   }

//   case ActionType::UseAction:
//   {
//     UseAction::Ptr use_act = std::dynamic_pointer_cast<UseAction>(action);
//     return genUseProb_test(env, use_act, n_steps);
//   }
//     // TODO: an null action should be defined for default case
//     //   default:
//     //   {
//     //     ROS_ERROR("Undefined action type.");
//     //     return NULL;
//     //   }
//   }
//   // return NULL;
// }

ProfileDictionary::Ptr ProbGenerator::genCartProfiles_(
    VKCEnvBasic &env, double collision_margin, double collision_coeff,
    Eigen::Vector3d pos_coeff, Eigen::Vector3d rot_coeff) {
  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();
  auto trajopt_composite_profile =
      std::make_shared<TrajOptDefaultCompositeProfile>();
  setCompositeProfile(trajopt_composite_profile, collision_margin,
                      collision_coeff,
                      (long int)env_->getActiveJointNames().size());

  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  setCartPlanProfile(trajopt_plan_profile, pos_coeff, rot_coeff);

  auto profiles = std::make_shared<ProfileDictionary>();
  profiles->addProfile<TrajOptCompositeProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
      trajopt_composite_profile);
  profiles->addProfile<TrajOptPlanProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile);

  return profiles;
}

PlannerRequest ProbGenerator::genPickProb(VKCEnvBasic &env, PickAction::Ptr act,
                                          int n_steps, int n_iter) {
  // to make sure the attach link exists
  if (nullptr == env.getAttachLocation(act->getAttachedObject())) {
    ROS_ERROR(
        "[%s]attach location named %s does not exist, please specified it "
        "firstly.",
        __func__, act->getAttachedObject().c_str());
    assert(false);
  }

  ROS_DEBUG("generating pick problem");

  ManipulatorInfo manip;
  manip.tcp_frame = env.getEndEffectorLink();
  manip.working_frame = "world";
  manip.manipulator = act->getManipulatorID();

  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();

  // set profiles
  double collision_margin = 0.0001;
  double collision_coeff = 10;

  auto pos_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);
  auto rot_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);

  auto profiles = genCartProfiles_(env, collision_margin, collision_coeff,
                                   pos_coeff, rot_coeff);
  setSolverProfile(profiles, n_iter);

  ROS_DEBUG("generating program");

  CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED,
                               manip);

  auto joint_group = env.getVKCEnv()->getTesseract()->getJointGroup(
      program.getManipulatorInfo().manipulator);

  // set initial pose
  setStartInstruction(
      program, joint_group->getJointNames(),
      env_->getCurrentJointValues(joint_group->getJointNames()));

  // set target pose
  BaseObject::AttachLocation::Ptr attach_location_ptr =
      env.getAttachLocation(act->getAttachedObject());
  Eigen::Isometry3d pick_pose_world_transform =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  addCartWaypoint(program, pick_pose_world_transform, "pick object");

  // generate seed
  auto cur_state = env.getVKCEnv()->getTesseract()->getState();
  ROS_DEBUG("generating seed");

  CompositeInstruction seed =
      generateSeed(program, cur_state, env.getVKCEnv()->getTesseract());

  ROS_INFO("composing request");

  // compose request
  PlannerRequest request;
  request.name = process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = program;
  request.profiles = profiles;
  request.seed = seed;
  request.env_state = cur_state;
  request.env = env.getVKCEnv()->getTesseract();

  ROS_WARN("pick request generated.");

  return request;
  // ProblemConstructionInfo pci(env_);
  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());

  // addJointTerm(pci, joint_num);
  // addCollisionTerm(pci, 0.0001, 10);
  // addCollisionTerm(pci, 0.01, 10);   // wanglei@bigai.ai, 2021-12-14

  // BaseObject::AttachLocation::Ptr attach_location_ptr =
  // env.getAttachLocation(act->getAttachedObject()); Eigen::Isometry3d
  // pick_pose_world_transform =
  // env.getVKCEnv()->getTesseract()->getLinkTransform(attach_location_ptr->link_name_)
  // *
  //                                               attach_location_ptr->local_joint_origin_transform;

  // std::shared_ptr<CartPoseTermInfo> PickPose =
  // std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  // PickPose->term_type = TT_CNT;
  // PickPose->name = "PickGoal";
  // PickPose->source_frame = env.getEndEffectorLink();
  // PickPose->timestep = pci.basic_info.n_steps - 1;
  // PickPose->target_frame = "world";
  // PickPose->target_frame_offset = pick_pose_world_transform;
  // // PickPose->xyz = pick_pose_world_transform.translation();
  // // PickPose->wxyz = getQuatFromIso(pick_pose_world_transform);
  // PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  // PickPose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  // pci.cnt_infos.push_back(PickPose);

  // // ROS_INFO("[%s]grasp pose: %f, %f, %f, grasp posture(wxyz): %f, %f, %f,
  // %f",
  // //          __func__, PickPose->xyz.x(), PickPose->xyz.y(),
  // PickPose->xyz.z(),
  // //          PickPose->wxyz.w(), PickPose->wxyz.x(), PickPose->wxyz.y(),
  // PickPose->wxyz.z());

  // std::vector<LinkDesiredPose> link_objs;
  // std::vector<JointDesiredPose> joint_objs;
  // Eigen::Isometry3d ee_pose = pick_pose_world_transform;
  // link_objs.push_back(LinkDesiredPose(attach_location_ptr->connection.parent_link_name,
  // ee_pose));

  // if (act->RequireInitTraj())
  // {
  //   pci.init_info.type = InitInfo::GIVEN_TRAJ;

  //   if (0 != act->getInitTraj().rows())
  //   {
  //     ROS_INFO("[%s]init trajectory with given in action object.", __func__);
  //     pci.init_info.data = act->getInitTraj();
  //   }
  //   else
  //   {
  //     ROS_INFO("[%s]init trajectory with generated by AStar.", __func__);
  //     pci.init_info.data = initTrajectory(env, link_objs, joint_objs,
  //                                         MapInfo(12, 12, 0.05),
  //                                         pci.init_info.data, n_steps,
  //                                         act->getManipulatorID());
  //   }
  // }
  // return ConstructProblem(pci);
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

PlannerRequest ProbGenerator::genPlaceProb(VKCEnvBasic &env,
                                           PlaceAction::Ptr act, int n_steps,
                                           int n_iter) {
  ROS_INFO("[@%s]generate a place problem with given data.", __func__);

  Environment::Ptr env_ = env.getVKCEnv()->getTesseract();

  BaseObject::AttachLocation::Ptr detach_location_ptr =
      env.getAttachLocation(act->getDetachedObject());
  ManipulatorInfo manip;
  manip.tcp_frame = detach_location_ptr->base_link_;
  manip.working_frame = "world";
  manip.manipulator = act->getManipulatorID();

  CompositeInstruction program("FREESPACE", CompositeInstructionOrder::ORDERED,
                               manip);

  double collision_margin = 0.0001;
  double collision_coeff = 10;

  auto pos_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);
  auto rot_coeff = Eigen::Vector3d(10.0, 10.0, 10.0);

  auto profiles = genCartProfiles_(env, collision_margin, collision_coeff,
                                   pos_coeff, rot_coeff);
  setSolverProfile(profiles, n_iter);

  setStartInstruction(program, env_->getActiveJointNames(),
                      env_->getCurrentJointValues());

  auto detach_pose = env_->getLinkTransform(detach_location_ptr->base_link_);
  addCartWaypoint(program, detach_pose, "pick object");

  act->addLinkObjectives(
      LinkDesiredPose(detach_location_ptr->base_link_, detach_pose));

  auto cur_state = env.getVKCEnv()->getTesseract()->getState();
  CompositeInstruction seed =
      generateSeed(program, cur_state, env.getVKCEnv()->getTesseract());

  // compose request
  PlannerRequest request;
  request.name = process_planner_names::TRAJOPT_PLANNER_NAME;
  request.instructions = program;
  request.profiles = profiles;
  request.seed = seed;
  request.env_state = cur_state;

  return request;
  // ProblemConstructionInfo
  // pci(std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());

  // addJointTerm(pci, joint_num);
  // addCollisionTerm(pci, 0.0001, 10);

  // BaseObject::AttachLocation::Ptr detach_location_ptr =
  // env.getAttachLocation(act->getDetachedObject()); if
  // (detach_location_ptr->fixed_base)
  // {
  //   ROS_INFO("[%s]generate a place problem for fixed base object.",
  //   __func__); for (int i = 0; i < pci.basic_info.n_steps; ++i)
  //   {
  //     std::shared_ptr<CartPoseTermInfo> BasePose =
  //     std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  //     BasePose->term_type = TT_CNT;
  //     BasePose->name = "BaseGoal_" + std::to_string(i);
  //     BasePose->source_frame = detach_location_ptr->base_link_;
  //     BasePose->timestep = i;
  //     BasePose->target_frame = "world";
  //     BasePose->target_frame_offset =
  //     env.getVKCEnv()->getTesseract()->getLinkTransform(detach_location_ptr->base_link_);
  //     // BasePose->xyz = env.getVKCEnv()
  //     //                     ->getTesseract()
  //     // ->getLinkTransform(detach_location_ptr->base_link_)
  //     //                     .translation();
  //     // BasePose->wxyz = getQuatFromIso(
  //     //
  //     env.getVKCEnv()->getTesseract()->getLinkTransform(detach_location_ptr->base_link_));
  //     BasePose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  //     BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  //     // BasePose->pos_coeffs = Eigen::Vector3d(1,1,1);
  //     // BasePose->rot_coeffs = Eigen::Vector3d(1,1,1);
  //     pci.cnt_infos.push_back(BasePose);
  //   }
  //   Eigen::Isometry3d ee_pose =
  //       env.getVKCEnv()->getTesseract()->getLinkTransform(detach_location_ptr->base_link_);
  //   act->addLinkObjectives(LinkDesiredPose(detach_location_ptr->base_link_,
  //   ee_pose));
  // }

  // for (auto &link_obj : act->getLinkObjectives())
  // {
  //   addTargetTerm(pci, link_obj, Eigen::Vector3d(10.0, 10.0, 10.0),
  //   Eigen::Vector3d(10.0, 10.0, 10.0));
  // }

  // for (auto &joint_obj : act->getJointObjectives())
  // {
  //   addTargetTerm(pci, joint_obj, joint_num, 100);
  // }

  // ROS_INFO("[%s]generate a place problem for %s object.", __func__,
  // (act->isRigidObject() ? "rigid" : "articulate"));
  // // if (detach_location_ptr->link_name_.find("cabinet") != std::string::npos
  // || detach_location_ptr->link_name_.find("dishwasher") != std::string::npos)
  // if (!act->isRigidObject())
  // {
  //   ROS_INFO("[%s]generate a place problem for articulate object.",
  //   __func__);
  //   // pci.init_info.type = InitInfo::GIVEN_TRAJ;
  //   initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(),
  //   MapInfo(12, 12, 0.2),
  //                  pci.init_info.data, n_steps, act->getManipulatorID());
  //   ROS_INFO("[%s]init trajectory size, rows: %ld, colums: %d", __func__,
  //   pci.init_info.data.rows(), pci.init_info.data.cols());

  //   Eigen::VectorXd end_pos;
  //   end_pos.resize(pci.kin->numJoints());
  //   end_pos.setZero();
  //   ROS_INFO("[%s](1)number of joints: %ld", __func__, pci.kin->numJoints());
  //   for (unsigned int j = 0; j < pci.kin->numJoints(); j++)
  //   {
  //     end_pos[j] = pci.init_info.data.topRows(1)(j);
  //   }
  //   ROS_INFO("[%s](2)number of joints: %ld", __func__, pci.kin->numJoints());
  //   end_pos(0) = pci.init_info.data.bottomRows(1)(0);
  //   end_pos(1) = pci.init_info.data.bottomRows(1)(1);
  //   if (pci.kin->numJoints() > 8)
  //   {
  //     end_pos(9) = act->getJointObjectives()[0].joint_angle;
  //   }
  //   ROS_INFO("[%s](3)number of joints: %ld", __func__, pci.kin->numJoints());
  //   pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  //   pci.init_info.data = end_pos;
  //   std::cout << "end pose: " << end_pos.transpose() << std::endl;
  // }
  // else
  // {
  //   // if (act->RequireInitTraj())
  //   {
  //     ROS_INFO("[%s]init trajectory for rigid object.", __func__);
  //     pci.init_info.type = InitInfo::GIVEN_TRAJ;
  //     if (0 != act->getInitTraj().rows())
  //     {
  //       ROS_INFO("[%s]init trajectory with given in action object.",
  //       __func__); pci.init_info.data = act->getInitTraj();
  //     }
  //     else
  //     {
  //       ROS_INFO("[%s]init trajectory with generated by AStar.", __func__);
  //       pci.init_info.data = initTrajectory(env, act->getLinkObjectives(),
  //       act->getJointObjectives(),
  //                                           MapInfo(12, 12, 0.2),
  //                                           pci.init_info.data, n_steps,
  //                                           act->getManipulatorID());
  //       // for (int k = 2; k < n_steps; k++)
  //       // {
  //       //   pci.init_info.data.row(k).rightCols(6) =
  //       pci.init_info.data.row(1).rightCols(6);
  //       // }
  //     }
  //   }
  // }

  // std::cout << pci.init_info.data(n_steps - 1,0) << ",\t" <<
  // pci.init_info.data(n_steps - 1,1) << std::endl;

  // return ConstructProblem(pci);
}

trajopt::ProblemConstructionInfo ProbGenerator::genPlaceProb_test(
    VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps) {
  ProblemConstructionInfo pci(
      std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());
  int joint_num = env.getVKCEnv()->getTesseract()->getActiveJointNames().size();

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 50);

  BaseObject::AttachLocation::Ptr detach_location_ptr =
      env.getAttachLocation(act->getDetachedObject());

  if (detach_location_ptr->fixed_base) {
    for (int i = 0; i < pci.basic_info.n_steps; ++i) {
      std::shared_ptr<CartPoseTermInfo> BasePose =
          std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
      BasePose->term_type = TT_CNT;
      BasePose->name = "BaseGoal_" + std::to_string(i);
      BasePose->source_frame = detach_location_ptr->base_link_;
      BasePose->timestep = i;
      BasePose->target_frame = "world";
      BasePose->target_frame_offset =
          env.getVKCEnv()->getTesseract()->getLinkTransform(
              detach_location_ptr->base_link_);
      // BasePose->xyz = env.getVKCEnv()
      //                     ->getTesseract()
      //                     ->getEnvironment()
      //                     ->getLinkTransform(detach_location_ptr->base_link_)
      //                     .translation();
      // BasePose->wxyz = getQuatFromIso(
      //     env.getVKCEnv()->getTesseract()->getLinkTransform(detach_location_ptr->base_link_));
      BasePose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      pci.cnt_infos.push_back(BasePose);
    }

    Eigen::Isometry3d ee_pose =
        env.getVKCEnv()->getTesseract()->getLinkTransform(
            detach_location_ptr->base_link_);
    act->addLinkObjectives(
        LinkDesiredPose(detach_location_ptr->base_link_, ee_pose));
  }

  for (auto &link_obj : act->getLinkObjectives()) {
    addTargetTerm(pci, link_obj, Eigen::Vector3d(100, 100, 100),
                  Eigen::Vector3d(100, 100, 100));
  }

  for (auto &joint_obj : act->getJointObjectives()) {
    addTargetTerm(pci, joint_obj, joint_num, 100);
  }
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(
      env, act->getLinkObjectives(), act->getJointObjectives(),
      MapInfo(4, 4, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
}

PlannerRequest ProbGenerator::genGotoProb(VKCEnvBasic &env, GotoAction::Ptr act,
                                          int n_steps) {
  ProblemConstructionInfo pci(
      std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());
  int joint_num = env.getVKCEnv()->getTesseract()->getActiveJointNames().size();

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);
  std::shared_ptr<CollisionTermInfo> collision =
      std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);

  for (auto &link_obj : act->getLinkObjectives()) {
    // ROS_ERROR("[%s]current end effector: %s, current link_obj name: %s",
    //           env.getEndEffectorLink().c_str(), link_obj.link_name.c_str());
    // assert(link_obj.link_name == env.getEndEffectorLink());

    addTargetTerm(pci, link_obj, Eigen::Vector3d(10, 10, 10),
                  Eigen::Vector3d(0, 0, 10));
  }

  for (auto &joint_obj : act->getJointObjectives()) {
    addTargetTerm(pci, joint_obj, joint_num, 10);
  }

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data =
      initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(),
                     MapInfo(12, 12, 0.05), pci.init_info.data, n_steps,
                     act->getManipulatorID());
  for (int k = 2; k < n_steps; k++) {
    pci.init_info.data.row(k).rightCols(6) =
        pci.init_info.data.row(1).rightCols(6);
  }

  // pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  // pci.init_info.data = initTrajectory(env, act->getLinkObjectives(),
  // act->getJointObjectives(), MapInfo(4, 4, 0.2),
  //                                     pci.init_info.data, n_steps,
  //                                     act->getManipulatorID());
  // pci.init_info.data = pci.init_info.data.bottomRows(1);

  std::cout << pci.init_info.data(n_steps - 1, 0) << ",\t"
            << pci.init_info.data(n_steps - 1, 1) << std::endl;

  // return ConstructProblem(pci);
  PlannerRequest request;
  return request;
}

trajopt::ProblemConstructionInfo ProbGenerator::genGotoProb_test(
    VKCEnvBasic &env, GotoAction::Ptr act, int n_steps) {
  ProblemConstructionInfo pci(
      std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());
  int joint_num = env.getVKCEnv()->getTesseract()->getActiveJointNames().size();

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 100);
  std::shared_ptr<CollisionTermInfo> collision =
      std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);

  for (auto &link_obj : act->getLinkObjectives()) {
    addTargetTerm(pci, link_obj, Eigen::Vector3d(5, 5, 0),
                  Eigen::Vector3d(0, 0, 0));
  }

  for (auto &joint_obj : act->getJointObjectives()) {
    addTargetTerm(pci, joint_obj, joint_num, 100);
  }

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(
      env, act->getLinkObjectives(), act->getJointObjectives(),
      MapInfo(4, 4, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
}

PlannerRequest ProbGenerator::genUseProb(VKCEnvBasic &env, UseAction::Ptr act,
                                         int n_steps) {
  ProblemConstructionInfo pci(
      std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());
  int joint_num = env.getVKCEnv()->getTesseract()->getActiveJointNames().size();

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);

  BaseObject::AttachLocation::Ptr attach_location_ptr =
      env.getAttachLocation(act->getAttachedObject());
  // update end effector
  if (act->getEndEffectorID() != "") {
    env.setEndEffector(act->getEndEffectorID());
  } else {
    std::cout << "Do not update end effector" << std::endl;
  }

  std::shared_ptr<CartPoseTermInfo> PickPose =
      std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->source_frame = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  Eigen::Isometry3d tf =
      attach_location_ptr->world_joint_origin_transform * act->getTransform();
  PickPose->target_frame = "world";
  PickPose->target_frame_offset = tf;
  // PickPose->xyz = tf.translation();
  // PickPose->wxyz = getQuatFromIso(tf);
  PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  PickPose->rot_coeffs = Eigen::Vector3d(0, 0, 0);
  pci.cnt_infos.push_back(PickPose);

  // return ConstructProblem(pci);
  PlannerRequest request;
  return request;
}

trajopt::ProblemConstructionInfo ProbGenerator::genUseProb_test(
    VKCEnvBasic &env, UseAction::Ptr act, int n_steps) {
  ProblemConstructionInfo pci(
      std::move(env.getVKCEnv()->getTesseract()->clone()));

  // int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps,
  // act->getManipulatorID());
  int joint_num = env.getVKCEnv()->getTesseract()->getActiveJointNames().size();

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 50);

  BaseObject::AttachLocation::Ptr attach_location_ptr =
      env.getAttachLocation(act->getAttachedObject());
  // update end effector
  if (act->getEndEffectorID() != "") {
    env.setEndEffector(act->getEndEffectorID());
  } else {
    std::cout << "Do not update end effector" << std::endl;
  }

  std::shared_ptr<CartPoseTermInfo> PickPose =
      std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->source_frame = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  Eigen::Isometry3d tf =
      attach_location_ptr->world_joint_origin_transform * act->getTransform();
  PickPose->target_frame = "world";
  PickPose->target_frame_offset = tf;
  // PickPose->xyz = tf.translation();
  // PickPose->wxyz = getQuatFromIso(tf);
  PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  PickPose->rot_coeffs = Eigen::Vector3d(0, 0, 0);
  pci.cnt_infos.push_back(PickPose);

  std::vector<LinkDesiredPose> link_objs;
  std::vector<JointDesiredPose> joint_objs;
  Eigen::Isometry3d ee_pose = attach_location_ptr->world_joint_origin_transform;
  link_objs.push_back(LinkDesiredPose(env.getEndEffectorLink(), ee_pose));

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data =
      initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2),
                     pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
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

  profile->velocity_coeff.setConstant(num_joints, 5);
  profile->velocity_coeff[0] = 30;
  profile->velocity_coeff[1] = 30;

  profile->acceleration_coeff.setConstant(num_joints, 5);
}

void ProbGenerator::setSolverProfile(ProfileDictionary::Ptr profiles,
                                     int n_iter) {
  auto trajopt_solver_profile =
      std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
  trajopt_solver_profile->opt_info.max_iter = n_iter;
  trajopt_solver_profile->opt_info.cnt_tolerance = 1e-3;
  trajopt_solver_profile->opt_info.trust_expand_ratio = 1.2;
  trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.8;
  trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;

  profiles->addProfile<TrajOptSolverProfile>(
      profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE",
      trajopt_solver_profile);

  return;
}

void ProbGenerator::setStartInstruction(CompositeInstruction &program,
                                        ros::V_string joint_names,
                                        Eigen::VectorXd joint_values) {
  Waypoint start_waypoint = StateWaypoint(joint_names, joint_values);
  PlanInstruction start_instruction(start_waypoint, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);
}

void ProbGenerator::addCartWaypoint(CompositeInstruction &program,
                                    Eigen::Isometry3d pose,
                                    std::string description) {
  PlanInstruction pick_plan(CartesianWaypoint(pose),
                            PlanInstructionType::FREESPACE, "FREESPACE");
  pick_plan.setDescription(description);
  program.push_back(pick_plan);
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

}  // namespace vkc
