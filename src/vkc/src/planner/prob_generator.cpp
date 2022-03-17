#include <vkc/planner/prob_generator.h>
#include <vkc/planner/traj_init.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

namespace vkc
{
ProbGenerator::ProbGenerator()
{
}

TrajOptProb::Ptr ProbGenerator::genProb(VKCEnvBasic &env, ActionBase::Ptr action, int n_steps)
{
  switch (action->getActionType())
  {
    case ActionType::PickAction:
    {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      // initFinalPose(env, std::vector<LinkDesiredPose>(), std::vector<JointDesiredPose>(), ActionType::PickAction);
      return genPickProb(env, pick_act, n_steps);
    }

    case ActionType::GotoAction:
    {
      GotoAction::Ptr goto_act = std::dynamic_pointer_cast<GotoAction>(action);
      return genGotoProb(env, goto_act, n_steps);
    }

    case ActionType::PlaceAction:
    {
      PlaceAction::Ptr place_act = std::dynamic_pointer_cast<PlaceAction>(action);
      return genPlaceProb(env, place_act, n_steps);
    }

    case ActionType::UseAction:
    {
      UseAction::Ptr use_act = std::dynamic_pointer_cast<UseAction>(action);
      return genUseProb(env, use_act, n_steps);
    }

    default:
    {
      ROS_ERROR("Undefined action type.");
      return nullptr;
    }
  }
  return nullptr;
}

trajopt::ProblemConstructionInfo ProbGenerator::genProbTest(VKCEnvBasic &env, ActionBase::Ptr action, int n_steps)
{
  switch (action->getActionType())
  {
    case ActionType::PickAction:
    {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      // initFinalPose(env, std::vector<LinkDesiredPose>(), std::vector<JointDesiredPose>(), ActionType::PickAction);
      return genPickProb_test(env, pick_act, n_steps);
    }

    case ActionType::GotoAction:
    {
      GotoAction::Ptr goto_act = std::dynamic_pointer_cast<GotoAction>(action);
      return genGotoProb_test(env, goto_act, n_steps);
    }

    case ActionType::PlaceAction:
    {
      PlaceAction::Ptr place_act = std::dynamic_pointer_cast<PlaceAction>(action);
      return genPlaceProb_test(env, place_act, n_steps);
    }

    case ActionType::UseAction:
    {
      UseAction::Ptr use_act = std::dynamic_pointer_cast<UseAction>(action);
      return genUseProb_test(env, use_act, n_steps);
    }
    // TODO: an null action should be defined for default case 
  //   default:
  //   {
  //     ROS_ERROR("Undefined action type.");
  //     return NULL;
  //   }
  }
  // return NULL;
}

int ProbGenerator::initProbInfo(ProblemConstructionInfo &pci, tesseract::Tesseract::Ptr tesseract, int n_steps,
                                std::string manip)
{
  // Populate Basic Info
  pci.basic_info.n_steps = n_steps;
  pci.basic_info.manip = manip;
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;
  // pci.basic_info.dofs_fixed = std::vector<int>({ 2 });  // jiao@2021-11-12, to disable rotation joint of mobile base 

  // validate if group id exist in the tesseract env
  if (!validateGroupID(tesseract, pci.basic_info.manip))
    exit(1);

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Populate Init Info
  EnvState::ConstPtr current_state = pci.env->getCurrentState();
  Eigen::VectorXd start_pos;
  start_pos.resize(pci.kin->numJoints());
  int joint_num = 0;
  planned_joints.clear();
  for (const auto &j : pci.kin->getJointNames())
  {
    start_pos[joint_num] = current_state->joints.at(j);
    planned_joints[j] = joint_num;
    ++joint_num;
  }

  pci.init_info.type = InitInfo::STATIONARY;

  // // [ToDo] how to init the initial trajectory
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  return joint_num;
}

TrajOptProb::Ptr ProbGenerator::genPickProb(VKCEnvBasic &env, PickAction::Ptr act, int n_steps)
{
  // to make sure the attach link exists
  if (nullptr == env.getAttachLocation(act->getAttachedObject()))
  {
    ROS_ERROR("[%s]attach location named %s does not exist, please specified it firstly.",
              __func__, act->getAttachedObject().c_str());
    assert(false);
  }

  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());
  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);
  // addCollisionTerm(pci, 0.01, 10);   // wanglei@bigai.ai, 2021-12-14

  BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());
  Eigen::Isometry3d object_world_transform = env.getVKCEnv()->getTesseract()->getEnvironment()->getLinkTransform(attach_location_ptr->link_name_);
  Eigen::Isometry3d pick_pose_world_transform = env.getVKCEnv()->getTesseract()->getEnvironment()->getLinkTransform(attach_location_ptr->link_name_) *
                                              attach_location_ptr->local_joint_origin_transform;

  std::shared_ptr<CartPoseTermInfo> PickPose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->link = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  PickPose->xyz = pick_pose_world_transform.translation();
  PickPose->wxyz = getQuatFromIso(pick_pose_world_transform);
  PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  PickPose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  pci.cnt_infos.push_back(PickPose);

  ROS_INFO("[%s]grasp pose: %f, %f, %f, grasp posture(wxyz): %f, %f, %f, %f",
           __func__, PickPose->xyz.x(), PickPose->xyz.y(), PickPose->xyz.z(),
           PickPose->wxyz.w(), PickPose->wxyz.x(), PickPose->wxyz.y(), PickPose->wxyz.z());

  ROS_INFO("[%s]object pose: %f, %f, %f",       __func__, object_world_transform.translation().x(), object_world_transform.translation().y(), object_world_transform.translation().z());

  std::vector<LinkDesiredPose> link_objs;
  std::vector<JointDesiredPose> joint_objs;
  Eigen::Isometry3d ee_pose = pick_pose_world_transform;
  link_objs.push_back(LinkDesiredPose(attach_location_ptr->connection.parent_link_name, ee_pose));

  if(act->RequireInitTraj())
  {
      pci.init_info.type = InitInfo::GIVEN_TRAJ;
      
      if(0 != act->getInitTraj().rows())
      {
        ROS_INFO("[%s]init trajectory with given in action object.", __func__);
        pci.init_info.data = act->getInitTraj();
      }
      else
      {
        ROS_INFO("[%s]init trajectory with generated by AStar.", __func__);
        pci.init_info.data = initTrajectory(env, link_objs, joint_objs,
                                            MapInfo(12, 12, 0.05), pci.init_info.data, n_steps, act->getManipulatorID());
        // for (int k = 2; k < n_steps; k++)
        // {
        //   pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
        // }
      }
  }
  // else if (act->getInitTraj().rows() == 1)
  // {
  //   pci.init_info.type = InitInfo::JOINT_INTERPOLATED;

  // }

  // if (attach_location_ptr->link_name_.find("marker") == std::string::npos)
  // {
  //   pci.init_info.type = InitInfo::GIVEN_TRAJ;
  //   pci.init_info.data = initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2), pci.init_info.data, n_steps);
  //   for (int k = 2; k < n_steps; k++)
  //   {
  //     pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
  //   }
  // }

  // initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());
  // Eigen::VectorXd end_pos;
  // end_pos.resize(pci.kin->numJoints());
  // for (int j = 0; j < pci.kin->numJoints(); j++)
  // {
  //   end_pos[j] = pci.init_info.data.bottomRows(1)(j);
  // }
  
  // pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  // pci.init_info.data = end_pos;
  // std::cout << end_pos << std::endl;

  // tesseract_environment::EnvState::ConstPtr current_state = pci.env->getCurrentState();
  // Eigen::VectorXd start_pos;
  // start_pos.resize(pci.kin->numJoints());
  // int njoints = 0;
  // for (const auto &j : pci.kin->getJointNames())
  // {
  //   start_pos[njoints] = current_state->joints.at(j);
  //   ++njoints;
  // }

  // Eigen::VectorXd end_pos;
  // end_pos.resize(pci.kin->numJoints());
  // end_pos = start_pos;
  // pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  // end_pos(0) = pci.init_info.data(n_steps - 1, 0);;
  // end_pos(1) = pci.init_info.data(n_steps - 1, 1);;
  // pci.init_info.data = end_pos;
  return ConstructProblem(pci);
}

trajopt::ProblemConstructionInfo ProbGenerator::genPickProb_test(VKCEnvBasic &env, PickAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());
  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());
  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 50);
  
  BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());

  std::shared_ptr<CartPoseTermInfo> PickPose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->link = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  PickPose->xyz = attach_location_ptr->world_joint_origin_transform.translation();
  PickPose->wxyz = getQuatFromIso(attach_location_ptr->world_joint_origin_transform);
  PickPose->pos_coeffs = Eigen::Vector3d(100.0, 100.0, 100.0);
  PickPose->rot_coeffs = Eigen::Vector3d(100.0, 100.0, 100.0);
  pci.cnt_infos.push_back(PickPose);
  std::vector<LinkDesiredPose> link_objs;
  std::vector<JointDesiredPose> joint_objs;
  Eigen::Isometry3d ee_pose = attach_location_ptr->world_joint_origin_transform;
  link_objs.push_back(LinkDesiredPose(attach_location_ptr->connection.parent_link_name, ee_pose));

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2),
                                      pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
}

TrajOptProb::Ptr ProbGenerator::genPlaceProb(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps)
{
  ROS_INFO("[@%s]generate a place problem with given data.", __func__);
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);  
  
  BaseObject::AttachLocation::Ptr detach_location_ptr = env.getAttachLocation(act->getDetachedObject());
  if (detach_location_ptr->fixed_base)
  {
    ROS_INFO("[%s]generate a place problem for fixed base object.", __func__);
    for (int i = 0; i < pci.basic_info.n_steps; ++i)
    {
      std::shared_ptr<CartPoseTermInfo> BasePose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
      BasePose->term_type = TT_CNT;
      BasePose->name = "BaseGoal_" + std::to_string(i);
      BasePose->link = detach_location_ptr->base_link_;
      BasePose->timestep = i;
      BasePose->xyz = env.getVKCEnv()
                          ->getTesseractEnvironment()
                          ->getLinkTransform(detach_location_ptr->base_link_)
                          .translation();
      BasePose->wxyz = getQuatFromIso(
          env.getVKCEnv()->getTesseractEnvironment()->getLinkTransform(detach_location_ptr->base_link_));
      BasePose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      // BasePose->pos_coeffs = Eigen::Vector3d(1,1,1);
      // BasePose->rot_coeffs = Eigen::Vector3d(1,1,1);
      pci.cnt_infos.push_back(BasePose);
    }
    Eigen::Isometry3d ee_pose =
        env.getVKCEnv()->getTesseractEnvironment()->getLinkTransform(detach_location_ptr->base_link_);
    act->addLinkObjectives(LinkDesiredPose(detach_location_ptr->base_link_, ee_pose));
  }


  for (auto &link_obj : act->getLinkObjectives())
  {
    addTargetTerm(pci, link_obj, Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(10.0, 10.0, 10.0));
  }

  for (auto &joint_obj : act->getJointObjectives())
  {
    addTargetTerm(pci, joint_obj, joint_num, 100);
  }

  ROS_INFO("[%s]generate a place problem for %s object.", __func__, (act->isRigidObject() ? "rigid" : "articulate"));
  // if (detach_location_ptr->link_name_.find("cabinet") != std::string::npos || detach_location_ptr->link_name_.find("dishwasher") != std::string::npos)
  if(!act->isRigidObject())
  {
    ROS_INFO("[%s]generate a place problem for articulate object.", __func__);
    // pci.init_info.type = InitInfo::GIVEN_TRAJ;
    initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(12, 12, 0.2),
                                        pci.init_info.data, n_steps, act->getManipulatorID());
    ROS_INFO("[%s]init trajectory size, rows: %d, colums: %d", __func__, pci.init_info.data.rows(), pci.init_info.data.cols());
    
    Eigen::VectorXd end_pos;
    end_pos.resize(pci.kin->numJoints());
    end_pos.setZero();
    ROS_INFO("[%s](1)number of joints: %d", __func__, pci.kin->numJoints());
    for (unsigned int j = 0; j < pci.kin->numJoints(); j++)
    {
      end_pos[j] = pci.init_info.data.topRows(1)(j);
    }
    ROS_INFO("[%s](2)number of joints: %d", __func__, pci.kin->numJoints());
    end_pos(0) = pci.init_info.data.bottomRows(1)(0);
    end_pos(1) = pci.init_info.data.bottomRows(1)(1);
    if (pci.kin->numJoints() > 8)
    {
      end_pos(9) = act->getJointObjectives()[0].joint_angle;
    }
    ROS_INFO("[%s](3)number of joints: %d", __func__, pci.kin->numJoints());
    pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
    pci.init_info.data = end_pos;
    std::cout << "end pose: " << end_pos.transpose() << std::endl;
  }
  else
  {
    if (act->RequireInitTraj())
    {
      ROS_INFO("[%s]init trajectory for rigid object.", __func__);
      pci.init_info.type = InitInfo::GIVEN_TRAJ;
      if(0 != act->getInitTraj().rows())
      {
        ROS_INFO("[%s]init trajectory with given in action object.", __func__);
        pci.init_info.data = act->getInitTraj();
      }
      else
      {
        ROS_INFO("[%s]init trajectory with generated by AStar.", __func__);
        pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), 
        MapInfo(12, 12, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());
        // for (int k = 2; k < n_steps; k++)
        // {
        //   pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
        // }
      }
    }
  }
  
  // std::cout << pci.init_info.data(n_steps - 1,0) << ",\t" << pci.init_info.data(n_steps - 1,1) << std::endl;
  
  return ConstructProblem(pci);
}

trajopt::ProblemConstructionInfo ProbGenerator::genPlaceProb_test(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 50);

  BaseObject::AttachLocation::Ptr detach_location_ptr = env.getAttachLocation(act->getDetachedObject());

  if (detach_location_ptr->fixed_base)
  {
    for (int i = 0; i < pci.basic_info.n_steps; ++i)
    {
      std::shared_ptr<CartPoseTermInfo> BasePose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
      BasePose->term_type = TT_CNT;
      BasePose->name = "BaseGoal_" + std::to_string(i);
      BasePose->link = detach_location_ptr->base_link_;
      BasePose->timestep = i;
      BasePose->xyz = env.getVKCEnv()
                          ->getTesseract()
                          ->getEnvironment()
                          ->getLinkTransform(detach_location_ptr->base_link_)
                          .translation();
      BasePose->wxyz = getQuatFromIso(
          env.getVKCEnv()->getTesseract()->getEnvironment()->getLinkTransform(detach_location_ptr->base_link_));
      BasePose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
      pci.cnt_infos.push_back(BasePose);
    }

    Eigen::Isometry3d ee_pose =
        env.getVKCEnv()->getTesseract()->getEnvironment()->getLinkTransform(detach_location_ptr->base_link_);
    act->addLinkObjectives(LinkDesiredPose(detach_location_ptr->base_link_, ee_pose));
  }

  for (auto &link_obj : act->getLinkObjectives())
  {
    addTargetTerm(pci, link_obj, Eigen::Vector3d(100, 100, 100), Eigen::Vector3d(100, 100, 100));
  }

  for (auto &joint_obj : act->getJointObjectives())
  {
    addTargetTerm(pci, joint_obj, joint_num, 100);
  }
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(4, 4, 0.2),
                                      pci.init_info.data, n_steps, act->getManipulatorID());


  return pci;
}

TrajOptProb::Ptr ProbGenerator::genGotoProb(VKCEnvBasic &env, GotoAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);
  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);

  for (auto &link_obj : act->getLinkObjectives())
  {
    // ROS_ERROR("[%s]current end effector: %s, current link_obj name: %s",
    //           env.getEndEffectorLink().c_str(), link_obj.link_name.c_str());
    // assert(link_obj.link_name == env.getEndEffectorLink());

    addTargetTerm(pci, link_obj, Eigen::Vector3d(10, 10, 10), Eigen::Vector3d(0, 0, 10));
  }

  for (auto &joint_obj : act->getJointObjectives())
  {
    addTargetTerm(pci, joint_obj, joint_num, 10);
  }

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(12, 12, 0.05),
                                      pci.init_info.data, n_steps, act->getManipulatorID());
  for (int k = 2; k < n_steps; k++)
  {
    pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
  }

  // pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
  // pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(4, 4, 0.2),
  //                                     pci.init_info.data, n_steps, act->getManipulatorID());
  // pci.init_info.data = pci.init_info.data.bottomRows(1);
                                  
  std::cout << pci.init_info.data(n_steps - 1,0) << ",\t" << pci.init_info.data(n_steps - 1,1) << std::endl;

  return ConstructProblem(pci);
}

trajopt::ProblemConstructionInfo ProbGenerator::genGotoProb_test(VKCEnvBasic &env, GotoAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 100);
  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);

  for (auto &link_obj : act->getLinkObjectives())
  {
    addTargetTerm(pci, link_obj, Eigen::Vector3d(5, 5, 0), Eigen::Vector3d(0, 0, 0));
  }

  for (auto &joint_obj : act->getJointObjectives())
  {
    addTargetTerm(pci, joint_obj, joint_num, 100);
  }

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(),
   MapInfo(4, 4, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
}

TrajOptProb::Ptr ProbGenerator::genUseProb(VKCEnvBasic &env, UseAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.0001, 10);

  BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());
  // update end effector
  if(act->getEndEffectorID() != "") {
    env.setEndEffector(act->getEndEffectorID());
  }
  else {
    std::cout << "Do not update end effector" << std::endl;
  }
  


  std::shared_ptr<CartPoseTermInfo> PickPose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->link = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  Eigen::Isometry3d tf = attach_location_ptr->world_joint_origin_transform * act->getTransform();
  PickPose->xyz = tf.translation();
  PickPose->wxyz = getQuatFromIso(tf);
  PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  PickPose->rot_coeffs = Eigen::Vector3d(0, 0, 0);
  pci.cnt_infos.push_back(PickPose);

  return ConstructProblem(pci);
}

trajopt::ProblemConstructionInfo ProbGenerator::genUseProb_test(VKCEnvBasic &env, UseAction::Ptr act, int n_steps)
{
  ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

  int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

  addJointTerm(pci, joint_num);
  addCollisionTerm(pci, 0.01, 50);

  BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());
  // update end effector
  if(act->getEndEffectorID() != "") {
    env.setEndEffector(act->getEndEffectorID());
  }
  else {
    std::cout << "Do not update end effector" << std::endl;
  }
  
  std::shared_ptr<CartPoseTermInfo> PickPose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  PickPose->term_type = TT_CNT;
  PickPose->name = "PickGoal";
  PickPose->link = env.getEndEffectorLink();
  PickPose->timestep = pci.basic_info.n_steps - 1;
  Eigen::Isometry3d tf = attach_location_ptr->world_joint_origin_transform * act->getTransform();
  PickPose->xyz = tf.translation();
  PickPose->wxyz = getQuatFromIso(tf);
  PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  PickPose->rot_coeffs = Eigen::Vector3d(0, 0, 0);
  pci.cnt_infos.push_back(PickPose);

  std::vector<LinkDesiredPose> link_objs;
  std::vector<JointDesiredPose> joint_objs;
  Eigen::Isometry3d ee_pose = attach_location_ptr->world_joint_origin_transform;
  link_objs.push_back(LinkDesiredPose(env.getEndEffectorLink(), ee_pose));

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2),
                                      pci.init_info.data, n_steps, act->getManipulatorID());

  return pci;
}

bool ProbGenerator::validateGroupID(tesseract::Tesseract::Ptr tesseract, const std::string &group_id)
{
  bool isfound_group = false;

  for (const auto &groups_ : tesseract->getSRDFModel()->getGroups())
  {
    if (groups_.name_ == group_id)
    {
      isfound_group = true;
      ROS_INFO("Found group %s.", group_id.c_str());
    }
  }

  if (!isfound_group)
  {
    ROS_ERROR("Cannot find group %s.", group_id.c_str());
  }

  return isfound_group;
}

Eigen::Vector4d ProbGenerator::getQuatFromIso(Eigen::Isometry3d iso)
{
  return Eigen::Vector4d(Eigen::Quaterniond(iso.rotation()).w(), Eigen::Quaterniond(iso.rotation()).x(),
                         Eigen::Quaterniond(iso.rotation()).y(), Eigen::Quaterniond(iso.rotation()).z());
}

void ProbGenerator::addJointTerm(ProblemConstructionInfo &pci, int joint_num)
{
  // the following if expression is added to avoid exception while to allocate an empty vector 
  // added by: wanglei@bigai.ai, 2021-11-08
  if(0 == joint_num)
  {
    ROS_INFO("[%s]joint number: %d", __func__, joint_num);
    return;
  }

  // Populate Cost Info
  std::shared_ptr<JointVelTermInfo> jv_cost = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv_cost->coeffs = std::vector<double>(static_cast<unsigned long int>(joint_num), 5);
  jv_cost->coeffs[0] = 30; //(rand() % 100) / 5 + 30;
  jv_cost->coeffs[1] = jv_cost->coeffs[0];
  jv_cost->targets = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jv_cost->first_step = 0;
  jv_cost->last_step = pci.basic_info.n_steps - 1;
  jv_cost->name = "joint_vel_cost";
  jv_cost->term_type = TT_COST;
  pci.cost_infos.push_back(jv_cost);


  std::shared_ptr<JointVelTermInfo> jv_limit = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  jv_limit->coeffs = std::vector<double>(static_cast<unsigned long int>(joint_num), 100.0);
  jv_limit->targets = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jv_limit->upper_tols = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.5);
  jv_limit->lower_tols = std::vector<double>(static_cast<unsigned long int>(joint_num),-0.5);
  jv_limit->first_step = 0;
  jv_limit->last_step = pci.basic_info.n_steps - 1;
  jv_limit->name = "joint_vel_limit";
  jv_limit->term_type = TT_CNT;
  pci.cnt_infos.push_back(jv_limit);


  std::shared_ptr<JointAccTermInfo> ja_cost = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  ja_cost->coeffs = std::vector<double>(static_cast<unsigned long int>(joint_num), 5.0);
  ja_cost->targets = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  ja_cost->first_step = 0;
  ja_cost->last_step = pci.basic_info.n_steps - 1;
  ja_cost->name = "joint_acc_cost";
  ja_cost->term_type = TT_COST;
  pci.cost_infos.push_back(ja_cost);


  std::shared_ptr<JointAccTermInfo> ja_limit = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  ja_limit->coeffs = std::vector<double>(static_cast<unsigned long int>(joint_num), 100.0);
  ja_limit->targets = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  ja_limit->upper_tols = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.3);
  ja_limit->lower_tols = std::vector<double>(static_cast<unsigned long int>(joint_num), -0.3);
  ja_limit->first_step = 0;
  ja_limit->last_step = pci.basic_info.n_steps - 1;
  ja_limit->name = "joint_acc_limit";
  ja_limit->term_type = TT_CNT;
  pci.cnt_infos.push_back(ja_limit);
}

void ProbGenerator::addCollisionTerm(ProblemConstructionInfo &pci, double margin, double coeff)
{
  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->name = "collision";
  collision->term_type = TT_CNT;
  collision->continuous = true;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 2;
  // collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, margin, coeff);
  pci.cnt_infos.push_back(collision);
}

void ProbGenerator::addTargetTerm(ProblemConstructionInfo &pci, LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                                  Eigen::Vector3d rot_coeff)
{
  std::shared_ptr<CartPoseTermInfo> Link_Goal = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  Link_Goal->term_type = TT_CNT;
  Link_Goal->name = link_pose.link_name + "_Goal";
  Link_Goal->link = link_pose.link_name;
  Link_Goal->timestep = pci.basic_info.n_steps - 1;
  Link_Goal->xyz = link_pose.tf.translation();
  Link_Goal->wxyz = getQuatFromIso(link_pose.tf);
  Link_Goal->pos_coeffs = pos_coeff;
  Link_Goal->rot_coeffs = rot_coeff;
  pci.cnt_infos.push_back(Link_Goal);
}

void ProbGenerator::addTargetTerm(ProblemConstructionInfo &pci, JointDesiredPose &joint_pose, int joint_num,
                                  double coeff)
{
  // the following if expression is added to avoid exception while to allocate an empty vector 
  // added by: wanglei@bigai.ai, 2021-11-08
  if(0 == joint_num)
  {
    ROS_INFO("[%s]joint number: %d", __func__, joint_num);
    return;
  }

  ROS_INFO("[%s]joint number: %d, joint: %s, target value: %f",
           __func__, joint_num, joint_pose.joint_name.c_str(), joint_pose.joint_angle);
   

  std::shared_ptr<JointPosTermInfo> jp = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
  jp->coeffs = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jp->coeffs[planned_joints.at(joint_pose.joint_name)] = coeff;
  jp->targets = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.0);
  jp->targets[planned_joints.at(joint_pose.joint_name)] = joint_pose.joint_angle;
  jp->upper_tols = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.001);
  jp->lower_tols = std::vector<double>(static_cast<unsigned long int>(joint_num), 0.001);
  jp->first_step = pci.basic_info.n_steps - 1;
  jp->last_step = pci.basic_info.n_steps - 1;
  jp->name = joint_pose.joint_name + "_Goal";
  jp->term_type = TT_CNT;
  pci.cnt_infos.push_back(jp);

  ROS_INFO("[%s]add target finished", __func__);
}

void ProbGenerator::addTargetCost(ProblemConstructionInfo &pci, LinkDesiredPose &link_pose, Eigen::Vector3d pos_coeff,
                                  Eigen::Vector3d rot_coeff)
{
  std::shared_ptr<CartPoseTermInfo> Link_Goal = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
  Link_Goal->term_type = TT_COST;
  Link_Goal->name = link_pose.link_name + "_Goal";
  Link_Goal->link = link_pose.link_name;
  Link_Goal->timestep = pci.basic_info.n_steps - 1;
  Link_Goal->xyz = link_pose.tf.translation();
  Link_Goal->wxyz = getQuatFromIso(link_pose.tf);
  Link_Goal->pos_coeffs = pos_coeff;
  Link_Goal->rot_coeffs = rot_coeff;
  pci.cost_infos.push_back(Link_Goal);
}


// trajopt::TrajOptProb::Ptr ProbGenerator::genPickProbOMPL(VKCEnvBasic &env, PickAction::Ptr act, int n_steps)
// {
//   ProblemConstructionInfo pci(env.getVKCEnv()->getTesseract());

//   int joint_num = initProbInfo(pci, env.getVKCEnv()->getTesseract(), n_steps, act->getManipulatorID());

//   addJointTerm(pci, joint_num);
//   addCollisionTerm(pci, 0.0001, 10);

//   BaseObject::AttachLocation::Ptr attach_location_ptr = env.getAttachLocation(act->getAttachedObject());

//   std::shared_ptr<CartPoseTermInfo> PickPose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
//   PickPose->term_type = TT_CNT;
//   PickPose->name = "PickGoal";
//   PickPose->link = env.getEndEffectorLink();
//   PickPose->timestep = pci.basic_info.n_steps - 1;
//   PickPose->xyz = attach_location_ptr->world_joint_origin_transform.translation();
//   PickPose->wxyz = getQuatFromIso(attach_location_ptr->world_joint_origin_transform);
//   PickPose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
//   PickPose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);

//   pci.cnt_infos.push_back(PickPose);

//   std::cout << PickPose->xyz << '\n' << PickPose->wxyz << std::endl;

//   std::vector<LinkDesiredPose> link_objs;
//   std::vector<JointDesiredPose> joint_objs;
//   Eigen::Isometry3d ee_pose = attach_location_ptr->world_joint_origin_transform;
//   link_objs.push_back(LinkDesiredPose(attach_location_ptr->connection.parent_link_name, ee_pose));

//   if (attach_location_ptr->link_name_.find("marker") == std::string::npos)
//   {
//     pci.init_info.type = InitInfo::GIVEN_TRAJ;
//     // pci.init_info.data = initTrajectoryByOMPL(env, act, n_steps);
//     pci.init_info.data = initTrajectory(env, link_objs, joint_objs, MapInfo(4, 4, 0.2), pci.init_info.data, n_steps, act->getManipulatorID());
//     for (int k = 2; k < n_steps; k++)
//     {
//       pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
//     }
//   }

//   return ConstructProblem(pci);
// }

// trajopt::TrajOptProb::Ptr ProbGenerator::genPlaceProbOMPL(VKCEnvBasic &env, PlaceAction::Ptr act, int n_steps)
// {
//   auto tesseract = env.getVKCEnv()->getTesseract();
//   auto tesseract_env = tesseract->getEnvironment();
//   ProblemConstructionInfo pci(tesseract);
//   int joint_num = initProbInfo(pci, tesseract, n_steps, act->getManipulatorID());

//   addJointTerm(pci, joint_num);
//   addCollisionTerm(pci, 0.0001, 10);

//   BaseObject::AttachLocation::Ptr detach_location_ptr = env.getAttachLocation(act->getDetachedObject());
//   if (detach_location_ptr->fixed_base)
//   {
//     for (int i = 0; i < pci.basic_info.n_steps; ++i)
//     {
//       std::shared_ptr<CartPoseTermInfo> BasePose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
//       BasePose->term_type = TT_CNT;
//       BasePose->name = "BaseGoal_" + std::to_string(i);
//       BasePose->link = detach_location_ptr->base_link_;
//       BasePose->timestep = i;
//       BasePose->xyz = tesseract_env->getLinkTransform(detach_location_ptr->base_link_).translation();
//       BasePose->wxyz = getQuatFromIso(tesseract_env->getLinkTransform(detach_location_ptr->base_link_));
//       BasePose->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
//       BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);

//       pci.cnt_infos.push_back(BasePose);
//     }

//     Eigen::Isometry3d ee_pose = tesseract_env->getLinkTransform(detach_location_ptr->base_link_);
//     act->addLinkObjectives(LinkDesiredPose(detach_location_ptr->base_link_, ee_pose));
//   }


//   if (detach_location_ptr->connection.parent_link_name.find("stick") != std::string::npos)
//   {
//     for (int i = 0; i < pci.basic_info.n_steps; ++i)
//     {
//       std::shared_ptr<CartPoseTermInfo> BasePose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
//       BasePose->term_type = TT_CNT;
//       BasePose->name = "BaseGoal_" + std::to_string(i);
//       BasePose->link = detach_location_ptr->base_link_;
//       BasePose->timestep = i;
//       BasePose->xyz = tesseract_env->getLinkTransform(detach_location_ptr->base_link_).translation();
//       BasePose->wxyz = getQuatFromIso(tesseract_env->getLinkTransform(detach_location_ptr->base_link_));
//       BasePose->pos_coeffs = Eigen::Vector3d(0.0, 0.0, 10.0);
//       BasePose->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 0.0);

//       pci.cnt_infos.push_back(BasePose);
//     }
//     Eigen::Isometry3d ee_pose =tesseract_env->getLinkTransform(detach_location_ptr->base_link_);
//     act->addLinkObjectives(LinkDesiredPose(detach_location_ptr->base_link_, ee_pose));

//     for (auto &link_obj : act->getLinkObjectives())
//     {
//       addTargetTerm(pci, link_obj, Eigen::Vector3d(0.0, 10.0, 0.0), Eigen::Vector3d(10.0, 10.0, 0.0));
//     }
//   }
//   else
//   {
//     for (auto &link_obj : act->getLinkObjectives())
//     {
//       addTargetTerm(pci, link_obj, Eigen::Vector3d(10.0, 10.0, 10.0), Eigen::Vector3d(10.0, 0.0, 10.0));
//     }
//   }

//   for (auto &joint_obj : act->getJointObjectives())
//   {
//     addTargetTerm(pci, joint_obj, joint_num, 100);
//   }

  
//   if (detach_location_ptr->link_name_.find("cabinet") != std::string::npos || detach_location_ptr->link_name_.find("dishwasher") != std::string::npos)
//   {
//     // pci.init_info.type = InitInfo::GIVEN_TRAJ;
//     initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(4, 4, 0.2),
//                                         pci.init_info.data, n_steps, act->getManipulatorID());
//     Eigen::VectorXd end_pos;
//     end_pos.resize(pci.kin->numJoints());
//     end_pos.setZero();
//     for (unsigned int j = 0; j < pci.kin->numJoints(); j++)
//     {
//       end_pos[j] = pci.init_info.data.topRows(1)(j);
//     }
//     end_pos(0) = pci.init_info.data.bottomRows(1)(0);
//     end_pos(1) = pci.init_info.data.bottomRows(1)(1);
//     if (pci.kin->numJoints() > 8)
//     {
//       end_pos(9) = act->getJointObjectives()[0].joint_angle;
//     }
//     pci.init_info.type = InitInfo::JOINT_INTERPOLATED;
//     pci.init_info.data = end_pos;
//     std::cout << "end pose: " << end_pos.transpose() << std::endl;
//   }
//   // else if (detach_location_ptr->connection.parent_link_name.find("marker") != std::string::npos)
//   // {
//   //   /* code */
//   // }
  
//   else if (detach_location_ptr->connection.parent_link_name.find("stick") == std::string::npos && env.getEndEffectorLink().find("stick") == std::string::npos)
//   {
//     pci.init_info.type = InitInfo::GIVEN_TRAJ;
//     // pci.init_info.data = initTrajectoryByOMPL(env, act, n_steps);
//     pci.init_info.data = initTrajectory(env, act->getLinkObjectives(), act->getJointObjectives(), MapInfo(4, 4, 0.2),
//                                       pci.init_info.data, n_steps, act->getManipulatorID());
//     for (int k = 2; k < n_steps; k++)
//     {
//       pci.init_info.data.row(k).rightCols(6) = pci.init_info.data.row(1).rightCols(6);
//     }
//   }  

  
//   return ConstructProblem(pci);    
// }
}  // namespace vkc
