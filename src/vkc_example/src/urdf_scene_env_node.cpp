#include <fmt/ranges.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>
// motion planning via OMPL
#include <tesseract_collision/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <iostream>
#include <string>
#include <trajopt_utils/eigen_conversions.hpp>
#include <vector>

#include "vkc/planner/prob_translator.h"

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
using TesseractJointTraj = tesseract_common::JointTrajectory;
// using namespace vkc_example;

void sampleInitBasePose(vkc::VKCEnvBasic &env) {
  bool init_base_position = false;
  vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
  Eigen::VectorXd base_values = Eigen::Vector2d(0., 0.);
  tesseract_collision::ContactResultMap contact_results;

  while (!init_base_position) {
    init_base_position = true;
    contact_results.clear();
    base_values = Eigen::Vector2d(rand() % 100 / 99. * 2. + 0.5,
                                  -rand() % 100 / 99. * 3. + 0.5);
    env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);

    env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
        contact_results, tesseract_collision::ContactTestType::ALL);

    for (auto &collision : contact_results) {
      if (collision.first.first == "base_link" ||
          collision.first.second == "base_link") {
        init_base_position = false;
        break;
      }
    }
  }
}

std::vector<double> run(vector<TesseractJointTraj> &joint_trajs,
                        VKCEnvBasic &env, ActionSeq &actions, int n_steps,
                        int n_iter, bool rviz_enabled, unsigned int nruns) {
  // int window_size = 3;
  // LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size, 9);
  ProbGenerator prob_generator;

  int j = 0;

  std::vector<double> elapsed_time;

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    // fmt::print("kg: {}", env.getVKCEnv()
    //                          ->getTesseract()
    //                          ->getKinematicGroup(action->getManipulatorID())
    //                          ->getJointNames());
    // seed_generator.generate(env, sub_actions);
    // std::cout << "sg after generate" << std::endl;
    // printSceneGraph(env.getVKCEnv()->getTesseract()->getSceneGraph());
    // action->switchCandidate();
    // fmt::print("kg: {}", env.getVKCEnv()
    //                          ->getTesseract()
    //                          ->getKinematicGroup(action->getManipulatorID())
    //                          ->getJointNames());
    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      if (rviz_enabled) {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }

      auto start = chrono::steady_clock::now();
      solveProb(prob_ptr, response, n_iter);
      auto end = chrono::steady_clock::now();

      // break;
      if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
          response.status.value())  // optimization converges
      {
        converged = true;
        elapsed_time.emplace_back(
            chrono::duration_cast<chrono::milliseconds>(end - start).count() /
            1000.);
        std::cout << elapsed_time.back() << std::endl;
        break;
      } else {
        ROS_WARN(
            "[%s]optimization could not converge, response code: %d, "
            "description: %s",
            __func__, response.status.value(),
            response.status.message().c_str());

        // action->switchCandidate();
      }
    }

    if (!converged) {
      elapsed_time.emplace_back(-1.0);
    }

    const auto &ci = response.results;

    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    tesseract_common::JointTrajectory refined_traj = trajectory;
    refineTrajectory(refined_traj, env);
    joint_trajs.emplace_back(trajectory);

    if (rviz_enabled && env.getPlotter() != nullptr) {
      ROS_INFO("plotting result");
      tesseract_common::Toolpath toolpath =
          toToolpath(ci, *env.getVKCEnv()->getTesseract());
      env.getPlotter()->plotMarker(
          tesseract_visualization::ToolpathMarker(toolpath));
      env.getPlotter()->plotTrajectory(
          refined_traj, *env.getVKCEnv()->getTesseract()->getStateSolver());
      env.getPlotter()->waitForInput(
          "Finished optimization. Press <Enter> to start next action");
    }

    // toDelimitedFile(ci,
    //                 "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/"
    //                 "trajectory/urdf_scene_env_" +
    //                     std::to_string(j) + ".csv",
    //                 ',');

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);
    CONSOLE_BRIDGE_logInform("update env finished");
    // const tesseract_srdf::KinematicsInformation kin_info =
    // env.getVKCEnv()->getTesseract()->getKinematicsInformation(); auto cmd =
    // std::make_shared<AddKinematicsInformationCommand>(kin_info);
    // env.getVKCEnv()->getTesseract()->applyCommand(cmd);

    if (env.getPlotter() != nullptr && rviz_enabled) env.getPlotter()->clear();
    ++j;
  }
  return elapsed_time;
}

void setInitState(VKCEnvBasic &env) {
  vector<string> joint_names(
      {"dishwasher_12065_joint_0", "dishwasher_12065_joint_1",
       "dishwasher_12065_joint_2",
       "cabinet_48479_joint_0"});  //, "cabinet_48479_joint_0"
  Eigen::Vector4d joint_values({-0.3080, -0.1359, -0.6457,
                                -0.12});  // open: 0.9250 close -0.6457 , -0.12
  env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
}

void baseline_reach(vkc::ActionSeq &actions, Eigen::VectorXd base_pose,
                    Eigen::Isometry3d ee_pose) {
  /** move base **/
  // action 1: move base to target
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    Eigen::Isometry3d tf;
    tf.setIdentity();
    tf.translation() += Eigen::Vector3d(base_pose[0], base_pose[1], 0.145);
    tf.linear() = Eigen::Quaterniond(1., 0., 0., 0.).matrix();

    link_objectives.emplace_back("base_link", tf);

    actions.emplace_back(
        make_shared<GotoAction>("base", link_objectives, joint_objectives));

    setBaseJoint(*actions.rbegin());
  }

  // action 2: move arm to target
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    link_objectives.emplace_back("robotiq_arg2f_base_link", ee_pose);

    actions.emplace_back(
        make_shared<GotoAction>("arm", link_objectives, joint_objectives));
  }
}

void moveBase(vkc::ActionSeq &actions, Eigen::VectorXd base_pose) {
  /** move base **/
  // action 1: move base to target
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    Eigen::Isometry3d tf;
    tf.setIdentity();
    tf.translation() += Eigen::Vector3d(base_pose[0], base_pose[1], 0.145);
    tf.linear() = Eigen::Quaterniond(1., 0., 0., 0.).matrix();

    link_objectives.emplace_back("base_link", tf);

    actions.emplace_back(
        make_shared<GotoAction>("base", link_objectives, joint_objectives));

    setBaseJoint(*actions.rbegin());
  }
}

void genTRODemoSeq(VKCEnvBasic &env, vkc::ActionSeq &actions,
                   const std::string &robot, int task_id) {
  PickAction::Ptr pick_action;
  PlaceAction::Ptr place_action;

  Eigen::VectorXd cost_coeff;
  Eigen::VectorXd pick_coeff;
  pick_coeff.setOnes(9);
  // pick_coeff[0] = 1;
  // pick_coeff[1] = 1;
  cost_coeff.setOnes(10);
  cost_coeff[9] = 0.;
  switch (task_id) {
    case 1:
      // action 1: pick fridge handle
      {
        pick_action = make_shared<PickAction>(robot, "attach_fridge_handle");
        setBaseJoint(pick_action);
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action 2: open fridge door
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint",
                                      -1.6);
        place_action = make_shared<PlaceAction>(
            robot, "attach_fridge_handle", link_objectives, joint_objectives);

        setBaseJoint(place_action);

        cost_coeff[6] = 5.;
        // cost_coeff[3] = 3.;
        // cost_coeff[0] = 3.;
        // cost_coeff[1] = 3.;
        place_action->setIKCostCoeff(cost_coeff);
        actions.emplace_back(place_action);
      }
      break;
    case 2:
      // action 1: pick door handle
      {
        pick_action = make_shared<PickAction>(robot, "attach_door_handle");
        setBaseJoint(pick_action);
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action 2: open door door
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("door_8966_joint_1", 1.5);
        place_action =
            make_shared<PlaceAction>(robot, "attach_door_handle",
                                     link_objectives, joint_objectives);

        setBaseJoint(place_action);

        cost_coeff[6] = 5.;
        // cost_coeff[4] = 5.;
        // cost_coeff[8] = 5.;
        // cost_coeff[0] = 3.;
        // cost_coeff[1] = 3.;
        place_action->setIKCostCoeff(cost_coeff);

        actions.emplace_back(place_action);
      }
      break;
    case 3:
      // action 1: pick cup
      {
        pick_action = make_shared<PickAction>(robot, "attach_cup");
        setBaseJoint(pick_action);
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action 2: place cup on the table
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        Eigen::Isometry3d destination;
        destination.setIdentity();
        destination.translation() = Eigen::Vector3d(-0.6, -1.6, 1.11);
        destination.linear() =
            Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();
        link_objectives.push_back(
            LinkDesiredPose("cup_cup_base_link", destination));

        place_action = make_shared<PlaceAction>(
            robot, "attach_cup", link_objectives, joint_objectives);

        setBaseJoint(place_action);
        actions.emplace_back(place_action);
      }
      break;
    case 4:
      // action1: pick drawer handle
      {
        auto pick_action = std::make_shared<PickAction>(robot, "attach_drawer");
        setBaseJoint(pick_action);
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action2: place drawer handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer_base_drawer1_joint", -0.22);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_drawer", link_objectives, joint_objectives);
        setBaseJoint(place_action);
        actions.emplace_back(place_action);
      }

      // Set door to open
      {
        vector<string> joint_names({"door_8966_joint_1"});
        Eigen::VectorXd joint_values;
        joint_values.setZero(1);
        joint_values[0] = -1.5;
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
    case 5:
      // action1: pick cabinet handle
      {
        auto pick_action =
            std::make_shared<PickAction>(robot, "attach_cabinet");
        setBaseJoint(pick_action);
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action2: place cabinet handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("cabinet_48479_joint_0", -0.4);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_cabinet", link_objectives, joint_objectives);
        // setBaseJoint(place_action);
        actions.emplace_back(place_action);
      }
      break;
    case 6:
      // action1: pick cabinet handle
      {
        auto pick_action =
            std::make_shared<PickAction>(robot, "attach_dishwasher");
        setBaseJoint(pick_action);
        pick_coeff.setZero(9);
        pick_coeff[0] = 1.;
        pick_coeff[1] = 1.;
        pick_action->setIKCostCoeff(pick_coeff);
        actions.emplace_back(pick_action);
      }

      // action2: place cabinet handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("dishwasher_joint_2", -0.8);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_dishwasher", link_objectives, joint_objectives);
        // cost_coeff[6] = 5.;
        // cost_coeff[3] = 3.;
        // cost_coeff[0] = 3.;
        // cost_coeff[1] = 3.;
        // place_action->setIKCostCoeff(cost_coeff);
        setBaseJoint(place_action);
        actions.emplace_back(place_action);
      }
      break;
  }
}

void genTRODemoEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                               UrdfSceneEnv::InverseChainsInfos &inverse_chains,
                               int task_id, int baseline = 0) {
  switch (task_id) {
    case 1:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                         "fridge_0001_dof_rootd_Aa002_r",
                                         "fridge_0001",
                                         {0.65, -0.25, -0.65},
                                         {0.5, -0.5, 0.5, 0.5},
                                         true});
      if (baseline == 0)
        inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
            "fridge_0001", "fridge_0001_dof_rootd_Aa002_r"});
      break;

    case 2:
      attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_door_handle",
                                                           "door_8966_link_2",
                                                           "door_8966_base",
                                                           {0.0, 0.0, 0.25},
                                                           {0, 0, 1, 0},
                                                           true});

      if (baseline == 0)
        inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
            "door_8966_base", "door_8966_link_2"});
      break;

    case 3:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_cup",
                                         "cup_cup_link",
                                         "cup_cup_base_link",
                                         {0.04, 0.05, -0.12},
                                         {0.707106781, 0, 0, 0.707106781},
                                         false});
      if (baseline == 0)
        inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
            "cup_cup_base_link", "cup_cup_link"});
      break;

    case 4:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_drawer",
                                         "drawer_handle1",
                                         "drawer_base_link",
                                         {0.16, 0.000, 0.00},
                                         {0.5000, -0.5000, -0.5000, 0.5000},
                                         true});
      if (baseline == 0)
        inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
            "drawer_base_link", "drawer_handle1"});
      break;

    case 5:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
                                         "cabinet_48479_handle0",
                                         "cabinet_48479_base_link",
                                         {0.0, 0.0, 0.15},
                                         {0, -0.707106781, -0.707106781, 0},
                                         true});
      if (baseline == 0)
        inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
            "cabinet_48479_base_link", "cabinet_48479_handle0"});
      break;

    case 6:
      attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
          "attach_dishwasher",
          "dishwasher_link_2",
          "dishwasher",
          {0.45, 0.55, 0.70},
          {-0.382683432365090, 0.923879532511287, 0.0, 0.0},
          true});
      if (baseline == 0)
        inverse_chains.emplace_back(
            UrdfSceneEnv::InverseChainsInfo{"dishwasher", "dishwasher_link_2"});
      break;

    default:
      break;
  }

  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

void interpBaselineReach(std::vector<double> &data,
                         std::vector<double> &elapsed_time,
                         std::vector<TesseractJointTraj> &joint_trajs) {
  if (elapsed_time[0] < 0. || elapsed_time[1] < 0.) {
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(0);
    return;
  } else {
    data.emplace_back(elapsed_time[0] + elapsed_time[1]);
    std::vector<Eigen::VectorXd> base_trajectory;
    std::vector<Eigen::VectorXd> arm_trajectory;
    for (auto state : joint_trajs[0].states) {
      base_trajectory.emplace_back(state.position.head(2));
    }
    for (auto state : joint_trajs[1].states) {
      arm_trajectory.emplace_back(state.position.head(6));
    }
    data.emplace_back(computeTrajLength(base_trajectory));
    data.emplace_back(computeTrajLength(arm_trajectory));
    data.emplace_back(1);
    return;
  }
  return;
}

std::vector<double> run_baseline1(vector<TesseractJointTraj> &joint_trajs,
                                  vkc::VKCEnvBasic &env,
                                  vkc::ActionSeq &actions, int n_steps,
                                  int n_iter, bool rviz_enabled,
                                  unsigned int nruns, int taskid) {
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  Eigen::Isometry3d task3_goal;
  task3_goal.setIdentity();
  task3_goal.translation() = Eigen::Vector3d(-0.6, -1.6, 1.11);
  task3_goal.linear() =
      Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();
  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          env.getVKCEnv()
              ->getTesseract()
              ->getKinematicGroup("vkc")
              ->getJointNames());

  switch (taskid) {
    case 1:
      target_joint = "fridge_0001_dof_rootd_Aa002_r_joint";
      target_value = 1.6;
      attach_location_ptr = env.getAttachLocation("attach_fridge_handle");
      break;
    case 2:
      target_joint = "door_8966_joint_1";
      target_value = -1.5;
      attach_location_ptr = env.getAttachLocation("attach_door_handle");
      break;
    case 3:
      attach_location_ptr = env.getAttachLocation("attach_cup");
      break;
    case 4:
      target_joint = "drawer_base_drawer1_joint";
      target_value = 0.22;
      attach_location_ptr = env.getAttachLocation("attach_drawer");
      {
        vector<string> joint_names({"door_8966_joint_1"});
        Eigen::VectorXd joint_values;
        joint_values.setZero(1);
        joint_values[0] = -1.5;
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
    case 5:
      target_joint = "cabinet_48479_joint_0";
      target_value = 0.4;
      attach_location_ptr = env.getAttachLocation("attach_cabinet");
      break;
    case 6:
      target_joint = "dishwasher_joint_2";
      target_value = 0.8;
      attach_location_ptr = env.getAttachLocation("attach_dishwasher");
      break;

    default:
      ROS_ERROR("Run baseline 1: Unknown task id.");
      break;
  }

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  int try_cnt = 0;
  std::vector<double> elapsed_time;
  while (try_cnt++ < nruns) {
    elapsed_time.clear();
    joint_trajs.clear();
    actions.clear();
    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("vkc")
                                                  ->getJointNames(),
                                              initial_joint_values);
    baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);
    elapsed_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, 1);
    if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
  }
  std::vector<double> data;

  tesseract_common::TrajArray arm_init =
      joint_trajs[1].states.front().position.head(6);
  tesseract_common::TrajArray arm_goal =
      joint_trajs[1].states.back().position.head(6);

  interpBaselineReach(data, elapsed_time, joint_trajs);

  if (taskid == 3) {
    tesseract_scene_graph::Joint new_joint("world_cup_joint");
    new_joint.parent_link_name = "world";
    new_joint.child_link_name = "cup_cup_base_link";
    new_joint.type = tesseract_scene_graph::JointType::FIXED;
    new_joint.parent_to_joint_origin_transform = task3_goal;
    auto move_link_cmd =
        std::make_shared<tesseract_environment::MoveLinkCommand>(new_joint);
    env.getVKCEnv()->getTesseract()->applyCommand(move_link_cmd);
  } else {
    env.getVKCEnv()->getTesseract()->setState(joint_target);
  }

  Eigen::Isometry3d pose_place =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  try_cnt = 0;
  auto start = chrono::steady_clock::now();
  auto end = chrono::steady_clock::now();
  bool success = true;
  std::vector<double> base_time;
  std::vector<Eigen::VectorXd> base_trajectory;
  std::vector<Eigen::VectorXd> arm_trajectory;

  if (taskid == 3) {
    initial_joint_values =
        env.getVKCEnv()->getTesseract()->getCurrentJointValues(
            env.getVKCEnv()
                ->getTesseract()
                ->getKinematicGroup("vkc")
                ->getJointNames());
    while (try_cnt++ < nruns) {
      elapsed_time.clear();
      joint_trajs.clear();
      actions.clear();
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("vkc")
                                                    ->getJointNames(),
                                                initial_joint_values);
      baseline_reach(actions, sampleBasePose(env, pose_place), pose_place);
      elapsed_time =
          run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, 1);
      if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
    }

    interpBaselineReach(data, elapsed_time, joint_trajs);

    return data;
  }

  while (try_cnt++ < nruns) {
    actions.clear();
    base_time.clear();
    joint_trajs.clear();
    success = true;
    auto base_place_pose = sampleBasePose(env, pose_place);
    if (taskid != 2) {
      env.getVKCEnv()->getTesseract()->setState(joint_init);
    }
    moveBase(actions, base_place_pose);

    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("arm")
                                                  ->getJointNames(),
                                              arm_init);
    base_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("arm")
                                                  ->getJointNames(),
                                              arm_goal);
    if (base_time[0] < 0.) continue;
    base_trajectory.clear();
    arm_trajectory.clear();
    for (auto state : joint_trajs[0].states) {
      base_trajectory.emplace_back(state.position.head(2));
    }
    tesseract_common::JointTrajectory base_traj = joint_trajs.back();
    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    Eigen::VectorXd base_values = Eigen::Vector2d(0, 0);

    env.getVKCEnv()->getTesseract()->setState(joint_init);
    Eigen::VectorXd arm_pose;
    start = chrono::steady_clock::now();
    for (int i = 0; i < n_steps; i++) {
      joint_init[target_joint] += joint_target[target_joint] / n_steps;
      env.getVKCEnv()->getTesseract()->setState(joint_init);
      base_values[0] = base_traj.states[i + 1].position[0];
      base_values[1] = base_traj.states[i + 1].position[1];
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
      Eigen::Isometry3d ee_target =
          env.getVKCEnv()->getTesseract()->getLinkTransform(
              attach_location_ptr->link_name_) *
          attach_location_ptr->local_joint_origin_transform;
      auto current_value =
          env.getVKCEnv()->getTesseract()->getCurrentJointValues(
              std::vector<std::string>({target_joint}))[0];
      auto ik_status = sampleArmPose1(env, ee_target, arm_pose);
      success = success &&
                (ik_status || (std::abs(current_value - target_value) < 0.1));
      if (!success) break;
      arm_trajectory.emplace_back(arm_pose);
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("arm")
                                                    ->getJointNames(),
                                                arm_pose);
      if (std::abs(current_value - target_value) < 0.1) break;
      if (rviz_enabled)
        env.getPlotter()->waitForInput("press Enter key to go on...");
    }
    end = chrono::steady_clock::now();
    if (success) {
      break;
    } else {
      joint_init[target_joint] = 0.0;
      env.getVKCEnv()->getTesseract()->setState(joint_target);
      base_values[0] = base_traj.states[0].position[0];
      base_values[1] = base_traj.states[0].position[1];
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    }
  }

  if (base_time[0] < 0. || !success) {
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(0);
    return data;
  }

  data.emplace_back(
      chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. +
      base_time[0]);
  data.emplace_back(computeTrajLength(base_trajectory));
  data.emplace_back(computeTrajLength(arm_trajectory));

  auto current_value = env.getVKCEnv()->getTesseract()->getCurrentJointValues(
      std::vector<std::string>({target_joint}))[0];

  data.emplace_back(success && (std::abs(current_value - target_value) < 0.1));

  return data;
}

std::vector<double> run_baseline2(vector<TesseractJointTraj> &joint_trajs,
                                  vkc::VKCEnvBasic &env,
                                  vkc::ActionSeq &actions, int n_steps,
                                  int n_iter, bool rviz_enabled,
                                  unsigned int nruns, int taskid) {
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  Eigen::Isometry3d task3_goal;
  task3_goal.setIdentity();
  task3_goal.translation() = Eigen::Vector3d(-0.6, -1.6, 1.11);
  task3_goal.linear() =
      Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();
  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          env.getVKCEnv()
              ->getTesseract()
              ->getKinematicGroup("vkc")
              ->getJointNames());

  switch (taskid) {
    case 1:
      target_joint = "fridge_0001_dof_rootd_Aa002_r_joint";
      target_value = 1.6;
      attach_location_ptr = env.getAttachLocation("attach_fridge_handle");
      break;
    case 2:
      target_joint = "door_8966_joint_1";
      target_value = -1.5;
      attach_location_ptr = env.getAttachLocation("attach_door_handle");
      break;
    case 3:
      attach_location_ptr = env.getAttachLocation("attach_cup");
      break;
    case 4:
      target_joint = "drawer_base_drawer1_joint";
      target_value = 0.22;
      attach_location_ptr = env.getAttachLocation("attach_drawer");
      {
        vector<string> joint_names({"door_8966_joint_1"});
        Eigen::VectorXd joint_values;
        joint_values.setZero(1);
        joint_values[0] = -1.5;
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
    case 5:
      target_joint = "cabinet_48479_joint_0";
      target_value = 0.4;
      attach_location_ptr = env.getAttachLocation("attach_cabinet");
      break;
    case 6:
      target_joint = "dishwasher_joint_2";
      target_value = 0.8;
      attach_location_ptr = env.getAttachLocation("attach_dishwasher");
      break;

    default:
      ROS_ERROR("Run baseline 1: Unknown task id.");
      break;
  }
  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  int try_cnt = 0;
  std::vector<double> elapsed_time;
  while (try_cnt++ < nruns) {
    elapsed_time.clear();
    joint_trajs.clear();
    actions.clear();
    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("vkc")
                                                  ->getJointNames(),
                                              initial_joint_values);
    baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);
    elapsed_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, 1);
    if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
  }
  std::vector<double> data;

  tesseract_common::TrajArray arm_init =
      joint_trajs[1].states.front().position.head(6);
  tesseract_common::TrajArray arm_goal =
      joint_trajs[1].states.back().position.head(6);

  interpBaselineReach(data, elapsed_time, joint_trajs);

  if (taskid == 3) {
    tesseract_scene_graph::Joint new_joint("world_cup_joint");
    new_joint.parent_link_name = "world";
    new_joint.child_link_name = "cup_cup_base_link";
    new_joint.type = tesseract_scene_graph::JointType::FIXED;
    new_joint.parent_to_joint_origin_transform = task3_goal;
    auto move_link_cmd =
        std::make_shared<tesseract_environment::MoveLinkCommand>(new_joint);
    env.getVKCEnv()->getTesseract()->applyCommand(move_link_cmd);
  } else {
    env.getVKCEnv()->getTesseract()->setState(joint_target);
  }

  Eigen::Isometry3d pose_place =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  try_cnt = 0;
  auto start = chrono::steady_clock::now();
  auto end = chrono::steady_clock::now();
  bool success = true;
  std::vector<double> base_time;
  std::vector<Eigen::VectorXd> base_trajectory;
  std::vector<Eigen::VectorXd> arm_trajectory;

  if (taskid == 3) {
    initial_joint_values =
        env.getVKCEnv()->getTesseract()->getCurrentJointValues(
            env.getVKCEnv()
                ->getTesseract()
                ->getKinematicGroup("vkc")
                ->getJointNames());
    while (try_cnt++ < nruns) {
      elapsed_time.clear();
      joint_trajs.clear();
      actions.clear();
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("vkc")
                                                    ->getJointNames(),
                                                initial_joint_values);
      baseline_reach(actions, sampleBasePose(env, pose_place), pose_place);
      elapsed_time =
          run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, 1);
      if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
    }

    interpBaselineReach(data, elapsed_time, joint_trajs);

    return data;
  }

  while (try_cnt++ < nruns) {
    actions.clear();
    base_time.clear();
    joint_trajs.clear();
    success = true;
    auto base_place_pose = sampleBasePose(env, pose_place);
    if (taskid != 2) {
      env.getVKCEnv()->getTesseract()->setState(joint_init);
    }
    moveBase(actions, base_place_pose);

    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("arm")
                                                  ->getJointNames(),
                                              arm_init);
    base_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("arm")
                                                  ->getJointNames(),
                                              arm_goal);
    if (base_time[0] < 0.) continue;
    base_trajectory.clear();
    arm_trajectory.clear();
    for (auto state : joint_trajs[0].states) {
      base_trajectory.emplace_back(state.position.head(2));
    }

    tesseract_common::JointTrajectory base_traj = joint_trajs.back();
    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    Eigen::VectorXd base_values = Eigen::Vector2d(0, 0);

    env.getVKCEnv()->getTesseract()->setState(joint_init);

    Eigen::VectorXd arm_pose;
    start = chrono::steady_clock::now();
    for (int i = 0; i < n_steps; i++) {
      base_values[0] = base_traj.states[i + 1].position[0];
      base_values[1] = base_traj.states[i + 1].position[1];
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
      auto current_value =
          env.getVKCEnv()->getTesseract()->getCurrentJointValues(
              std::vector<std::string>({target_joint}))[0];
      auto ik_status = sampleArmPose2(env, target_joint, target_value, arm_pose,
                                      attach_location_ptr, n_steps - i);
      success = success &&
                (ik_status || (std::abs(current_value - target_value) < 0.1));
      if (!success) break;
      arm_trajectory.emplace_back(arm_pose);
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("arm")
                                                    ->getJointNames(),
                                                arm_pose);
      if (std::abs(current_value - target_value) < 0.1) break;
      if (rviz_enabled)
        env.getPlotter()->waitForInput("press Enter key to go on...");
    }
    end = chrono::steady_clock::now();
    if (success) {
      break;
    } else {
      joint_init[target_joint] = 0.0;
      env.getVKCEnv()->getTesseract()->setState(joint_target);
      base_values[0] = base_traj.states[0].position[0];
      base_values[1] = base_traj.states[0].position[1];
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    }
  }
  if (base_time[0] < 0. || !success) {
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(0);
    return data;
  }
  data.emplace_back(
      chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. +
      base_time[0]);
  data.emplace_back(computeTrajLength(base_trajectory));
  data.emplace_back(computeTrajLength(arm_trajectory));
  auto current_value = env.getVKCEnv()->getTesseract()->getCurrentJointValues(
      std::vector<std::string>({target_joint}))[0];

  data.emplace_back(success && (std::abs(current_value - target_value) < 0.1));

  return data;
}

int main(int argc, char **argv) {
  srand((unsigned)time(
      NULL));  // for generating waypoint randomly motion planning
  ros::init(argc, argv, "urdf_scene_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  setupLog(console_bridge::CONSOLE_BRIDGE_LOG_WARN);
  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int baseline = 0;
  int steps = 10;
  int n_iter = 1000;
  int nruns = 5;
  int taskid = 1;
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);
  pnh.param<int>("taskid", taskid, taskid);
  pnh.param<int>("baseline", baseline, baseline);

  UrdfSceneEnv::AttachObjectInfos attaches;
  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  genTRODemoEnvironmentInfo(attaches, inverse_chains, taskid, baseline);

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  sampleInitBasePose(env);
  vector<TesseractJointTraj> joint_trajs;
  ActionSeq actions;
  std::vector<double> data;

  if (baseline == 0) {
    genTRODemoSeq(env, actions, robot, taskid);
    auto elapsed_time =
        run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
    interpVKCData(data, elapsed_time, joint_trajs);
    saveDataToFile(data,
                   "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp2/"
                   "household_env_vkc_" +
                       std::to_string(taskid) + ".csv");
  } else if (baseline == 1) {
    data = run_baseline1(joint_trajs, env, actions, steps, n_iter, rviz, nruns,
                         taskid);
    saveDataToFile(data,
                   "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp2/"
                   "household_env_bs1_" +
                       std::to_string(taskid) + ".csv");
  } else if (baseline == 2) {
    data = run_baseline2(joint_trajs, env, actions, steps, n_iter, rviz, nruns,
                         taskid);
    saveDataToFile(data,
                   "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp2/"
                   "household_env_bs2_" +
                       std::to_string(taskid) + ".csv");
  }
}
