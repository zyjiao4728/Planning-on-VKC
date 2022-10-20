#include <math.h>
#include <ros/console.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/benchmark_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
// using namespace vkc_example;

using TesseractJointTraj = tesseract_common::JointTrajectory;

double DOOR_TARGET = 1.5;
double DRAWER_TARGET = 0.6;

std::vector<double> run(vector<TesseractJointTraj> &joint_trajs,
                        VKCEnvBasic &env, ActionSeq &actions, int n_steps,
                        int n_iter, bool rviz_enabled, unsigned int nruns,
                        bool long_horizon = false, bool use_ompl = false) {
  int window_size = 3;
  LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size, 9);
  if (long_horizon) {
    seed_generator.generate(env, actions);
  }
  ProbGenerator prob_generator;

  int j = 0;

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  std::vector<double> elapsed_time;

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      auto start = chrono::steady_clock::now();
      tesseract_planning::PlannerRequest prob_ptr;
      if (!use_ompl) {
        prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);
      } else {
        prob_ptr = prob_generator.getOmplRequest(env, action, n_steps, n_iter);
      }

      if (rviz_enabled) {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }

      // CostInfo cost = solveProb(prob_ptr, response, n_iter);
      if (!use_ompl)
        solveProb(prob_ptr, response, n_iter);
      else
        solveOmplProb(prob_ptr, response, n_iter);
      auto end = chrono::steady_clock::now();

      // break;
      if (!use_ompl) {
        if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
            response.status.value())  // optimization converges
        {
          converged = true;
          elapsed_time.emplace_back(
              chrono::duration_cast<chrono::milliseconds>(end - start).count() /
              1000.);
          break;
        } else {
          ROS_WARN(
              "[%s]optimization could not converge, response code: %d, "
              "description: %s",
              __func__, response.status.value(),
              response.status.message().c_str());
          if (long_horizon) {
            ActionSeq sub_actions(ptr, actions.end());
            seed_generator.generate(env, sub_actions);
          }
        }
      } else {
        if (OMPLMotionPlannerStatusCategory::SolutionFound ==
            response.status.value())  // optimization converges
        {
          converged = true;
          elapsed_time.emplace_back(
              chrono::duration_cast<chrono::milliseconds>(end - start).count() /
              1000.);
          break;
        } else {
          ROS_WARN(
              "[%s]optimization could not converge, response code: %d, "
              "description: %s",
              __func__, response.status.value(),
              response.status.message().c_str());
          if (long_horizon) {
            ActionSeq sub_actions(ptr, actions.end());
            seed_generator.generate(env, sub_actions);
          }
        }
      }
    }

    if (!converged) {
      elapsed_time.emplace_back(-1.0);
    }

    if (!use_ompl || converged) {
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
      if (!use_ompl)
        env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                      action);

      if (env.getPlotter() != nullptr && rviz_enabled)
        env.getPlotter()->clear();
      ++j;
    }
  }
  return elapsed_time;
}

void pullDoor(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open door **/
  // action 1: pick the door handle
  {
    actions.emplace_back(
        make_shared<PickAction>(robot, "attach_door_north_handle_link"));
    setBaseJoint(*actions.rbegin());
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", 1.3);
    place_action =
        make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    setBaseJoint(place_action);

    actions.emplace_back(place_action);
  }
}

void pushDoor(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  ROS_INFO("creating push door actions.");

  /** open door **/
  // action 1: pick the door handle
  {
    actions.emplace_back(
        make_shared<PickAction>(robot, "attach_door_north_handle_link"));
    setBaseJoint(*actions.rbegin());
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", -DOOR_TARGET);
    place_action =
        make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    setBaseJoint(place_action);

    actions.emplace_back(place_action);
  }

  ROS_INFO("push door actions created.");
}

void pullDrawer(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open drawer **/
  // action 1: pick the drawer handle
  {
    actions.emplace_back(
        make_shared<PickAction>(robot, "attach_drawer0_handle_link"));
    setBaseJoint(*actions.rbegin());
  }

  // action 2: open drawer
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer0_base_drawer_joint", -DRAWER_TARGET);
    place_action =
        make_shared<PlaceAction>(robot, "attach_drawer0_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    setBaseJoint(place_action);

    actions.emplace_back(place_action);
  }
}

void pushDrawer(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open drawer **/
  // action 1: pick the drawer handle
  {
    actions.emplace_back(
        make_shared<PickAction>(robot, "attach_drawer0_handle_link"));
    setBaseJoint(*actions.rbegin());
  }

  // action 2: open drawer
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer0_base_drawer_joint", 0.0);
    place_action =
        make_shared<PlaceAction>(robot, "attach_drawer0_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    setBaseJoint(place_action);

    actions.emplace_back(place_action);
  }
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

void vkc_ompl_reach(vkc::ActionSeq &actions, Eigen::VectorXd vkc_pose) {
  // action 1: move vkc to target
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    // link_objectives.emplace_back("robotiq_arg2f_base_link", ee_pose);

    auto action =
        make_shared<GotoAction>("vkc", link_objectives, joint_objectives);
    action->joint_candidate = vkc_pose;

    actions.emplace_back(action);
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

void sampleInitBasePose(vkc::VKCEnvBasic &env, int envid) {
  bool init_base_position = false;
  vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
  Eigen::VectorXd base_values = Eigen::Vector2d(0., 0.);
  tesseract_collision::ContactResultMap contact_results;

  while (!init_base_position) {
    init_base_position = true;
    contact_results.clear();
    if (envid == 1) {
      base_values = Eigen::Vector2d(rand() % 100 / 99. * -2. - 0.5,
                                    6. * (rand() % 100 / 99. - 0.5));
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    } else if (envid == 2) {
      base_values =
          Eigen::Vector2d(1. * (rand() % 100 / 99.), rand() % 100 / 99. * 5.);
      env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    }

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
                                  unsigned int nruns, int envid) {
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  if (envid == 1) {
    target_joint = "door_north_door_joint";
    target_value = DOOR_TARGET;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = DRAWER_TARGET;
    attach_location_ptr = env.getAttachLocation("attach_drawer0_handle_link");
  } else {
    ROS_ERROR("Run baseline 1: Unknown environment id.");
  }
  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          env.getVKCEnv()
              ->getTesseract()
              ->getKinematicGroup("vkc")
              ->getJointNames());

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  // baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);

  // auto elapsed_time =
  //     run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
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
    elapsed_time = run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled,
                       1, false, false);
    if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
  }
  std::vector<double> data;

  tesseract_common::TrajArray arm_init =
      joint_trajs[1].states.front().position.head(6);
  tesseract_common::TrajArray arm_goal =
      joint_trajs[1].states.back().position.head(6);

  interpBaselineReach(data, elapsed_time, joint_trajs);

  env.getVKCEnv()->getTesseract()->setState(joint_target);

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

  while (try_cnt++ < nruns) {
    actions.clear();
    base_time.clear();
    joint_trajs.clear();
    success = true;
    if (envid == 2) {
      env.getVKCEnv()->getTesseract()->setState(joint_init);
    }
    moveBase(actions, sampleBasePose(env, pose_place));

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
                                  unsigned int nruns, int envid) {
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  if (envid == 1) {
    target_joint = "door_north_door_joint";
    target_value = DOOR_TARGET;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = DRAWER_TARGET;
    attach_location_ptr = env.getAttachLocation("attach_drawer0_handle_link");
  } else {
    ROS_ERROR("Run baseline 2: Unknown environment id.");
  }

  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          env.getVKCEnv()
              ->getTesseract()
              ->getKinematicGroup("vkc")
              ->getJointNames());

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  // baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);

  // auto elapsed_time =
  //     run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);

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
    elapsed_time = run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled,
                       1, false, false);
    if (elapsed_time[0] > 0. && elapsed_time[1] > 0.) break;
  }

  std::vector<double> data;

  tesseract_common::TrajArray arm_init =
      joint_trajs[1].states.front().position.head(6);
  tesseract_common::TrajArray arm_goal =
      joint_trajs[1].states.back().position.head(6);

  interpBaselineReach(data, elapsed_time, joint_trajs);
  env.getVKCEnv()->getTesseract()->setState(joint_target);

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

  while (try_cnt++ < nruns) {
    actions.clear();
    base_time.clear();
    joint_trajs.clear();
    success = true;
    if (envid == 2) {
      env.getVKCEnv()->getTesseract()->setState(joint_init);
    }
    auto base_pose = sampleBasePose(env, pose_place);
    moveBase(actions, base_pose);

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

std::vector<double> run_ompl(vector<TesseractJointTraj> &joint_trajs,
                             vkc::VKCEnvBasic &env, vkc::ActionSeq &actions,
                             int n_steps, int n_iter, bool rviz_enabled,
                             unsigned int nruns, int envid) {
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  if (envid == 1) {
    target_joint = "door_north_door_joint";
    target_value = DOOR_TARGET;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = DRAWER_TARGET;
    attach_location_ptr = env.getAttachLocation("attach_drawer0_handle_link");
  } else {
    ROS_ERROR("Run ompl: Unknown environment id.");
  }

  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          env.getVKCEnv()
              ->getTesseract()
              ->getKinematicGroup("vkc")
              ->getJointNames());

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  // baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);
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
    auto vkc_pose = sampleBasePose(env, pose_close);
    vkc_ompl_reach(actions, vkc_pose);
    elapsed_time = run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled,
                       1, false, true);
    if (elapsed_time[0] > 0.) break;
  }

  std::vector<double> data;

  interpVKCData(data, elapsed_time, joint_trajs);

  if (elapsed_time[0] < 0.) {
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(-1);
    data.emplace_back(0);
    return data;
  }

  env.updateEnv(joint_trajs.back().back().joint_names,
                joint_trajs.back().back().position, actions.back());

  tesseract_common::TrajArray arm_init =
      joint_trajs[0].states.front().position.segment(3, 6);
  tesseract_common::TrajArray arm_goal =
      joint_trajs[0].states.back().position.segment(3, 6);

  env.getVKCEnv()->getTesseract()->setState(joint_target);
  std::cout << "After set state" << std::endl;

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

  while (try_cnt++ < nruns) {
    actions.clear();
    base_time.clear();
    joint_trajs.clear();
    success = true;
    if (envid == 2) {
      env.getVKCEnv()->getTesseract()->setState(joint_init);
    }
    moveBase(actions, sampleBasePose(env, pose_place));

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
  srand(time(NULL));

  setupLog(console_bridge::CONSOLE_BRIDGE_LOG_WARN);

  ros::init(argc, argv, "open_door_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  bool long_horizon = false;
  bool ompl = false;
  int steps = 30;
  int n_iter = 100;
  int nruns = 5;
  int envid = 1;
  int runbs = 0;
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);
  pnh.param<int>("envid", envid, envid);
  pnh.param<int>("baseline", runbs, runbs);
  pnh.param<bool>("longhorizon", long_horizon, long_horizon);
  pnh.param<bool>("ompl", ompl, ompl);

  BenchmarkEnv env(nh, plotting, rviz, steps, envid, runbs, ompl);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  sampleInitBasePose(env, envid);

  vector<TesseractJointTraj> joint_trajs;

  ActionSeq actions;

  if (envid == 1 && !runbs && !ompl) {
    pushDoor(actions, robot);
    auto elapsed_time = run(joint_trajs, env, actions, steps, n_iter, rviz,
                            nruns, long_horizon);

    std::vector<double> data;

    interpVKCData(data, elapsed_time, joint_trajs);
    if (long_horizon) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_door_push_vkc_long.csv");
    } else {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_door_push_vkc.csv");
    }

  } else if (envid == 2 && !runbs && !ompl) {
    pullDrawer(actions, robot);
    auto elapsed_time = run(joint_trajs, env, actions, steps, n_iter, rviz,
                            nruns, long_horizon);

    std::vector<double> data;

    interpVKCData(data, elapsed_time, joint_trajs);
    if (long_horizon) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_drawer_pull_vkc_long.csv");
    } else {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_drawer_pull_vkc.csv");
    }

  } else if (runbs == 1) {
    auto data = run_baseline1(joint_trajs, env, actions, steps, n_iter, rviz,
                              nruns, envid);
    if (envid == 2) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_drawer_pull_bl1.csv");
    } else if (envid == 1) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_door_push_bl1.csv");
    }
  } else if (runbs == 2) {
    auto data = run_baseline2(joint_trajs, env, actions, steps, n_iter, rviz,
                              nruns, envid);
    if (envid == 2) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_drawer_pull_bl2.csv");
    } else if (envid == 1) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_door_push_bl2.csv");
    }
  } else if (ompl) {
    auto data =
        run_ompl(joint_trajs, env, actions, steps, n_iter, rviz, nruns, envid);
    if (envid == 2) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_drawer_pull_ompl.csv");
    } else if (envid == 1) {
      saveDataToFile(data,
                     "/home/jiao/Dropbox/UCLA/Research/2022-TRO-VKC/exp/exp1/"
                     "open_door_push_ompl.csv");
    }
  }
  //   pullDoor(actions, robot);
  // pushDrawer(actions, robot);
}