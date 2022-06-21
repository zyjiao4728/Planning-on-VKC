#include <math.h>
#include <ros/console.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>
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

int saveDataToFile(const std::vector<double> &data,
                   const std::string filename) {
  ofstream fileout;
  fileout.open(filename, std::ios_base::app);

  if (!fileout.is_open()) {
    std::cout << "Cannot open file: " << filename << std::endl;
    return -1;
  }

  for (auto item : data) {
    fileout << item << ",";
  }
  fileout << "\n";
  fileout.close();
  std::cout << "Trajectory file save at: " << filename << std::endl;

  return 0;
}

std::vector<double> run(vector<TesseractJointTraj> &joint_trajs,
                        VKCEnvBasic &env, ActionSeq &actions, int n_steps,
                        int n_iter, bool rviz_enabled, unsigned int nruns) {
  // int window_size = 3;
  // LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size);
  // seed_generator.generate(env, actions);
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
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      if (rviz_enabled) {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }

      // CostInfo cost = solveProb(prob_ptr, response, n_iter);
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
        break;
      } else {
        ROS_WARN(
            "[%s]optimization could not converge, response code: %d, "
            "description: %s",
            __func__, response.status.value(),
            response.status.message().c_str());
        // ActionSeq sub_actions(ptr, actions.end());
        // seed_generator.generate(env, sub_actions);
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

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);

    if (env.getPlotter() != nullptr && rviz_enabled) env.getPlotter()->clear();
    ++j;
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
    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
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

    place_action->setBaseJoint(std::make_pair(
        std::string("base_y_base_x"), std::string("base_theta_base_y")));

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
    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", -1.5);
    place_action =
        make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    place_action->setBaseJoint(std::make_pair(
        std::string("base_y_base_x"), std::string("base_theta_base_y")));

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
    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
  }

  // action 2: open drawer
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer0_base_drawer_joint", -0.6);
    place_action =
        make_shared<PlaceAction>(robot, "attach_drawer0_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    place_action->setBaseJoint(std::make_pair(
        std::string("base_y_base_x"), std::string("base_theta_base_y")));

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
    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
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

    place_action->setBaseJoint(std::make_pair(
        std::string("base_y_base_x"), std::string("base_theta_base_y")));

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

    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
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

    (*actions.rbegin())
        ->setBaseJoint(std::make_pair(std::string("base_y_base_x"),
                                      std::string("base_theta_base_y")));
  }
}

bool sampleArmPose1(vkc::VKCEnvBasic &env, Eigen::Isometry3d ee_goal,
                    Eigen::VectorXd &ik_result) {
  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("arm"));
  tesseract_kinematics::KinGroupIKInputs ik_inputs;

  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_goal, "world", "robotiq_arg2f_base_link"));

  tesseract_collision::ContactResultMap contact_results;
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd ik_seed =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());

  ik_result = ik_seed;
  tesseract_kinematics::IKSolutions result;
  for (int tries = 0; tries < 200; tries++) {
    result = kin->calcInvKin(ik_inputs, ik_seed);
    if (result.size() > 0) break;
  }

  double max_cost = 1e6;
  bool in_collision = true;
  for (const auto &res : result) {
    if ((ik_seed - res).array().abs().sum() < max_cost) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() > 0) {
        in_collision = true;
      } else {
        ik_result = res;
        max_cost = (ik_seed - res).array().abs().sum();
        in_collision = false;
        std::cout << "found ik, collision: " << in_collision
                  << ", cost: " << max_cost << std::endl;
      }
    }
  }
  if (in_collision) {
    std::cout << "ik not found." << std::endl;
  }
  contact_results.clear();
  return !in_collision;
}

bool sampleArmPose2(
    vkc::VKCEnvBasic &env, std::string target_joint, double target_value,
    Eigen::VectorXd &ik_result,
    vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr,
    int remaining_steps) {
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          std::vector<std::string>({target_joint}))[0];

  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("arm"));

  if (joint_init[target_joint] > target_value) {
    ik_result = env.getVKCEnv()->getTesseract()->getCurrentJointValues(
        kin->getJointNames());
    return true;
  }

  int max_inc = 100;
  double joint_res =
      (target_value - joint_init[target_joint]) / remaining_steps;
  double joint_fineres =
      (target_value - joint_init[target_joint]) / remaining_steps / 10;
  int n_inc = 0;

  bool no_collision = false;
  bool found_ik = false;

  joint_init[target_joint] += joint_res;

  env.getVKCEnv()->getTesseract()->setState(joint_init);

  tesseract_kinematics::IKSolutions result;

  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  Eigen::Isometry3d ee_target =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;
  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_target, "world", "robotiq_arg2f_base_link"));
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd ik_seed =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());
  ik_result = ik_seed;

  for (int tries = 0; tries < 200; tries++) {
    result = kin->calcInvKin(ik_inputs, ik_seed);
    if (result.size() > 0) {
      found_ik = true;
      ik_result = result[0];
      break;
    }
  }

  tesseract_collision::ContactResultMap contact_results;
  double max_cost = 1e6;
  for (const auto &res : result) {
    if ((ik_seed - res).array().abs().sum() < max_cost) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() > 0) {
        for (auto contact_result : contact_results) {
          std::cout << contact_result.first.first << ":\t"
                    << contact_result.first.second << std::endl;
        }
        no_collision = false;
      } else {
        max_cost = (ik_seed - res).array().abs().sum();
        ik_result = res;
        no_collision = true;
      }
    }
  }

  if (no_collision) {
    std::cout << std::boolalpha;
    std::cout << "found ik " << found_ik << " in collision: " << !no_collision
              << ", cost: " << max_cost
              << ", joint_init: " << joint_init[target_joint] << std::endl;
    return (found_ik && no_collision);
  }

  auto temp_joint_init = joint_init;

  while (!no_collision && n_inc < max_inc) {
    n_inc++;
    ik_inputs.clear();

    if (found_ik) {
      temp_joint_init[target_joint] += joint_fineres;
      temp_joint_init[target_joint] =
          std::min(target_value, temp_joint_init[target_joint]);
      env.getVKCEnv()->getTesseract()->setState(temp_joint_init);
    } else {
      temp_joint_init[target_joint] -= joint_fineres;
      temp_joint_init[target_joint] =
          std::max(0.0, temp_joint_init[target_joint]);
      env.getVKCEnv()->getTesseract()->setState(temp_joint_init);
    }

    Eigen::Isometry3d ee_target =
        env.getVKCEnv()->getTesseract()->getLinkTransform(
            attach_location_ptr->link_name_) *
        attach_location_ptr->local_joint_origin_transform;
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
        ee_target, "world", "robotiq_arg2f_base_link"));

    for (int tries = 0; tries < 200; tries++) {
      result = kin->calcInvKin(ik_inputs, ik_seed);
      if (result.size() > 0) {
        ik_result = result[0];
        found_ik = true;
        break;
      } else {
        found_ik = false;
      }
    }
    max_cost = 1e6;
    for (const auto &res : result) {
      if ((ik_seed - res).array().abs().sum() < max_cost) {
        env.getVKCEnv()->getTesseract()->setState(joint_names, res);
        env.getVKCEnv()
            ->getTesseract()
            ->getDiscreteContactManager()
            ->contactTest(contact_results,
                          tesseract_collision::ContactTestType::ALL);
        if (contact_results.size() > 0) {
          for (auto contact_result : contact_results) {
            std::cout << contact_result.first.first << ":\t"
                      << contact_result.first.second << std::endl;
          }
          no_collision = false;
        } else {
          max_cost = (ik_seed - res).array().abs().sum();
          ik_result = res;
          no_collision = true;
        }
      }
      contact_results.clear();
    }
    if (found_ik) {
      std::cout << std::boolalpha;
      std::cout << "found ik " << found_ik << " in collision: " << !no_collision
                << ", cost: " << max_cost
                << ", temp_joint_init: " << temp_joint_init[target_joint]
                << std::endl;
    }
  }
  return (found_ik && no_collision);
}

Eigen::VectorXd sampleBasePose(vkc::VKCEnvBasic &env,
                               Eigen::Isometry3d ee_goal) {
  tesseract_kinematics::KinematicGroup::Ptr kin =
      std::move(env.getVKCEnv()->getTesseract()->getKinematicGroup("vkc"));
  tesseract_kinematics::KinGroupIKInputs ik_inputs;

  ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(
      ee_goal, "world", "robotiq_arg2f_base_link"));

  tesseract_common::KinematicLimits limits = kin->getLimits();
  tesseract_collision::ContactResultMap contact_results;
  tesseract_srdf::JointGroup joint_names = kin->getJointNames();
  Eigen::VectorXd initial_joint_values =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues(
          kin->getJointNames());

  double max_cost = 1e6;
  Eigen::VectorXd ik_result;
  int sample_base_pose_tries = 0;
  while (sample_base_pose_tries < 200) {
    sample_base_pose_tries++;
    Eigen::VectorXd ik_seed =
        tesseract_common::generateRandomNumber(limits.joint_limits);
    tesseract_kinematics::IKSolutions result =
        kin->calcInvKin(ik_inputs, ik_seed);
    for (const auto &res : result) {
      if ((ik_seed - res).array().abs().sum() < max_cost) {
        env.getVKCEnv()->getTesseract()->setState(joint_names, res);
        env.getVKCEnv()
            ->getTesseract()
            ->getDiscreteContactManager()
            ->contactTest(contact_results,
                          tesseract_collision::ContactTestType::ALL);
        if (contact_results.size() == 0) {
          ik_result = res;
          max_cost = (ik_seed - res).array().abs().sum();
        }
      }
    }
    contact_results.clear();
  }
  env.getVKCEnv()->getTesseract()->setState(joint_names, initial_joint_values);
  return ik_result;
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
    target_value = 1.5;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = 0.6;
    attach_location_ptr = env.getAttachLocation("attach_drawer0_handle_link");
  } else {
    ROS_ERROR("Run baseline 1: Unknown environment id.");
  }

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);

  auto elapsed_time =
      run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
  std::vector<double> data;

  interpBaselineReach(data, elapsed_time, joint_trajs);

  env.getVKCEnv()->getTesseract()->setState(joint_target);

  Eigen::Isometry3d pose_place =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  int try_cnt = 0;
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
    base_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
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
      auto ik_status = sampleArmPose1(env, ee_target, arm_pose);
      success = success && ik_status;
      if (!ik_status) break;
      arm_trajectory.emplace_back(arm_pose);
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("arm")
                                                    ->getJointNames(),
                                                arm_pose);
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
    return data;
  }

  data.emplace_back(
      chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. +
      base_time[0]);
  data.emplace_back(computeTrajLength(base_trajectory));
  data.emplace_back(computeTrajLength(arm_trajectory));
  data.emplace_back(success);

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
    target_value = 1.5;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = 0.6;
    attach_location_ptr = env.getAttachLocation("attach_drawer0_handle_link");
  } else {
    ROS_ERROR("Run baseline 1: Unknown environment id.");
  }

  std::unordered_map<std::string, double> joint_target;
  joint_target[target_joint] = target_value;
  std::unordered_map<std::string, double> joint_init;
  joint_init[target_joint] = 0.0;

  Eigen::Isometry3d pose_close =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  baseline_reach(actions, sampleBasePose(env, pose_close), pose_close);

  auto elapsed_time =
      run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);
  std::vector<double> data;

  interpBaselineReach(data, elapsed_time, joint_trajs);

  env.getVKCEnv()->getTesseract()->setState(joint_target);

  Eigen::Isometry3d pose_place =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  int try_cnt = 0;
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
    base_time =
        run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);

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

      auto ik_status = sampleArmPose2(env, target_joint, target_value, arm_pose,
                                      attach_location_ptr, n_steps - i);
      success = success && ik_status;
      if (!ik_status) break;
      arm_trajectory.emplace_back(arm_pose);
      env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                    ->getTesseract()
                                                    ->getKinematicGroup("arm")
                                                    ->getJointNames(),
                                                arm_pose);
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

  console_bridge::setLogLevel(
      console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  ros::init(argc, argv, "open_door_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int steps = 30;
  int n_iter = 1000;
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

  BenchmarkEnv env(nh, plotting, rviz, steps, envid, runbs);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  sampleInitBasePose(env, envid);

  vector<TesseractJointTraj> joint_trajs;

  ActionSeq actions;

  if (envid == 1 && !runbs) {
    pushDoor(actions, robot);
    auto elapsed_time =
        run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

    std::vector<double> data;

    for (int i = 0; i < elapsed_time.size(); i++) {
      data.emplace_back(elapsed_time[i]);
      std::vector<Eigen::VectorXd> base_trajectory;
      std::vector<Eigen::VectorXd> arm_trajectory;
      for (auto state : joint_trajs[i].states) {
        base_trajectory.emplace_back(state.position.head(2));
        arm_trajectory.emplace_back(state.position.segment(3, 6));
      }
      double base_cost = computeTrajLength(base_trajectory);
      double arm_cost = computeTrajLength(arm_trajectory);
      data.emplace_back(base_cost);
      data.emplace_back(arm_cost);
    }
    saveDataToFile(data,
                   "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                   "open_door_push_vkc.csv");
  } else if (envid == 2 && !runbs) {
    pullDrawer(actions, robot);
    auto elapsed_time =
        run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

    std::vector<double> data;

    for (int i = 0; i < elapsed_time.size(); i++) {
      data.emplace_back(elapsed_time[i]);
      std::vector<Eigen::VectorXd> base_trajectory;
      std::vector<Eigen::VectorXd> arm_trajectory;
      for (auto state : joint_trajs[i].states) {
        base_trajectory.emplace_back(state.position.head(2));
        arm_trajectory.emplace_back(state.position.segment(3, 6));
      }
      double base_cost = computeTrajLength(base_trajectory);
      double arm_cost = computeTrajLength(arm_trajectory);
      data.emplace_back(base_cost);
      data.emplace_back(arm_cost);
    }
    saveDataToFile(data,
                   "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                   "open_drawer_pull_vkc.csv");
  } else if (runbs == 1) {
    auto data = run_baseline1(joint_trajs, env, actions, steps, n_iter, rviz,
                              nruns, envid);
    if (envid == 2) {
      saveDataToFile(data,
                     "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                     "open_drawer_pull_bl1.csv");
    } else if (envid == 1) {
      saveDataToFile(data,
                     "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                     "open_door_pull_bl1.csv");
    }
  } else if (runbs == 2) {
    auto data = run_baseline2(joint_trajs, env, actions, steps, n_iter, rviz,
                              nruns, envid);
    if (envid == 2) {
      saveDataToFile(data,
                     "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                     "open_drawer_pull_bl2.csv");
    } else if (envid == 1) {
      saveDataToFile(data,
                     "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/benchmarking/"
                     "open_door_pull_bl2.csv");
    }
  }
  //   pullDoor(actions, robot);
  // pushDrawer(actions, robot);
}