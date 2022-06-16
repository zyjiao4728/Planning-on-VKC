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

void run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env,
         ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled,
         unsigned int nruns) {
  int window_size = 3;
  // LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size);
  // seed_generator.generate(env, actions);
  ProbGenerator prob_generator;

  int j = 0;

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      env.getPlotter()->waitForInput(
          "optimization is ready. Press <Enter> to process the request.");

      // CostInfo cost = solveProb(prob_ptr, response, n_iter);
      solveProb(prob_ptr, response, n_iter);

      // break;
      if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
          response.status.value())  // optimization converges
      {
        converged = true;
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

    const auto &ci = response.results;

    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    tesseract_common::JointTrajectory refined_traj = trajectory;
    refineTrajectory(refined_traj, env);
    joint_trajs.emplace_back(trajectory);

    // ROS_WARN("trajectory: ");
    // for (auto jo : trajectory) {
    //   std::cout << jo.position << std::endl;
    // }

    // refine the orientation of the move base

    // tesseract_common::TrajArray trajectory =
    //     response.trajectory.leftCols(response.joint_names.size());
    // refineTrajectory(trajectory);

    // std::cout << "optimized trajectory: " << std::endl
    //           << trajectory << std::endl;
    if (env.getPlotter() != nullptr) {
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
    //                 "trajectory/open_door_pull.csv",
    //                 ',');
    // saveTrajToFile(trajectory,
    // "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/open_door_pull.csv");

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);

    // for (auto joint_name :
    // env.getVKCEnv()->getTesseract()->getActiveJointNames())
    // {
    //     std::cout << joint_name << std::endl;
    // }

    // std::cout << env.getVKCEnv()->getTesseract()->getCurrentJointValues() <<
    // std::endl;

    // ROS_WARN("environment updated");
    if (env.getPlotter() != nullptr) env.getPlotter()->clear();
    ++j;
  }
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

    joint_objectives.emplace_back("door_north_door_joint", -M_PI_2);
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

    joint_objectives.emplace_back("drawer0_base_drawer_joint", -0.8);
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

Eigen::VectorXd sampleArmPose(vkc::VKCEnvBasic &env,
                              Eigen::Isometry3d ee_goal) {
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

  Eigen::VectorXd ik_result = ik_seed;
  tesseract_kinematics::IKSolutions result;
  for (int tries = 0; tries < 1000; tries++) {
    result = kin->calcInvKin(ik_inputs, ik_seed);
    if (result.size() > 0) break;
  }

  double max_cost = 1e6;
  bool in_collision = false;
  for (const auto &res : result) {
    if ((ik_seed - res).array().abs().sum() < max_cost) {
      ik_result = res;
      max_cost = (ik_seed - res).array().abs().sum();
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() > 0) {
        in_collision = true;
      } else {
        in_collision = false;
      }
      std::cout << "found ik, collision: " << in_collision
                << ", cost: " << max_cost << std::endl;
    }
  }
  contact_results.clear();
  return ik_result;
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

  while (true) {
    Eigen::VectorXd ik_seed =
        tesseract_common::generateRandomNumber(limits.joint_limits);
    tesseract_kinematics::IKSolutions result =
        kin->calcInvKin(ik_inputs, ik_seed);
    for (const auto &res : result) {
      env.getVKCEnv()->getTesseract()->setState(joint_names, res);
      env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
          contact_results, tesseract_collision::ContactTestType::ALL);
      if (contact_results.size() == 0) {
        env.getVKCEnv()->getTesseract()->setState(joint_names,
                                                  initial_joint_values);
        return res;
      }
    }
    contact_results.clear();
  }
}

void run_baseline1(vector<TesseractJointTraj> &joint_trajs,
                   vkc::VKCEnvBasic &env, vkc::ActionSeq &actions, int n_steps,
                   int n_iter, bool rviz_enabled, unsigned int nruns,
                   int envid) {
  // push door open baseline 1
  std::string target_joint("");
  double target_value = 0.;
  vkc::BaseObject::AttachLocation::ConstPtr attach_location_ptr;
  if (envid == 1) {
    target_joint = "door_north_door_joint";
    target_value = M_PI_2;
    attach_location_ptr =
        env.getAttachLocation("attach_door_north_handle_link");
  } else if (envid == 2) {
    target_joint = "drawer0_base_drawer_joint";
    target_value = 0.8;
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
  run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);

  env.getVKCEnv()->getTesseract()->setState(joint_target);

  Eigen::Isometry3d pose_place =
      env.getVKCEnv()->getTesseract()->getLinkTransform(
          attach_location_ptr->link_name_) *
      attach_location_ptr->local_joint_origin_transform;

  actions.clear();
  if (envid == 2){
    env.getVKCEnv()->getTesseract()->setState(joint_init);
  }
  moveBase(actions, sampleBasePose(env, pose_place));
  run(joint_trajs, env, actions, n_steps, n_iter, rviz_enabled, nruns);

  tesseract_common::JointTrajectory base_traj = joint_trajs.back();
  vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
  Eigen::VectorXd base_values = Eigen::Vector2d(0, 0);

  env.getVKCEnv()->getTesseract()->setState(joint_init);

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
    Eigen::VectorXd arm_pose = sampleArmPose(env, ee_target);
    env.getVKCEnv()->getTesseract()->setState(env.getVKCEnv()
                                                  ->getTesseract()
                                                  ->getKinematicGroup("arm")
                                                  ->getJointNames(),
                                              arm_pose);
    if (rviz_enabled)
      env.getPlotter()->waitForInput("press Enter key to go on...");
  }
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
  bool runbs = false;
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);
  pnh.param<int>("envid", envid, envid);
  pnh.param<bool>("baseline", runbs, runbs);

  BenchmarkEnv env(nh, plotting, rviz, steps, envid, runbs);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  vector<TesseractJointTraj> joint_trajs;

  ActionSeq actions;
  //   pushDoor(actions, robot);
  //   pullDoor(actions, robot);
  // pullDrawer(actions, robot);
  // pushDrawer(actions, robot);
  // run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

  run_baseline1(joint_trajs, env, actions, steps, n_iter, rviz, nruns, envid);
}