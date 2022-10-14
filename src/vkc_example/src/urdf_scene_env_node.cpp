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

void run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env,
         ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled,
         unsigned int nruns) {
  int window_size = 3;
  LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size, 9);
  ProbGenerator prob_generator;

  int j = 0;

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

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

        // action->switchCandidate();
      }
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

    toDelimitedFile(ci,
                    "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/"
                    "trajectory/urdf_scene_env_" +
                        std::to_string(j) + ".csv",
                    ',');

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

void genVKCDemoSeq(vkc::ActionSeq &actions, const std::string &robot) {
  PickAction::Ptr pick_action;
  PlaceAction::Ptr place_action;

  // action 1: pick fridge handle
  {
    pick_action = make_shared<PickAction>(robot, "attach_fridge_handle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action 2: open fridge door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint", -1.6);
    place_action =
        make_shared<PlaceAction>(robot, "attach_fridge_handle", link_objectives,
                                 joint_objectives, false);

    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

    actions.emplace_back(place_action);
  }

  // action 3: pick bottle
  {
    pick_action = make_shared<PickAction>(robot, "attach_bottle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action 4: place bottle in the fridge
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() = Eigen::Vector3d(3.0, 3.0, 0.76);
    destination.linear() =
        Eigen::Quaterniond(0.70710678118, 0.70710678118, 0.0, 0).matrix();
    // destination.translation() = Eigen::Vector3d(-1.6, 1.6, 0.9);
    // destination.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50,
    // -0.50).matrix();
    link_objectives.push_back(LinkDesiredPose("bottle", destination));

    place_action = make_shared<PlaceAction>(
        robot, "attach_bottle", link_objectives, joint_objectives, false);

    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  // action 5: pick fridge handle
  {
    pick_action = make_shared<PickAction>(robot, "attach_fridge_handle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action 6: close fridge door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint", 0.0);
    place_action =
        make_shared<PlaceAction>(robot, "attach_fridge_handle", link_objectives,
                                 joint_objectives, false);

    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

    actions.emplace_back(place_action);
  }
}

void genVKCDemoEnvironmentInfo(
    UrdfSceneEnv::AttachObjectInfos &attaches,
    UrdfSceneEnv::InverseChainsInfos &inverse_chains) {
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_bottle",
                                                       "bottle_link_0",
                                                       "bottle",
                                                       {-0.2, 0, -0.0},
                                                       {0.5, 0.5, 0.5, 0.5},
                                                       false});

  attaches.emplace_back(
      UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                     "fridge_0001_dof_rootd_Aa002_r",
                                     "fridge_0001",
                                     {0.95, -0.28, -0.85},
                                     {0.5, -0.5, 0.5, 0.5},
                                     true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_drawer",
                                                       "cabinet_45290_2_link_0",
                                                       "cabinet_45290_2_base",
                                                       {-0.05, 0, -0.17},
                                                       {1, 0, 0, 0},
                                                       true});

  attaches.emplace_back(
      UrdfSceneEnv::AttachObjectInfo{"attach_door",
                                     "door_8966_link_2",
                                     "door_8966_base",
                                     {0, 0, -0.3},
                                     {0.707106781, 0, -0.707106781, 0},
                                     true});

  // attaches.emplace_back(
  //     UrdfSceneEnv::AttachObjectInfo{"attach_dishwasher",
  //                                    "dishwasher_12065_link_0",
  //                                    "dishwasher_12065",
  //                                    {0.0, -0.2, 0.35},
  //                                    {0.707106781, 0, 0.707106781, 0},
  //                                    true});

  // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
  //                                                      "link_1",
  //                                                      "cabinet_44781",
  //                                                      {0.6, -0.65, 0.2},
  //                                                      {0.5, 0.5, 0.5, -0.5},
  //                                                      true});
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"bottle", "bottle_link_0"});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "fridge_0001", "fridge_0001_dof_rootd_Aa002_r"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_45290_2_base",
  // "cabinet_45290_2_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"door_8966_base",
  // "door_8966_link_2"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"dishwasher_12065",
  // "dishwasher_12065_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_44781",
  // "link_1"});
  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

void genTRODemoSeq(VKCEnvBasic &env, vkc::ActionSeq &actions,
                   const std::string &robot, int task_id) {
  PickAction::Ptr pick_action;
  PlaceAction::Ptr place_action;

  Eigen::VectorXd cost_coeff;
  cost_coeff.setOnes(10);
  cost_coeff[9] = 0.;
  switch (task_id) {
    case 1:
      // action 1: pick fridge handle
      {
        pick_action = make_shared<PickAction>(robot, "attach_fridge_handle");
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action 2: open fridge door
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint",
                                      -1.6);
        place_action =
            make_shared<PlaceAction>(robot, "attach_fridge_handle",
                                     link_objectives, joint_objectives, false);

        place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

        cost_coeff[3] = 5.;
        cost_coeff[0] = 2.;
        cost_coeff[1] = 2.;
        place_action->setIKCostCoeff(cost_coeff);

        actions.emplace_back(place_action);
      }
      break;
    case 2:
      // action 1: pick door handle
      {
        pick_action = make_shared<PickAction>(robot, "attach_door_handle");
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action 2: open door door
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("door_8966_joint_1", 1.5);
        place_action =
            make_shared<PlaceAction>(robot, "attach_door_handle",
                                     link_objectives, joint_objectives, false);

        place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

        cost_coeff[6] = 5.;
        cost_coeff[3] = 3.;
        cost_coeff[0] = 3.;
        cost_coeff[1] = 3.;
        place_action->setIKCostCoeff(cost_coeff);

        actions.emplace_back(place_action);
      }
      break;
    case 3:
      // action 1: pick cup
      {
        pick_action = make_shared<PickAction>(robot, "attach_cup");
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action 2: place cup on the table
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        Eigen::Isometry3d destination;
        destination.setIdentity();
        destination.translation() = Eigen::Vector3d(-0.6, -1.8, 1.11);
        destination.linear() =
            Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();
        link_objectives.push_back(
            LinkDesiredPose("cup_cup_base_link", destination));

        place_action = make_shared<PlaceAction>(
            robot, "attach_cup", link_objectives, joint_objectives, true);

        place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(place_action);
      }
      break;
    case 4:
      // action1: pick drawer handle
      {
        auto pick_action = std::make_shared<PickAction>(robot, "attach_drawer");
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action2: place drawer handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer_base_drawer1_joint", -0.22);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_drawer", link_objectives, joint_objectives, false);
        // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
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
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action2: place cabinet handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("cabinet_48479_joint_0", -0.4);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_cabinet", link_objectives, joint_objectives, false);
        // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(place_action);
      }
      break;
    case 6:
      // action1: pick cabinet handle
      {
        auto pick_action =
            std::make_shared<PickAction>(robot, "attach_dishwasher");
        pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(pick_action);
      }

      // action2: place cabinet handle
      {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("dishwasher_joint_0", 0.3);
        auto place_action = std::make_shared<PlaceAction>(
            robot, "attach_dishwasher", link_objectives, joint_objectives,
            false);
        cost_coeff[6] = 5.;
        cost_coeff[3] = 3.;
        cost_coeff[0] = 3.;
        cost_coeff[1] = 3.;
        place_action->setIKCostCoeff(cost_coeff);
        // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
        actions.emplace_back(place_action);
      }

      {
        vector<string> joint_names(
            {"dishwasher_joint_0", "dishwasher_joint_1",
             "dishwasher_joint_2"});  //, "cabinet_48479_joint_0"
        Eigen::Vector3d joint_values(
            {0.0, 0.0, 0.9});  // open: 0.9250 close -0.6457 , -0.12
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
  }
}

void genTRODemoEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                               UrdfSceneEnv::InverseChainsInfos &inverse_chains,
                               int task_id) {
  switch (task_id) {
    case 1:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                         "fridge_0001_dof_rootd_Aa002_r",
                                         "fridge_0001",
                                         {0.65, -0.25, -0.65},
                                         {0.5, -0.5, 0.5, 0.5},
                                         true});
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
      inverse_chains.emplace_back(
          UrdfSceneEnv::InverseChainsInfo{"cup_cup_base_link", "cup_cup_link"});
      break;

    case 4:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_drawer",
                                         "drawer_handle1",
                                         "drawer_base_link",
                                         {0.16, 0.000, 0.00},
                                         {0.5000, -0.5000, -0.5000, 0.5000},
                                         true});
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
      inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
          "cabinet_48479_base_link", "cabinet_48479_handle0"});
      break;

    case 6:
      attaches.emplace_back(
          UrdfSceneEnv::AttachObjectInfo{"attach_dishwasher",
                                         "dishwasher_link_0",
                                         "dishwasher",
                                         {0.0, -0.2, 0.37},
                                         {0, 1, 0, 0},
                                         true});
      inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
          "dishwasher", "dishwasher_link_0"});
      break;

    default:
      break;
  }

  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

int main(int argc, char **argv) {
  srand((unsigned)time(
      NULL));  // for generating waypoint randomly motion planning
  ros::init(argc, argv, "urdf_scene_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  setupLog(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
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

  UrdfSceneEnv::AttachObjectInfos attaches;
  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  genTRODemoEnvironmentInfo(attaches, inverse_chains, taskid);

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
  ActionSeq actions;
  genTRODemoSeq(env, actions, robot, taskid);
  vector<TesseractJointTraj> joint_trajs;
  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
}
