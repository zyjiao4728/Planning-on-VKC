#include <ros/console.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/uam_env.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>
#include <trajopt_utils/eigen_conversions.hpp>

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
  // int window_size = 3;
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
        ActionSeq sub_actions(ptr, actions.end());
        // seed_generator.generate(env, sub_actions);
      }
    }

    const auto &ci = response.results;

    tesseract_common::JointTrajectory refined_traj = toJointTrajectory(ci);
    joint_trajs.emplace_back(refined_traj);

    // ROS_WARN("trajectory: ");
    // for (auto jo : refined_traj) {
    //   std::cout << jo.position << std::endl;
    // }

    // refine the orientation of the move base

    // tesseract_common::TrajArray refined_traj =
    //     response.trajectory.leftCols(response.joint_names.size());
    // refineTrajectory(refined_traj);

    // std::cout << "optimized trajectory: " << std::endl
    //           << refined_traj << std::endl;
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

    toDelimitedFile(ci,
                    "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/"
                    "trajectory/uam_lightbulb_" + std::to_string(j) +".csv",
                    ',');
    // saveTrajToFile(refined_traj,
    // "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/uam_lightbulb.csv");
    env.updateEnv(refined_traj.back().joint_names, refined_traj.back().position,
                  action);
    CONSOLE_BRIDGE_logInform("environment updated, starting next action...");
    if (env.getPlotter() != nullptr) env.getPlotter()->clear();
    ++j;
  }
}

void installLightBulb(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** install light bulb **/
  // action 1: pick the light bulb
  {
    auto pick_action =
        make_shared<PickAction>(robot, "attach_lightbulb0_marker_link");
    actions.emplace_back(pick_action);
  }

  // action 2: place the light bulb
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() = Eigen::Vector3d(3, 0, 2.2);
    destination.linear() = Eigen::Quaterniond(0, 0, -1, 0.0).matrix();
    link_objectives.push_back(LinkDesiredPose("lightbulb0_base_link", destination));

    place_action =
        make_shared<PlaceAction>(robot, "attach_lightbulb0_marker_link",
                                 link_objectives, joint_objectives, false);

    actions.emplace_back(place_action);
  }
}

void picknplaceLightBulb(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** install light bulb **/
  // action 1: pick the light bulb
  {
    auto pick_action =
        make_shared<PickAction>(robot, "attach_bulb");
    actions.emplace_back(pick_action);
  }

  // action 2: place the light bulb
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() = Eigen::Vector3d(0.4, 1.52, 2.1);
    destination.linear() = Eigen::Quaterniond(0.70710678, 0.70710678, 0.0, 0.0).matrix();
    link_objectives.push_back(LinkDesiredPose("bulb0_bulb_link", destination));

    place_action =
        make_shared<PlaceAction>(robot, "attach_bulb",
                                 link_objectives, joint_objectives, true);

    actions.emplace_back(place_action);
  }
}

void openCabinet(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open cabinet **/
  // action 1: pick the cabinet handle
  {
    auto pick_action =
        make_shared<PickAction>(robot, "attach_cabinet");
    // pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
    (*actions.rbegin())->RequireInitTraj(false);
  }

  // action 2: open cabinet
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("cabinet0_small_left_door_joint", -1.57);
    place_action =
        make_shared<PlaceAction>(robot, "attach_cabinet",
                                 link_objectives, joint_objectives, false);
    actions.emplace_back(place_action);
  }
}

void closeCabinet(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** close cabinet **/
  // action 1: pick the cabinet handle
  {
    auto pick_action =
        make_shared<PickAction>(robot, "attach_cabinet");
    // pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
    (*actions.rbegin())->RequireInitTraj(false);
  }

  // action 2: close cabinet
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("cabinet0_small_left_door_joint", 0.0);
    place_action =
        make_shared<PlaceAction>(robot, "attach_cabinet",
                                 link_objectives, joint_objectives, false);
    actions.emplace_back(place_action);
  }
}


void setInitState(VKCEnvBasic &env) {
  vector<string> cabinet_joint({ "cabinet0_small_left_door_joint",  "cabinet0_small_right_door_joint" });
  Eigen::Vector2d cabinet_value({ 1.57, 0.0 }); //open: 0.9250 close -0.6457
  env.getVKCEnv()->getTesseract()->setState(cabinet_joint, cabinet_value);
}


int main(int argc, char **argv) {
  srand(time(NULL));
  ros::init(argc, argv, "uam_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  console_bridge::setLogLevel(
      console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);

  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int steps = 30;
  int n_iter = 1000;
  int nruns = 5;
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);

  UrdfSceneEnv::AttachObjectInfos attaches;

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
                                                       "cabinet0_small_left_door_link",
                                                       "cabinet0_base_link",
                                                       {0.15, 0.515, 0.32},
                                                       {0.70710678,0.70710678,0.,0.},
                                                       true});
  
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_bulb",
                                                       "bulb0_bulb_link",
                                                       "bulb0_bulb_link",
                                                       {0, 0, 0.15},
                                                       {0.70710678, 0, -0.70710678, 0},
                                                       false});

  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"cabinet0_base_link", "cabinet0_small_left_door_link"});

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);

  Commands cmds;
  cmds.clear();
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_base_link", "gripper_base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "bulb0_bulb_link", "gripper_base_link", "Never"));

  env.getVKCEnv()->getTesseract()->applyCommands(cmds);

  // setInitState(env); 
  vector<TesseractJointTraj> joint_trajs;
  ActionSeq actions;
  // installLightBulb(actions, robot);
  openCabinet(actions, robot);
  picknplaceLightBulb(actions, robot);
  closeCabinet(actions, robot);

  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
}