#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>
// motion planning via OMPL
#include <tesseract_collision/core/types.h>
#include <tesseract_command_language/utils/utils.h>
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
  ProbGenerator prob_generator;

  int j = 0;

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      ROS_WARN("generating request");
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
                    "trajectory/open_door_pull.csv",
                    ',');
    // saveTrajToFile(refined_traj,
    // "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/open_door_pull.csv");
    env.updateEnv(refined_traj.back().joint_names, refined_traj.back().position,
                  action);
    CONSOLE_BRIDGE_logInform("environment updated, starting next action...");
    if (env.getPlotter() != nullptr) env.getPlotter()->clear();
    ++j;
  }
}

void genVKCDemoDeq(ActionSeq &seq,
                   std::unordered_map<std::string, double> home_pose) {
  std::vector<JointDesiredPose> joint_home = getJointHome(home_pose);
  ActionBase::Ptr action;
  vector<LinkDesiredPose> link_objectives;
  vector<JointDesiredPose> joint_objectives;
  Eigen::Isometry3d destination;

  action = make_shared<PickAction>("vkc", "attach_bottle");
  action->RequireInitTraj(true);
  seq.push_back(action);

  link_objectives.clear();
  joint_objectives.clear();
  destination.translation() = Eigen::Vector3d(-1.6, 1.4, 0.9);
  destination.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();
  // destination.linear() = Eigen::Quaterniond(0.7071, 0.7071, 0, 0.0).matrix();
  link_objectives.push_back(LinkDesiredPose("bottle", destination));
  action = make_shared<PlaceAction>("vkc", "attach_bottle", link_objectives,
                                    joint_objectives);

  seq.push_back(action);
}

int main(int argc, char **argv) {
  srand((unsigned)time(
      NULL));  // for generating waypoint randomly motion planning
  ros::init(argc, argv, "urdf_scene_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int steps = 10;
  int n_iter = 1000;
  int nruns = 5;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);

  UrdfSceneEnv::AttachObjectInfos attaches;
  attaches.emplace_back(
      UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                     "fridge_0001_dof_rootd_Aa002_r",
                                     "fridge_0001",
                                     {0.61, -0.30, -0.60},
                                     {0.0, 0.707106781, 0.707106781, 0.0},
                                     true});

  attaches.emplace_back(
      UrdfSceneEnv::AttachObjectInfo{"attach_bottle",
                                     "bottle_link_0",
                                     "bottle",
                                     {-0.04, 0.0, -0.15},
                                     {0.707106781, 0.707106781, -0.0, 0.0},
                                     //  {0.5, 0.5, -0.5, 0.5},
                                     false});

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

  attaches.emplace_back(
      UrdfSceneEnv::AttachObjectInfo{"attach_dishwasher",
                                     "dishwasher_12065_link_0",
                                     "dishwasher_12065",
                                     {0.0, -0.2, 0.35},
                                     {0.707106781, 0, 0.707106781, 0},
                                     true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
                                                       "link_1",
                                                       "cabinet_44781",
                                                       {0.6, -0.65, 0.2},
                                                       {0.5, 0.5, 0.5, -0.5},
                                                       true});

  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"fridge_0001",
  // "fridge_0001_dof_rootd_Aa002_r"});
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"bottle", "bottle_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_45290_2_base",
  // "cabinet_45290_2_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"door_8966_base",
  // "door_8966_link_2"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"dishwasher_12065",
  // "dishwasher_12065_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_44781",
  // "link_1"});

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);

  Commands cmds;
  cmds.clear();

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "bottle_link_0", "bottle_link_1", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "bottle_link_1", "rect_table", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_1_link_0", "cabinet_45290_1_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_1_link_1", "cabinet_45290_1_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_1_link_2", "cabinet_45290_1_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_2_link_0", "cabinet_45290_2_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_2_link_1", "cabinet_45290_2_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet_45290_2_link_2", "cabinet_45290_2_link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "dishwasher_12065_link_0", "dishwasher_12065_link_4", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "dishwasher_12065_link_1", "dishwasher_12065_link_4", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "dishwasher_12065_link_2", "dishwasher_12065_link_4", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "dishwasher_12065_link_3", "dishwasher_12065_link_4", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_8966_link_0", "door_8966_link_1", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_8966_link_0", "door_8966_link_2", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "fridge_0001", "fridge_0001_dof_rootd_Aa002_r", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "fridge_0001", "fridge_0001_dof_rootd_Ba001_t", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "fridge_0001", "fridge_0001_dof_rootd_Ba002_t", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "fridge_0001", "fridge_0001_dof_rootd_Ba003_t", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "fridge_0001_dof_rootd_Aa002_r", "fridge_0001_dof_rootd_Ba003_t",
      "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "link_0", "link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "link_1", "link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "link_2", "link_3", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "link_0", "link_3", "Never"));

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_1", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_2", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_3", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_4", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_5", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_6", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "oven_101917_link_7", "oven_101917_link_8", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "rect_table", "ur_arm_forearm_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "rect_table", "ur_arm_wrist_1_link", "Never"));

  ActionSeq actions;
  genVKCDemoDeq(actions, env.getHomePose());
  // cache the planning result for replaying
  vector<TesseractJointTraj> joint_trajs;

  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

  // visualize the trajectory as planned
  TrajectoryVisualize(env, actions, joint_trajs);
}
