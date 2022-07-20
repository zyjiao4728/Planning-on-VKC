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

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);

    if (env.getPlotter() != nullptr && rviz_enabled) env.getPlotter()->clear();
    ++j;
  }
}

void genVKCDemoDeq(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open door **/
  // action 1: pick the door handle
  {
    auto pick_action = make_shared<PickAction>(robot, "attach_bottle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() = Eigen::Vector3d(-1.6, 1.6, 0.9);
    destination.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();
    link_objectives.push_back(LinkDesiredPose("bottle", destination));

    place_action = make_shared<PlaceAction>(
        robot, "attach_bottle", link_objectives, joint_objectives, false);

    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }
}

void setInitState(VKCEnvBasic &env) {
  vector<string> dishwasher_joints({ "dishwasher_12065_joint_0", "dishwasher_12065_joint_1" , "dishwasher_12065_joint_2" });
  Eigen::Vector3d dishwasher_values({ 0.3080, -0.1359, -0.6457}); //open: 0.9250 close -0.6457
  env.getVKCEnv()->getTesseract()->setState(dishwasher_joints, dishwasher_values);
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
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);

  UrdfSceneEnv::AttachObjectInfos attaches;

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
                                     {0.61, -0.30, -0.60},
                                     {0.0, 0.707106781, 0.707106781, 0.0},
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
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"bottle", "bottle_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"fridge_0001",
  // "fridge_0001_dof_rootd_Aa002_r"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_45290_2_base",
  // "cabinet_45290_2_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"door_8966_base",
  // "door_8966_link_2"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"dishwasher_12065",
  // "dishwasher_12065_link_0"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_44781",
  // "link_1"});

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
  setInitState(env);
  ActionSeq actions;
  genVKCDemoDeq(actions, robot);
  vector<TesseractJointTraj> joint_trajs;
  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
}
