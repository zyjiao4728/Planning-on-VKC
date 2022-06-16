#include <ros/console.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/open_door_env.h>
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

void pullDoor(vkc::ActionSeq &actions, const std::string &robot) {
  PlaceAction::Ptr place_action;

  /** open door **/
  // action 1: pick the door handle
  {
    auto pick_action =
        make_shared<PickAction>(robot, "attach_door_north_handle_link");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
    (*actions.rbegin())->RequireInitTraj(false);
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", 1.5);
    place_action =
        make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                 link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    place_action->RequireInitTraj(true);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

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
    (*actions.rbegin())->RequireInitTraj(false);
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

    place_action->RequireInitTraj(true);

    actions.emplace_back(place_action);
  }

  ROS_INFO("push door actions created.");
}

int main(int argc, char **argv) {
  srand(time(NULL));

  console_bridge::setLogLevel(
      console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  ros::init(argc, argv, "open_door_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

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

  OpenDoorEnv env(nh, plotting, rviz, steps);

  vector<TesseractJointTraj> joint_trajs;

  ActionSeq actions;
  // pushDoor(actions, robot);
  pullDoor(actions, robot);

  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
}