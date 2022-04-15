#include <vkc/env/open_door_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

#include <vkc/action/actions.h>

#include <iostream>
#include <string>
#include <vector>
#include <tesseract_command_language/utils/utils.h>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
// using namespace vkc_example;

using TesseractJointTraj = tesseract_common::JointTrajectory;

void run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env, ActionSeq &actions,
         int n_steps, int n_iter, bool rviz_enabled, unsigned int nruns)
{
  ProbGenerator prob_generator;

  int j = 0;

  for (auto &action : actions)
  {

    ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract());

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns)
    {
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      ROS_WARN("optimization is ready. Press <Enter> to start next action");
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      // CostInfo cost = solveProb(prob_ptr, response, n_iter);
      solveProb(prob_ptr, response, n_iter);

      if (TrajOptMotionPlannerStatusCategory::SolutionFound == response.status.value()) // optimization converges
      {
        converged = true;
        break;
      }
      else
      {
        ROS_WARN("[%s]optimizationi could not converge, response code: %d, description: %s",
                 __func__, response.status.value(), response.status.message().c_str());
      }
    }

    const auto &ci = response.results;

    tesseract_common::JointTrajectory refined_traj = toJointTrajectory(ci);
    joint_trajs.emplace_back(refined_traj);

    // refine the orientation of the move base

    // tesseract_common::TrajArray refined_traj =
    //     response.trajectory.leftCols(response.joint_names.size());
    // refineTrajectory(refined_traj);

    // std::cout << "optimized trajectory: " << std::endl
    //           << refined_traj << std::endl;

    plotter->plotTrajectory(refined_traj, *env.getVKCEnv()->getTesseract()->getStateSolver());

    ROS_WARN("Finished optimization. Press <Enter> to start next action");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    toDelimitedFile(ci, "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/open_door_pull.csv", ',');
    // saveTrajToFile(refined_traj, "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/open_door_pull.csv");

    env.updateEnv(refined_traj.back().joint_names, refined_traj.back().position, action);
    plotter->clear();
    ++j;
  }

  // ProbGenerator prob_generator;
  // ROSPlottingPtr plotter;

  // vector<vector<string>> joint_names_record;
  // vector<PlannerResponse> planner_responses;

  // for (auto &action : actions)
  // {
  //   PlannerResponse response;
  //   TrajOptProb::Ptr prob_ptr = nullptr;
  //   plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

  //   prob_ptr = prob_generator.genRequest(env, action, n_steps);

  //   if (rviz_enabled)
  //   {
  //     ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
  //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  //   }

  //   solveProb(prob_ptr, response, n_iter);
  //   // record planning result
  //   planner_responses.push_back(response);
  //   // record optimized joint names in this step
  //   joint_names_record.push_back(prob_ptr->GetKin()->getJointNames());

  //   // refine the orientation of the move base
  //   tesseract_common::TrajArray refined_traj =
  //       response.trajectory.leftCols(response.joint_names.size());
  //   refineTrajectory(refined_traj);

  //   std::cout << "Refined traj:" << std::endl;
  //   std::cout << refined_traj << std::endl;

  //   // plot current `action` result

  //   plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

  //   ROS_WARN("Update Env");
  //   std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  //   // update env according to the action
  //   env.updateEnv(response.joint_names, response.trajectory.bottomRows(1).transpose(), action);
  //   plotter->clear();

  //   plotter->clear();
  // }
}

void pullDoor(vkc::ActionSeq &actions, const std::string &robot)
{
  PlaceAction::Ptr place_action;

  /** open door **/
  // action 1: pick the door handle
  {
    actions.emplace_back(make_shared<PickAction>(robot, "attach_door_north_handle_link"));
    (*actions.rbegin())->RequireInitTraj(false);
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", 1.5);
    place_action = make_shared<PlaceAction>(robot, "attach_door_north_handle_link", link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    place_action->RequireInitTraj(true);

    actions.emplace_back(place_action);
  }
}

void pushDoor(vkc::ActionSeq &actions, const std::string &robot)
{
  PlaceAction::Ptr place_action;

  /** open door **/
  // action 1: pick the door handle
  {
    actions.emplace_back(make_shared<PickAction>(robot, "attach_door_north_handle_link"));
    (*actions.rbegin())->RequireInitTraj(false);
  }

  // action 2: open door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_north_door_joint", -1.5);
    place_action = make_shared<PlaceAction>(robot, "attach_door_north_handle_link", link_objectives, joint_objectives, false);
    place_action->setOperationObjectType(false);

    place_action->RequireInitTraj(true);

    actions.emplace_back(place_action);
  }
}

int main(int argc, char **argv)
{
  srand(time(NULL));
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
  pushDoor(actions, robot);

  run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);
}