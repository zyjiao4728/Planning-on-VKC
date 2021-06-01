#include <vkc/env/arena_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

#include <vkc/action/actions.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;
// using namespace vkc_example;

void run(VKCEnvBasic &env, ActionSeq actions, int n_steps, int n_iter, bool rviz_enabled, int nruns)
{
  ProbGenerator prob_generator;
  ROSPlottingPtr plotter;

  CostInfo cost;
  
  vector<vector<string> > joint_names_record;
  vector<PlannerResponse> planner_responses;

  vector<string> base_joints({ "base_y_base_x", "base_theta_base_y" });
  vector<double> base_values({0, 0});

  env.getVKCEnv()->getTesseract()->getEnvironment()->setState(base_joints, base_values);

  for (auto &action : actions)
  {
    PlannerResponse response;
    TrajOptProb::Ptr prob_ptr = nullptr;
    plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

    bool converged = false;
    int tries = 0;
    std::chrono::duration<double> elapsed_seconds;
    
    while (!converged && tries < 5)
    {
      tries += 1;
      prob_ptr = prob_generator.genProb(env, action, n_steps);

      if ((abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(0)) - 6) < 1e-6 || abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(1)) - 6) < 1e-6) 
      && env.getEndEffectorLink().find("cabinet")==string::npos){
        std::cout << prob_ptr->GetInitTraj().bottomRows(1)(0) << " " << prob_ptr->GetInitTraj().bottomRows(1)(1) << std::endl;
        if (tries < 4)
          continue;
      }

      if (rviz_enabled)
      {
        ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }

      auto start = std::chrono::system_clock::now();
      cost = solveProb_cost(prob_ptr, response, n_iter);
      elapsed_seconds = std::chrono::system_clock::now() - start;

      if (response.status.value() == 0 || action->getActionType() == ActionType::PlaceAction){
        converged = true;
      }
    }

    // record planning result
    planner_responses.push_back(response);
    // record optimized joint names in this step
    joint_names_record.push_back(prob_ptr->GetKin()->getJointNames());

    // refine the orientation of the move base
    tesseract_common::TrajArray refined_traj =
        response.joint_trajectory.trajectory.leftCols(static_cast<long>(prob_ptr->GetKin()->getJointNames().size()));
    refineTrajectory(refined_traj);

    std::cout << "Refined traj:" << std::endl;
    std::cout << refined_traj << std::endl;

    // plot current `action` result

    plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

    if (rviz_enabled)
    {
      ROS_WARN("Update Env");
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    // update env according to the action
    env.updateEnv(joint_names_record.back(), response, action);

    plotter->clear();
  }
}

void genPickBallSeq(ActionSeq &seq, std::unordered_map<std::string, double> home_pose)
{
  std::vector<JointDesiredPose> joint_home = getJointHome(home_pose);
  ActionBase::Ptr action;
  vector<LinkDesiredPose> link_objectives;
  vector<JointDesiredPose> joint_objectives;
  Eigen::Isometry3d destination;

  // pick up the hook
  action = make_shared<PickAction>("vkc", "attach_stick0_stick_link");
  seq.push_back(action);

  // use stick to fetch the ball
  Eigen::Isometry3d tf1;
  tf1.translation() = Eigen::Vector3d(0.1, 0, 0);
  tf1.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();
  action = make_shared<UseAction>("vkc", "attach_marker0_marker_link", tf1, "stick0_stick_link");
  seq.push_back(action);

  // Move the ball out of table
  link_objectives.clear();
  joint_objectives.clear();
  destination.translation() = Eigen::Vector3d(-1, 1.5, 0.1);
  destination.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();
  link_objectives.push_back(LinkDesiredPose("marker0_base_link", destination));
  action = make_shared<PlaceAction>("vkc", "attach_marker0_marker_link", link_objectives, joint_objectives);
  seq.push_back(action);

  //Place hook back to table
  link_objectives.clear();
  joint_objectives.clear();
  destination.translation() = Eigen::Vector3d(2.5, 1, 0.8);
  destination.linear() = Eigen::Quaterniond(0.7071, 0, 0, -0.7071).matrix();
  link_objectives.push_back(LinkDesiredPose("stick0_base_link", destination));
  action = make_shared<PlaceAction>("vkc", "attach_stick0_stick_link", link_objectives, joint_objectives);
  seq.push_back(action);

  // Pick cabinet handle
  action = make_shared<PickAction>("vkc", "attach_cabinet0_handle_link");
  seq.push_back(action);

  // Open cabinet
  link_objectives.clear();
  joint_objectives.clear();
  joint_objectives.push_back(JointDesiredPose("cabinet0_cabinet_door_joint", 1.5));
  action = make_shared<PlaceAction>("vkc", "attach_cabinet0_handle_link", link_objectives, joint_objectives);
  seq.push_back(action);
  
  // Pick ball
  action = make_shared<PickAction>("vkc", "attach_marker0_marker_link");
  seq.push_back(action);

  // Place ball into cabinet
  link_objectives.clear();
  joint_objectives.clear();
  destination.setIdentity();
  destination.translation() = Eigen::Vector3d(0, -2, 1.1);
  destination.linear() = Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5).matrix();
  link_objectives.push_back(LinkDesiredPose("marker0_base_link", destination));
  action = make_shared<PlaceAction>("vkc", "attach_marker0_marker_link", link_objectives, joint_objectives);
  seq.push_back(action);

  // Pick cabinet handle
  action = make_shared<PickAction>("vkc", "attach_cabinet0_handle_link");
  seq.push_back(action);

  // Close cabinet
  link_objectives.clear();
  joint_objectives.clear();
  joint_objectives.push_back(JointDesiredPose("cabinet0_cabinet_door_joint", 0));
  action = make_shared<PlaceAction>("vkc", "attach_cabinet0_handle_link", link_objectives, joint_objectives);
  seq.push_back(action);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_door_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int steps = 10;
  int n_iter = 1;
  int nruns = 1;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);

  ArenaEnv env(nh, plotting, rviz, steps);

  ActionSeq actions;
  genPickBallSeq(actions, env.getHomePose());

  run(env, actions, steps, n_iter, rviz, nruns);
}
