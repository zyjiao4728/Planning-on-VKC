#include <vkc/env/urdf_scene_env.h>
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

void run(VKCEnvBasic &env, ActionSeq actions, int n_steps, int n_iter, bool rviz_enabled)
{
  ProbGenerator prob_generator;
  ROSPlottingPtr plotter;

  vector<vector<string> > joint_names_record;
  vector<PlannerResponse> planner_responses;

  CostInfo cost;

  vector<string> dishwasher_joints({ "dishwasher_12065_joint_0", "dishwasher_12065_joint_1" , "dishwasher_12065_joint_2" });
  vector<double> dishwasher_values({ 0.3080, -0.1359, 0.9250}); //open: 0.9250 close -0.6457
  env.getVKCEnv()->getTesseract()->getEnvironment()->setState(dishwasher_joints, dishwasher_values);

  vector<string> base_joints({ "base_y_base_x", "base_theta_base_y" });
  vector<double> base_values({ 0, 2 });
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

      if (rviz_enabled)
      {
        ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
        ROS_WARN("Running optimization iteration : %i", tries);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
      
      auto start = std::chrono::system_clock::now();
      cost = solveProb_cost(prob_ptr, response, n_iter);
      elapsed_seconds = std::chrono::system_clock::now() - start;

      if (response.status.value() == 0 || action->getActionType() == ActionType::PlaceAction){
        converged = true;
      }
    }

    if (rviz_enabled)
    {
      ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    // record planning result
    planner_responses.push_back(response);
    // record optimized joint names in this step
    joint_names_record.push_back(prob_ptr->GetKin()->getJointNames());

    // refine the orientation of the move base
    tesseract_common::TrajArray refined_traj =
        response.joint_trajectory.trajectory.leftCols(static_cast<long>(prob_ptr->GetKin()->getJointNames().size()));
    
    std::cout << "Traj:" << std::endl;
    std::cout << refined_traj << std::endl;

    refineTrajectory(refined_traj);

    // plot current `action` result

    plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

    ROS_WARN("Update Env");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // update env according to the action
    env.updateEnv(joint_names_record.back(), response.joint_trajectory.trajectory.bottomRows(1).transpose(), action);

    plotter->clear();
  }
}

void genVKCDemoDeq(ActionSeq &seq, std::unordered_map<std::string, double> home_pose)
{
  std::vector<JointDesiredPose> joint_home = getJointHome(home_pose);
  ActionBase::Ptr action;
  vector<LinkDesiredPose> link_objectives;
  vector<JointDesiredPose> joint_objectives;
  Eigen::Isometry3d destination;

  action = make_shared<PickAction>("vkc", "attach_bottle");
  seq.push_back(action);

  link_objectives.clear();
  joint_objectives.clear();
  destination.translation() = Eigen::Vector3d(-1.6, 1.4, 0.9);
  destination.linear() = Eigen::Quaterniond(0.7071, 0.7071, 0, 0.0).matrix();
  link_objectives.push_back(LinkDesiredPose("bottle", destination));
  action = make_shared<PlaceAction>("vkc", "attach_bottle", link_objectives, joint_objectives);
  seq.push_back(action);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_scene_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  int steps = 10;
  int n_iter = 1000;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);

  UrdfSceneEnv env(nh, plotting, rviz, steps);

  ActionSeq actions;
  genVKCDemoDeq(actions, env.getHomePose());
  
  run(env, actions, steps, n_iter, rviz);
}
