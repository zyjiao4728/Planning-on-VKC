#include <vkc/env/urdf_scene_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

#include <vkc/action/actions.h>

// motion planning via OMPL
#include "vkc/planner/prob_translator.h"

#include <tesseract_collision/core/types.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/chain_ompl_interface.h>

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;
using TesseractJointTraj = tesseract_common::JointTrajectory;
// using namespace vkc_example;

void run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env, ActionSeq actions, int n_steps, int n_iter, bool rviz_enabled)
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
  vector<double> base_values({ 0.0, 2 });
  env.getVKCEnv()->getTesseract()->getEnvironment()->setState(base_joints, base_values);

  // create OMPL planner specified problem translator
  vkc::OmplPlanParameters params; // use all default parameters for our beginning
  params.plan_params.n_steps = n_steps;
  params.inv_attp_max = 1000;
  ProbTranslator prob_translator(params);

  for (auto &action : actions)
  {
    std::cout << "motion planning for action: " << std::endl
              << action << std::endl;
    PlannerResponse response;
    TrajOptProb::Ptr prob_ptr = nullptr;
    plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

    // TODO: articulated body planning together with robot is still has problem
    // solve by OMPL to get an init trajectory
    std::vector<std::vector<double>> res_traj;
    prob_translator.transProb(env, action);
    prob_translator.solveProblem(response, res_traj);
    if (response.status_code)
    {
      ROS_INFO("[%s]OMPL planning plan successfully", __func__);
      action->setInitTrajectory(response.trajectory);
    }
    else
    {
      ROS_INFO("[%s]OMPL planning plan failed", __func__);
    }

    ROS_INFO("[%s]press enter to go on...", __func__);
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    bool converged = false;
    int tries = 0;
    std::chrono::duration<double> elapsed_seconds;

    while (!converged && tries < 5)
    {
      tries += 1;
      prob_ptr = prob_generator.genRequest(env, action, n_steps);

      if (rviz_enabled)
      {
        ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
        ROS_WARN("Running optimization iteration : %i", tries);
        // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
      
      auto start = std::chrono::system_clock::now();
      cost = solveProb_cost(prob_ptr, response, n_iter);
      elapsed_seconds = std::chrono::system_clock::now() - start;

      if (0 == response.status_code){
        converged = true;
      }
    }


    // record planning result
    planner_responses.push_back(response);
    // record optimized joint names in this step
    joint_names_record.push_back(prob_ptr->GetKin()->getJointNames());

    // refine the orientation of the move base
    tesseract_common::TrajArray refined_traj =
        response.trajectory.leftCols(static_cast<long>(prob_ptr->GetKin()->getJointNames().size()));
    std::cout << "Raw Traj:" << std::endl
              << refined_traj << std::endl;
    refineTrajectory(refined_traj);
    std::cout << "Refined Traj:" << std::endl
              << refined_traj << std::endl;

    joint_trajs.emplace_back(TesseractJointTraj{response.joint_names, refined_traj});
    // plot current `action` result
    if (rviz_enabled)
    {
      ROS_WARN("optimizing problem finished. Press <Enter> to start update environment");
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

    // update env according to the action
    env.updateEnv(joint_names_record.back(), response.trajectory.bottomRows(1).transpose(), action);

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
  action->RequireInitTraj(true);
  seq.push_back(action);

  link_objectives.clear();
  joint_objectives.clear();
  destination.translation() = Eigen::Vector3d(-1.6, 1.4, 0.9);
  destination.linear() = Eigen::Quaterniond(0.5, 0.5, -0.50, -0.50).matrix();
  // destination.linear() = Eigen::Quaterniond(0.7071, 0.7071, 0, 0.0).matrix();
  link_objectives.push_back(LinkDesiredPose("bottle", destination));
  action = make_shared<PlaceAction>("vkc", "attach_bottle", link_objectives, joint_objectives);

  seq.push_back(action);
}

int main(int argc, char **argv)
{
  srand((unsigned)time(NULL)); // for generating waypoint randomly motion planning
  ros::init(argc, argv, "urdf_scene_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
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

  UrdfSceneEnv::AttachObjectInfos attaches;
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                                       "fridge_0001_dof_rootd_Aa002_r",
                                                       "fridge_0001",
                                                       {0.61, -0.30, -0.60},
                                                       {0.0, 0.7071, 0.7071, 0.0},
                                                       true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_bottle",
                                                       "bottle_link_0",
                                                       "bottle",
                                                       {-0.04, 0.0, -0.15},
                                                       {0.707, 0.707, -0.0, 0.0},
                                                      //  {0.5, 0.5, -0.5, 0.5},
                                                       false});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_drawer",
                                                       "cabinet_45290_2_link_0",
                                                       "cabinet_45290_2_base",
                                                       {-0.05, 0, -0.17},
                                                       {1, 0, 0, 0},
                                                       true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_door",
                                                       "door_8966_link_2",
                                                       "door_8966_base",
                                                       {0, 0, -0.3},
                                                       {0.7071, 0, -0.7071, 0},
                                                       true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_dishwasher",
                                                       "dishwasher_12065_link_0",
                                                       "dishwasher_12065",
                                                       {0.0, -0.2, 0.35},
                                                       {0.7071, 0, 0.7071, 0},
                                                       true});

  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
                                                       "link_1",
                                                       "cabinet_44781",
                                                       {0.6, -0.65, 0.2},
                                                       {0.5, 0.5, 0.5, -0.5},
                                                       true});

  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  //inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"fridge_0001", "fridge_0001_dof_rootd_Aa002_r"});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"bottle", "bottle_link_0"});
  //inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_45290_2_base", "cabinet_45290_2_link_0"});
  //inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"door_8966_base", "door_8966_link_2"});
  //inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"dishwasher_12065", "dishwasher_12065_link_0"});
  //inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet_44781", "link_1"});

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);

  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("bottle_link_0", "bottle_link_1", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("bottle_link_1", "rect_table", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_1_link_0", "cabinet_45290_1_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_1_link_1", "cabinet_45290_1_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_1_link_2", "cabinet_45290_1_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_2_link_0", "cabinet_45290_2_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_2_link_1", "cabinet_45290_2_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("cabinet_45290_2_link_2", "cabinet_45290_2_link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("dishwasher_12065_link_0", "dishwasher_12065_link_4", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("dishwasher_12065_link_1", "dishwasher_12065_link_4", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("dishwasher_12065_link_2", "dishwasher_12065_link_4", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("dishwasher_12065_link_3", "dishwasher_12065_link_4", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("door_8966_link_0", "door_8966_link_1", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("door_8966_link_0", "door_8966_link_2", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("fridge_0001", "fridge_0001_dof_rootd_Aa002_r", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("fridge_0001", "fridge_0001_dof_rootd_Ba001_t", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("fridge_0001", "fridge_0001_dof_rootd_Ba002_t", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("fridge_0001", "fridge_0001_dof_rootd_Ba003_t", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("fridge_0001_dof_rootd_Aa002_r", "fridge_0001_dof_rootd_Ba003_t", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("link_0", "link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("link_1", "link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("link_2", "link_3", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("link_0", "link_3", "Never");

  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_1", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_2", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_3", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_4", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_5", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_6", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("oven_101917_link_7", "oven_101917_link_8", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("rect_table", "ur_arm_forearm_link", "Never");
  env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("rect_table", "ur_arm_wrist_1_link", "Never");

  ActionSeq actions;
  genVKCDemoDeq(actions, env.getHomePose());
  // cache the planning result for replaying
  vector<TesseractJointTraj> joint_trajs;

  run(joint_trajs, env, actions, steps, n_iter, rviz);

  // visualize the trajectory as planned
  TrajectoryVisualize(env, actions, joint_trajs);
}
