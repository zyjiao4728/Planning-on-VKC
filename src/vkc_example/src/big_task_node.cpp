#include "vkc/env/urdf_scene_env.h"
#include "vkc/env/vkc_env_basic.h"
#include "vkc/planner/prob_generator.h"
#include "vkc_example/motion_plan_actions.h"
#include "vkc_example/utils.h"

#include "vkc/action/actions.h"

// motion planning via OMPL
#include "vkc/planner/prob_translator.h"

#include "tesseract_collision/core/types.h"
#include "tesseract_motion_planners/ompl/conversions.h"
#include "tesseract_motion_planners/ompl/chain_ompl_interface.h"

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;
using TesseractJointTraj = tesseract_common::JointTrajectory;

void Run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env, ActionSeq &actions,
         int n_steps, int n_iter, bool rviz_enabled, unsigned int nruns)
{
    // TODO
    // create OMPL planner specified problem translator
    vkc::OmplPlanParameters params; // use all default parameters for our beginning
    params.plan_params.n_steps = n_steps;
    params.inv_attp_max = 500;
    params.planner = OMPLPlanners::RRT_Connect;
    ProbTranslator prob_translator(params);
    ProbGenerator prob_generator;


    ofstream record_traj("opt_traj.txt");
    int j = 0;
    for (auto &action : actions)
    {
        ROS_WARN("optimization is ready. Press <Enter> to start next action");
        // if( 0 == action->getInitTraj().rows())
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


        PlannerResponse response;
        if(action->RequireInitTraj() && 0 == action->getInitTraj().rows())
        {
            // solve by OMPL to get an init trajectory
            std::vector<std::vector<double>> res_traj;
            prob_translator.transProb(env, action);

            prob_translator.solveProblem(response, res_traj);
            if (response.status_code)
            {
                ROS_INFO("OMPL planning plan successfully");
                action->setInitTrajectory(response.trajectory);
            }
            else
            {
                ROS_INFO("OMPL planning plan failed     ");
            }
        }

        ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseractEnvironment());

        unsigned int try_cnt = 0;
        bool converged = false;
        while (try_cnt++ < nruns)
        {
            TrajOptProb::Ptr prob_ptr = prob_generator.genProb(env, action, n_steps);
            CostInfo cost = solveProb_cost(prob_ptr, response, n_iter);

            if (sco::OptStatus::OPT_CONVERGED == response.status_code)
            {
                converged = true;
                break;
            }
            else
            {
                ROS_INFO("[%s]optimizationi could not converge, response code: %d, description: %s",
                         __func__, response.status_code, response.status_description.c_str());
            }
        }


        joint_trajs.emplace_back(TesseractJointTraj{response.joint_names, response.trajectory});

        // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.trajectory.leftCols(response.joint_names.size());
        // refineTrajectory(refined_traj);

        std::cout << "optimized trajectory: " << std::endl
                  << refined_traj << std::endl;

        record_traj << action << std::endl;
        record_traj << refined_traj << std::endl;

        plotter->plotTrajectory(response.joint_names, refined_traj);

        
        //ROS_WARN("optimization finished. Press <Enter> to start next action");
        // if(j > 12)
        //    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        env.updateEnv(response.joint_names, refined_traj.bottomRows(1).transpose(), action);
        plotter->clear();
        ++j;
    }

    record_traj.close();
}


void InitEnvState(VKCEnvBasic &env)
{
    // vector<string> base_joints({"base_y_base_x", "base_theta_base_y", "base_link_base_theta"});
    // vector<double> base_values({0, 0, 1.57});
    // env.getVKCEnv()->getTesseractEnvironment()->setState(base_joints, base_values);

    env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("Cabinet_417_link", "Cabinet_417_link_dof_rootd_Bb001_r", "Never");
    env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("right_arm_upper_arm_link", "right_arm_base_link_inertia", "Never");
    // env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("hook_1_link", "hook_1_head_link", "Adjacent");
}

int main(int argc, char **argv)
{
    srand((unsigned)time(NULL));  // for generating waypoint randomly motion planning 

    ros::init(argc, argv, "big_task_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ROS_INFO("Initializaing environment node...");
    bool plotting = true;
    bool rviz = true;
    int steps = 10;
    int n_iter = 1;
    int nruns = 1;
    int demo_index = 0;
    std::string robot{"vkc"};

    // Get ROS Parameters
    pnh.param<std::string>("robot", robot, robot);
    pnh.param("plotting", plotting, plotting);
    pnh.param("rviz", rviz, rviz);
    pnh.param<int>("steps", steps, steps);
    pnh.param<int>("niter", n_iter, n_iter);
    pnh.param<int>("nruns", nruns, nruns);
    pnh.param<int>("demo_index", demo_index, demo_index);

    ROS_INFO("[%s]Init information", __func__);
    ROS_INFO("\trobot: %s", robot.c_str());
    ROS_INFO("\tploting enabled: %s", plotting ? "true" : "false");
    ROS_INFO("\trviz enabled: %s", rviz ? "true" : "false");
    ROS_INFO("\tmax iteration: %d", n_iter);
    ROS_INFO("\ttry count: %d", nruns);

    UrdfSceneEnv::AttachObjectInfos attaches;
    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_1_body",
                                                         "Cup_1_link",
                                                         "Cup_1_link",
                                                         {0.0, 0.15, 0.150},     // Y-Axis is stright up
                                                         {0.707, -0.707, 0.0, 0.0}, // this gripper's y_axis is parallelling with the axis of gripper palm
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_1",
                                                         "Cup_1_link",
                                                         "Cup_1_link",
                                                         {0.0, 0.32, 0.0},     // Y-Axis is stright up
                                                         {0.0, 0.0, 0.0, -1.0}, // this gripper's y_axis is parallelling with the axis of gripper palm
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_2_body",
                                                         "Cup_2_link",
                                                         "Cup_2_link",
                                                         {0.0, 0.15, 0.150}, // Y-Axis is stright up
                                                         {0.707, -0.707, 0.0, 0.0}, //{0.707, -0.707, 0.0, 0.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_2_body_rotated",
                                                         "Cup_2_link",
                                                         "Cup_2_link",
                                                         {-0.15, 0.150, 0.075}, // Y-Axis is stright up
                                                         {0.612, -0.612, -0.354, -0.354}, //{0.707, -0.707, 0.0, 0.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_2",
                                                         "Cup_2_link",
                                                         "Cup_2_link",
                                                         {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.0, 0.0, 0.0, -1.0},
                                                         false});


    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3_body",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.0, 0.15, 0.150}, // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.707, -0.707, 0.0, 0.0},
                                                         false});


    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.0, 0.32, 0.0}, // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.0, 0.0, 0.0, -1.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3_front",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.0, 0.32, 0.08}, // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.0, 0.0, 0.0, -1.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3_left",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.10, 0.25, 0.0}, // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.0, -0.707, 0.0, -0.707},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3_right",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.1, 0.25, 0.1}, // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.0, 0.707, 0.0, -0.707},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3_with_tool",
                                                         "Cup_3_link",
                                                         "Cup_3_link",
                                                         {0.0, 0.20, -0.10},            // {0.0, 0.32, 0.0}, // Y-Axis is stright up
                                                         {0.086, -0.011, 0.130, 0.988}, // rx ry rz: -15 0.0 170
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_4_body",
                                                         "Cup_4_link",
                                                         "Cup_4_link",
                                                         {0.0, 0.12, 0.5},             // Y-Axis is stright up
                                                         {0.707, -0.707, 0.0, 0.0}, //   45.0 0.0 90.0  {0.707, -0.707, 0.0, -0.0}, //
                                                         false});
    
    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_4_body_rotated",
                                                         "Cup_4_link",
                                                         "Cup_4_link",
                                                         {-0.10, 0.12, 0.1},             // Y-Axis is stright up
                                                         {0.653, -0.653, -0.271, -0.271}, //   45.0 0.0 90.0  {0.707, -0.707, 0.0, -0.0}, //
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_4_with_tool",
                                                         "Cup_4_link",
                                                         "Cup_4_link",
                                                         {0.0, 0.20, 0.07},             // Y-Axis is stright up
                                                         {0.086, -0.011, 0.130, 0.988}, 
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_plate_1",
                                                         "Plate_1_link",
                                                         "Plate_1_link",
                                                         {0.0, 0.03, 0.66},     // Y-Axis is stright up
                                                         {0.707, -0.707, 0, 0},
                                                         false});


    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_chair_645",
                                                         "Chair_645_link",
                                                         "Chair_645_link",
                                                         {-0.55, 0.0, 0.35}, // Z-Axis is stright up, and the attach point at the outside of chair's right handrail
                                                         {0.707, 0.0, 0, -0.707},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet_417_door",
                                                         "Cabinet_417_link_dof_rootd_Bb001_r", // Z-Axis is stright up, and set the attach point is a little distatnce away to  of door knob
                                                         "Cabinet_417_link",
                                                         {0.26, -0.75, 0.0},
                                                         {0.707, 0, 0, 0.707},
                                                         true});

    UrdfSceneEnv::InverseChainsInfos inverse_chains;
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"Cabinet_417_link", "Cabinet_417_link_dof_rootd_Bb001_r"});

    UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
    InitEnvState(env);

    // cache the planning result for replaying
    vector<TesseractJointTraj> joint_trajs;

    ActionSeq actions;
    ROS_INFO("given demo index %d is %s, and the demo with index %d is going to run...",
             demo_index, (0 > demo_index || demo_index > 4) ? "invalid" : "valid", 
             (0 > demo_index || demo_index > 4) ? 2 : demo_index);
    switch(demo_index)
    {
    case 0:
        GenerateGraspCup3WithoutToolActions(actions, robot);
        break;
    case 1:
        GenerateActionsUnefficiency(actions, robot);
        break;
    case 2:
        GenerateOderedActionsByDistance(actions, robot);
        break;
    case 3:
        GenerateActionsGraspCup4WithPlate(actions, robot);
        break;
    case 4:
        GenerateRobotBigTaskActions(actions, robot);
        break;
    default:
        GenerateOderedActionsByDistance(actions, robot);
        break;
    }
    


    // plan motion trajectory according to given task actions
    Run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

    env.getVKCEnv()->getTesseractEnvironment()->setState(env.getHomePose());

    // visualize the trajectory as planned
    TrajectoryVisualize(env, actions, joint_trajs);
}