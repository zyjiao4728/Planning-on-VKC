#include "vkc/env/urdf_scene_env.h"
#include "vkc/env/vkc_env_basic.h"
#include "vkc/planner/prob_generator.h"
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
    params.inv_attp_max = 1000;
    params.planner = OMPLPlanners::RRT_Star;
    // ProbTranslator prob_translator(params);

    ProbGenerator prob_generator;
    for (auto &action : actions)
    {

        PlannerResponse response;
        if(action->RequireInitTraj())
        {
            // // solve by OMPL to get an init trajectory
            // std::vector<std::vector<double>> res_traj;
            // prob_translator.transProb(env, action);

            // prob_translator.solveProblem(response, res_traj);
            // if (response.status_code)
            // {
            //     ROS_INFO("OMPL planning plan successfully");
            //     action->setInitTrajectory(response.trajectory);
            // }
            // else
            // {
            //     ROS_INFO("OMPL planning plan failed     ");
            // }
        }

        ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseractEnvironment());

        int try_cnt = 0;
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
        }

        if (!converged)
        {
            ROS_INFO("[%s]optimizationi could not converge", __func__);
            //return;
        }

        
        joint_trajs.emplace_back(TesseractJointTraj{response.joint_names, response.trajectory});

        // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.trajectory.leftCols(response.joint_names.size());
        // refineTrajectory(refined_traj);

        std::cout << "optimized trajectory: " << std::endl
                  << refined_traj << std::endl;

        plotter->plotTrajectory(response.joint_names, refined_traj);

        ROS_WARN("optimization finished. Press <Enter> to start next action");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        env.updateEnv(response.joint_names, refined_traj.bottomRows(1).transpose(), action);
        plotter->clear();
    }
}

void GenerateActions(ActionSeq &actions, const std::string& robot)
{
    // TODO
    Eigen::Isometry3d dst_tf;
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    PlaceAction::Ptr place_action;

 
    // // action: pick up cup_1
    // actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_1"));
    // (*actions.rbegin())->RequireInitTraj(true);

    // /** pick up cup_1 and place it onto plate_1 **/
    // // action: place cup_1 onto plate_1
    // link_objectives.clear();
    // dst_tf.translation() = Eigen::Vector3d(-0.20, 3.48, 0.8);
    // dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

    // link_objectives.emplace_back("Cup_1_link", dst_tf);
    // place_action = make_shared<PlaceAction>(robot, "attach_cup_1", link_objectives, joint_objectives, false);
    // place_action->setNewAttachObject("Plate_1_link");
    // place_action->RequireInitTraj(true);
    // actions.emplace_back(place_action);



    // /** pick up cup_2 and place it onto plate_1 **/
    // // action 3: pick up cup_2
    // actions.emplace_back(make_shared<PickAction>(robot, "attach_cup_2"));
    // (*actions.rbegin())->RequireInitTraj(true);

    // // action 4: place cup_2 onto plate_1
    // link_objectives.clear();
    // dst_tf.translation() = Eigen::Vector3d(-0.19, 3.34, 0.8);
    // // dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();
    // dst_tf.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

    // link_objectives.emplace_back("Cup_2_link", dst_tf);
    // place_action = make_shared<PlaceAction>(robot, "attach_cup_2", link_objectives, joint_objectives, false);
    // place_action->setNewAttachObject("Plate_1_link");
    // place_action->RequireInitTraj(true);
    // actions.emplace_back(place_action);


 
    /** pick up tape_1 and place it onto plate_1 **/
    // action 5: pick up tape_1
    actions.emplace_back(make_shared<PickAction>(robot, "attach_tape_1"));
    (*actions.rbegin())->RequireInitTraj(true);


    // action 6: place tape_1 onto plate_1
    link_objectives.clear();
    dst_tf.translation() = Eigen::Vector3d(-0.069, 3.47, 0.95);
    // dst_tf.linear() = Eigen::Quaterniond(0.0, 0.0, 0.0, -1.0).matrix();   // cylindrical surface attached to plate
    dst_tf.linear() = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0).matrix();   // cylindrical surface attached to plate

    link_objectives.emplace_back("Tape_1_link", dst_tf);
    place_action = make_shared<PlaceAction>(robot, "attach_tape_1", link_objectives, joint_objectives, false);
    place_action->setNewAttachObject("Plate_1_link");
    place_action->RequireInitTraj(true);
    actions.emplace_back(place_action);

    /** pick up stick_1 and then use it to pick up rubik_cube_1 and place onto plate_1 **/
    // action 7: pick up stick_1
    actions.emplace_back(make_shared<PickAction>(robot, "attach_stick_1"));
    (*actions.rbegin())->RequireInitTraj(true);


    // action 8: pick up rubik_cube_1
    actions.emplace_back(make_shared<PickAction>(robot, "attach_rubik_cube_1"));
    (*actions.rbegin())->RequireInitTraj(true);

    // action 9: place rubik_cube_1 onto plate_1
    link_objectives.clear();
    dst_tf.translation() = Eigen::Vector3d(-0.07, 3.345, 0.85);
    dst_tf.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();  // 

    link_objectives.emplace_back("Rubik_cube_1_link", dst_tf);
    place_action = make_shared<PlaceAction>(robot, "attach_rubik_cube_1", link_objectives, joint_objectives, false);
    place_action->setNewAttachObject("Plate_1_link");
    place_action->RequireInitTraj(true);
    actions.emplace_back(place_action);


    // action 10: place stick_1 onto plate_1
    link_objectives.clear();
    dst_tf.translation() = Eigen::Vector3d(-0.02, 3.7, 0.95);
    dst_tf.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();
    // dst_tf.linear() = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5).matrix();

    link_objectives.emplace_back("Stick_1_link", dst_tf);
    place_action = make_shared<PlaceAction>(robot, "attach_stick_1", link_objectives, joint_objectives, false);
    place_action->setNewAttachObject("Plate_1_link");
    place_action->RequireInitTraj(true);
    actions.emplace_back(place_action);



    // action 9: pick plate_1
    actions.emplace_back(make_shared<PickAction>(robot, "attach_plate_1"));
    (*actions.rbegin())->RequireInitTraj(true);


    // action 10: place plate_1 and things on it onto table_5
    link_objectives.clear();
    dst_tf.translation() = Eigen::Vector3d(0.45, 3.4, 0.80);
    dst_tf.linear() = Eigen::Quaterniond(0.707107, 0.707107, 0.0, 0.0).matrix();

    link_objectives.emplace_back("Plate_1_link", dst_tf);
    place_action = make_shared<PlaceAction>(robot, "attach_plate_1", link_objectives, joint_objectives, false);
    place_action->setNewAttachObject("Table_5_link");
    place_action->RequireInitTraj(true);
    actions.emplace_back(place_action);


    /** open cabinet_417 and then place plate_1 into it **/
    // // action 11: pick plate_1
    // actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_417_door"));
    // (*actions.rbegin())->RequireInitTraj(true);

    // // action 12: pick plate_1
    // link_objectives.clear();
    // joint_objectives.clear();
    // joint_objectives.emplace_back("Cabinet_417_link_dof_rootd_Bb001_r_joint", 1.5);
    // place_action = make_shared<PlaceAction>(robot, "attach_cabinet_417_door", link_objectives, joint_objectives, false);
    // //place_action->setNewAttachObject("Table_5_link");
    // place_action->RequireInitTraj(false);
    // place_action->setOperationObjectType(false);
    // actions.emplace_back(place_action);



    /** move away chair which blocks the robot to pick something **/
    // // action 13: pick plate_1
    // actions.emplace_back(make_shared<PickAction>(robot, "attach_chair_645"));
    // (*actions.rbegin())->RequireInitTraj(true);

    // // action 14: place plate_1 and things on it onto table_5
    // link_objectives.clear();
    // dst_tf.translation() = Eigen::Vector3d(-1.5, 1.0, 0.54);
    // dst_tf.linear() = Eigen::Quaterniond(1.0, 0, 0.0, 0.0).matrix();

    // link_objectives.emplace_back("Chair_645_link", dst_tf);
    // place_action = make_shared<PlaceAction>(robot, "attach_chair_645", link_objectives, joint_objectives, false);
    // place_action->setNewAttachObject("world");
    // place_action->RequireInitTraj(true);
    // actions.emplace_back(place_action);
}

void InitEnvState(VKCEnvBasic &env)
{
    // vector<string> base_joints({"base_y_base_x", "base_theta_base_y", "base_link_base_theta"});
    // vector<double> base_values({0, 0, 1.57});
    // env.getVKCEnv()->getTesseractEnvironment()->setState(base_joints, base_values);
    
    //env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("Cabinet_417_link", "Cabinet_417_link_dof_rootd_Bb001_r", "Never");
    //env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("hook_1_link", "hook_1_head_link", "Adjacent");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bigai_tamp_node");
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
    std::string robot{"vkc"};

    // Get ROS Parameters
    pnh.param<int>("robot", nruns, nruns);
    pnh.param("plotting", plotting, plotting);
    pnh.param("rviz", rviz, rviz);
    pnh.param<int>("steps", steps, steps);
    pnh.param<int>("niter", n_iter, n_iter);
    pnh.param<int>("nruns", nruns, nruns);

    ROS_INFO("[%s]Init information", __func__);
    ROS_INFO("\trobot: %s",  robot.c_str());
    ROS_INFO("\tploting enabled: %s", plotting ? "true" : "false");
    ROS_INFO("\trviz enabled: %s",  rviz ? "true" : "false");
    ROS_INFO("\tmax iteration: %d", n_iter);
    ROS_INFO("\ttry count: %d", nruns);

    UrdfSceneEnv::AttachObjectInfos attaches;
    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_1",
                                                         "Cup_1_link",
                                                         "Cup_1_link",
                                                         {0.0, 0.33, 0.0},  // joint origin is {1.18 -0.3 0.445}, box height is ${scale} 0.1
                                                         {0.0, 0.0, 0.0, 1.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_2",
                                                         "Cup_2_link",
                                                         "Cup_2_link",
                                                         {-0, 0.33, 0.0}, // joint origin is {1.18 -0.3 0.445}, box height is ${scale} 0.1
                                                         {0.0, 0.0, 0.0, 1.0},
                                                         false});

    // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_3",
    //                                                      "Cup_3_link",
    //                                                      "Cup_3_link",
    //                                                      {-0, 0.3, 0.0}, // joint origin is {1.18 -0.3 0.445}, box height is ${scale} 0.1
    //                                                      {1, 0, 0, .0},
    //                                                      false});

    // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cup_4",
    //                                                      "Cup_4_link",
    //                                                      "Cup_4_link",
    //                                                      {-0, 0.3, 0.0}, // joint origin is {1.18 -0.3 0.445}, box height is ${scale} 0.1
    //                                                      {1, 0, 0, .0},
    //                                                      false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_tape_1",
                                                         "Tape_1_link",
                                                         "Tape_1_link",
                                                         {-0, 0.0, 0.25}, // joint origin is {1.18 -0.3 0.445}, box height is ${scale} 0.1
                                                         {0.707, -0.707, 0.0, 0.0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_rubik_cube_1",
                                                         "Rubik_cube_1_link",
                                                         "Rubik_cube_1_link",
                                                         {0.0, 0.0, 0.2}, // joint origin is {1.18 -0.3 0.445}, plate height is ${scale} 0.1
                                                         {0.707, 0, 0.707, 0},
                                                         false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_plate_1",
                                                         "Plate_1_link",
                                                         "Plate_1_link",
                                                         {0.0, 0.40, 0.25}, // joint origin is {1.18 -0.3 0.445}, plate height is ${scale} 0.1
                                                         {0.0, 0, 0, 1.0},
                                                         false});

    // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_trash_can_1",
    //                                                      "Trash_can_1_link",
    //                                                      "Trash_can_1_link",
    //                                                      {0.0, -55.0, 0.0}, // joint origin is {1.18 -0.3 0.445}, plate height is ${scale} 0.1
    //                                                      {1, 0, 0, 0},
    //                                                      false});

    // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_bucket_1",
    //                                                      "Bucket_1_link",
    //                                                      "Bucket_1_link",
    //                                                      {0.0, -55.0, 0.0}, // joint origin is {1.18 -0.3 0.445}, plate height is ${scale} 0.1
    //                                                      {1, 0, 0, 0},
    //                                                      false});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_stick_1",
                                                         "Stick_1_link",
                                                         "Stick_1_link",
                                                         {-0.15, 0.0, -0.7}, 
                                                         {0.707, 0, 0, -0.707},
                                                         false});


    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_chair_645",
                                                         "Chair_645_link",
                                                         "Chair_645_link",
                                                         {-0.5, 0.0, 0.4}, 
                                                         {1.0, 0.0, 0, 0.0},
                                                         true});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet_417_door",
                                                         "Cabinet_417_link_dof_rootd_Bb001_r",
                                                         "Cabinet_417_link",
                                                         {0.2, -0.45, 0.0}, 
                                                         {0.707, 0, 0, 0.707},
                                                         true});


    UrdfSceneEnv::InverseChainsInfos inverse_chains;
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"Cabinet_417_link", "Cabinet_417_link_dof_rootd_Bb001_r"});



    UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
    InitEnvState(env);

    // cache the planning result for replaying
    vector<TesseractJointTraj> joint_trajs;

    ActionSeq actions;
    GenerateActions(actions, robot);

    // plan motion trajectory according to given task actions
    Run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);


    // visualize the trajectory as planned
    TrajectoryVisualize(env, actions, joint_trajs);
}