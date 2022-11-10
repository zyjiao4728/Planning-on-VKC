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
    vkc::OmplPlanParameters params; // use all default parameters for our beginning
    params.plan_params.n_steps = n_steps;
    params.inv_attp_max = 1000;
    params.planner = OMPLPlanners::RRT_Star;
    ProbTranslator prob_translator(params);
    ProbGenerator prob_generator;

    ofstream record_traj("pog_exp_traj_close.txt");
    int j = 0;
    ROS_WARN("optimization is ready. Press <Enter> to start next action");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    for (auto &action : actions)
    {
        PlannerResponse response;
        // if (action->RequireInitTraj() && 0 == action->getInitTraj().rows())
        // {
        //     // solve by OMPL to get an init trajectory
        //     std::vector<std::vector<double>> res_traj;
        //     prob_translator.transProb(env, action);

        //     prob_translator.solveProblem(response, res_traj);
        //     if (response.status_code)
        //     {
        //         ROS_INFO("OMPL planning plan successfully");
        //         action->setInitTrajectory(response.trajectory);
        //     }
        //     else
        //     {
        //         ROS_INFO("OMPL planning plan failed     ");
        //     }
        // }

        ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract());

        unsigned int try_cnt = 0;
        bool converged = false;
        while (try_cnt++ < nruns)
        {
            TrajOptProb::Ptr prob_ptr = prob_generator.genRequest(env, action, n_steps);
            CostInfo cost = solveProb_cost(prob_ptr, response, n_iter);

            if (sco::OptStatus::OPT_CONVERGED == response.status_code)
            {
                converged = true;
                break;
            }
            else
            {
                ROS_WARN("[%s]optimizationi could not converge, response code: %d, description: %s",
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

        ROS_WARN("Finished optimization. Press <Enter> to start next action");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        env.updateEnv(response.joint_names, refined_traj.bottomRows(1).transpose(), action);
        plotter->clear();
        ++j;
    }

    record_traj.close();
}

void InitEnvState(VKCEnvBasic &env)
{
    // vector<string> base_joints({"base_y_base_x", "base_theta_base_y", "base_link_base_theta"});
    // vector<double> base_values({-1, -1, 0});
    // env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);

    // vector<string> furniture_joints({"wardrobe_joint6", "cabinet_joint6", "drawer_joint2"});
    // vector<double> furniture_values({0.72, 1.7, -0.49});
    // env.getVKCEnv()->getTesseractEnvironment()->setState(furniture_joints, furniture_values);

    // env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("Cabinet_417_link", "Cabinet_417_link_dof_rootd_Bb001_r", "Never");
    // env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("right_arm_upper_arm_link", "right_arm_base_link_inertia", "Never");
    // env.getVKCEnv()->getTesseractEnvironment()->addAllowedCollision("hook_1_link", "hook_1_head_link", "Adjacent");
}

/**
 * @brief for pog demo, open three containers: wardrobe, cabinet and drawer.
 *
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void OpenContainers(vkc::ActionSeq &actions, const std::string &robot)
{
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const bool enable_init_traj = true;

    /** open wardrobe **/
    // action 1: pick the handle of wardrobe
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_wardrobe_handle"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 2: open door of wardrobe
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("wardrobe_joint6", 0.72);
        place_action = make_shared<PlaceAction>(robot, "attach_wardrobe_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(false);

        actions.emplace_back(place_action);
    }

    /** open cabinet **/
    // action 3: pick the handle of cabinet
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_handle"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 4: open door of cabinet
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("cabinet_joint6", 1.7);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** go back a little bit **/
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        // joint_objectives.emplace_back("base_y_base_x", 0.3);
        // joint_objectives.emplace_back("base_theta_base_y", -0.3);
        // joint_objectives.emplace_back("ur_arm_elbow_joint", -2.2043);
        // joint_objectives.emplace_back("ur_arm_shoulder_lift_joint", -0.8678);
        // joint_objectives.emplace_back("ur_arm_shoulder_pan_joint", 1.57);
        // joint_objectives.emplace_back("ur_arm_wrist_1_joint", -0.0347);
        // joint_objectives.emplace_back("ur_arm_wrist_2_joint", 1.6315);
        // joint_objectives.emplace_back("ur_arm_wrist_3_joint", -2.2043);

        dst_tf.translation() = Eigen::Vector3d(0.5, -2.5, 1.0);
        dst_tf.linear() = Eigen::Quaterniond(0.7071, 0, 0, 0.7071).matrix();
        link_objectives.push_back(LinkDesiredPose("ur_arm_tool0", dst_tf));

        actions.emplace_back(make_shared<GotoAction>(robot, link_objectives, joint_objectives));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    /** open drawer **/
    // action 5: pick the handle of drawer
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_drawer_handle"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 6: open door of drawer
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer_joint2", -0.49);
        place_action = make_shared<PlaceAction>(robot, "attach_drawer_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(false);

        actions.emplace_back(place_action);
    }
}

/**
 * @brief for pog demo, close three containers: drawer, cabinet and wardrobe.
 *
 * @param actions output parameter, which carries the generated actions for motion planning
 * @param robot input parameter, which specifies the robot who will act the actions
 * @return
 */
void CloseContainers(vkc::ActionSeq &actions, const std::string &robot)
{
    // TODO
    Eigen::Isometry3d dst_tf;
    PlaceAction::Ptr place_action;
    const bool enable_init_traj = true;

    /** close drawer **/
    // action 1: pick the handle of drawer
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_drawer_handle"));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    // action 2: close door of drawer
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer_joint2", -0.49);
        place_action = make_shared<PlaceAction>(robot, "attach_drawer_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(false);

        actions.emplace_back(place_action);
    }

    /** go back a little bit **/
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        // joint_objectives.emplace_back("base_y_base_x", 0.3);
        // joint_objectives.emplace_back("base_theta_base_y", -0.3);
        // joint_objectives.emplace_back("ur_arm_elbow_joint", -2.2043);
        // joint_objectives.emplace_back("ur_arm_shoulder_lift_joint", -0.8678);
        // joint_objectives.emplace_back("ur_arm_shoulder_pan_joint", 1.57);
        // joint_objectives.emplace_back("ur_arm_wrist_1_joint", -0.0347);
        // joint_objectives.emplace_back("ur_arm_wrist_2_joint", 1.6315);
        // joint_objectives.emplace_back("ur_arm_wrist_3_joint", -2.2043);

        dst_tf.translation() = Eigen::Vector3d(0.5, -2.5, 1.0);
        dst_tf.linear() = Eigen::Quaterniond(0.7071, 0, 0, 0.7071).matrix();
        link_objectives.push_back(LinkDesiredPose("ur_arm_tool0", dst_tf));

        actions.emplace_back(make_shared<GotoAction>(robot, link_objectives, joint_objectives));
        (*actions.rbegin())->RequireInitTraj(true);
    }

    /** open cabinet **/
    // action 3: pick the handle of cabinet
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_cabinet_handle"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 4: open door of cabinet
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("cabinet_joint6", 1.7);
        place_action = make_shared<PlaceAction>(robot, "attach_cabinet_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    /** close wardrobe **/
    // action 5: pick the handle of wardrobe
    {
        actions.emplace_back(make_shared<PickAction>(robot, "attach_wardrobe_handle"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 6: open door of wardrobe
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("wardrobe_joint6", 0.72);
        place_action = make_shared<PlaceAction>(robot, "attach_wardrobe_handle", link_objectives, joint_objectives);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(false);

        actions.emplace_back(place_action);
    }
}

int main(int argc, char **argv)
{
    srand((unsigned)time(NULL)); // for generating waypoint randomly motion planning

    ros::init(argc, argv, "pog_env_node");
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
    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_wardrobe_handle",
                                                         "wardrobe_link8",
                                                         "wardrobe",
                                                         {0.0, 0.0, 0.0},  // Y-Axis is stright up
                                                         {1, 0, 0.0, 0.0}, // this gripper's y_axis is parallelling with the axis of gripper palm
                                                         true});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_cabinet_handle",
                                                         "cabinet_link7",
                                                         "cabinet",
                                                         {0.0, 0.0, 0.0},  // Y-Axis is stright up
                                                         {1, 0, 0.0, 0.0}, // this gripper's y_axis is parallelling with the axis of gripper palm
                                                         true});

    attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{"attach_drawer_handle",
                                                         "drawer_link6",
                                                         "drawer",
                                                         {0.0, 0.0, 0.0},  // Y-Axis is stright up
                                                         {1, 0, 0.0, 0.0}, // this gripper's y_axis is parallelling with the axis of gripper palm
                                                         true});

    UrdfSceneEnv::InverseChainsInfos inverse_chains;
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"wardrobe", "wardrobe_link8"});
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"cabinet", "cabinet_link7"});
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{"drawer", "drawer_link6"});

    UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
    InitEnvState(env);

    // cache the planning result for replaying
    vector<TesseractJointTraj> joint_trajs;

    ActionSeq actions;

    OpenContainers(actions, robot);

    // plan motion trajectory according to given task actions
    Run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

    env.getVKCEnv()->getTesseractEnvironment()->setState(env.getHomePose());

    // visualize the trajectory as planned
    TrajectoryVisualize(env, actions, joint_trajs);
}