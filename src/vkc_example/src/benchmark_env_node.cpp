#include <ros/console.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/benchmark_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
// using namespace vkc_example;

using TesseractJointTraj = tesseract_common::JointTrajectory;

void run(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env,
         ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled,
         unsigned int nruns)
{
    int window_size = 3;
    // LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size);
    // seed_generator.generate(env, actions);
    ProbGenerator prob_generator;

    int j = 0;

    env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

    for (auto ptr = actions.begin(); ptr < actions.end(); ptr++)
    {
        auto action = *ptr;
        PlannerResponse response;
        unsigned int try_cnt = 0;
        bool converged = false;
        while (try_cnt++ < nruns)
        {
            auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

            env.getPlotter()->waitForInput(
                "optimization is ready. Press <Enter> to process the request.");

            // CostInfo cost = solveProb(prob_ptr, response, n_iter);
            solveProb(prob_ptr, response, n_iter);

            // break;
            if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
                response.status.value()) // optimization converges
            {
                converged = true;
                break;
            }
            else
            {
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

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        tesseract_common::JointTrajectory refined_traj = trajectory;
        refineTrajectory(refined_traj, env);
        joint_trajs.emplace_back(trajectory);

        // ROS_WARN("trajectory: ");
        // for (auto jo : trajectory) {
        //   std::cout << jo.position << std::endl;
        // }

        // refine the orientation of the move base

        // tesseract_common::TrajArray trajectory =
        //     response.trajectory.leftCols(response.joint_names.size());
        // refineTrajectory(trajectory);

        // std::cout << "optimized trajectory: " << std::endl
        //           << trajectory << std::endl;
        if (env.getPlotter() != nullptr)
        {
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

        // toDelimitedFile(ci,
        //                 "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/"
        //                 "trajectory/open_door_pull.csv",
        //                 ',');
        // saveTrajToFile(trajectory,
        // "/home/jiao/BIGAI/vkc_ws/ARoMa/applications/vkc-planning/trajectory/open_door_pull.csv");

        env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                      action);

        // for (auto joint_name : env.getVKCEnv()->getTesseract()->getActiveJointNames())
        // {
        //     std::cout << joint_name << std::endl;
        // }

        // std::cout << env.getVKCEnv()->getTesseract()->getCurrentJointValues() << std::endl;

        // ROS_WARN("environment updated");
        if (env.getPlotter() != nullptr)
            env.getPlotter()->clear();
        ++j;
    }
}

void run_baseline(vector<TesseractJointTraj> &joint_trajs, VKCEnvBasic &env,
                  ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled,
                  unsigned int nruns)
{
    ProbGenerator prob_generator;

    int j = 0;

    env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

    for (auto ptr = actions.begin(); ptr < actions.end(); ptr++)
    {
        auto action = *ptr;
        PlannerResponse response;
        unsigned int try_cnt = 0;
        bool converged = false;
        while (try_cnt++ < nruns)
        {
            auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

            env.getPlotter()->waitForInput(
                "optimization is ready. Press <Enter> to process the request.");

            solveProb(prob_ptr, response, n_iter);

            if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
                response.status.value()) // optimization converges
            {
                converged = true;
                break;
            }
            else
            {
                ROS_WARN(
                    "[%s]optimization could not converge, response code: %d, "
                    "description: %s",
                    __func__, response.status.value(),
                    response.status.message().c_str());
                ActionSeq sub_actions(ptr, actions.end());
            }
        }

        const auto &ci = response.results;

        tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
        tesseract_common::JointTrajectory refined_traj = trajectory;
        refineTrajectory(refined_traj, env);
        joint_trajs.emplace_back(trajectory);

        if (env.getPlotter() != nullptr)
        {
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

        if (env.getPlotter() != nullptr)
            env.getPlotter()->clear();
        ++j;
    }
}

void pullDoor(vkc::ActionSeq &actions, const std::string &robot)
{
    PlaceAction::Ptr place_action;

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

        joint_objectives.emplace_back("door_north_door_joint", M_PI_2);
        place_action =
            make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                     link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }
}

void pushDoor(vkc::ActionSeq &actions, const std::string &robot)
{
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

        joint_objectives.emplace_back("door_north_door_joint", -M_PI_2);
        place_action =
            make_shared<PlaceAction>(robot, "attach_door_north_handle_link",
                                     link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }

    ROS_INFO("push door actions created.");
}

void pullDrawer(vkc::ActionSeq &actions, const std::string &robot)
{
    PlaceAction::Ptr place_action;

    /** open drawer **/
    // action 1: pick the drawer handle
    {
        actions.emplace_back(
            make_shared<PickAction>(robot, "attach_drawer0_handle_link"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 2: open drawer
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer0_base_drawer_joint", -0.8);
        place_action =
            make_shared<PlaceAction>(robot, "attach_drawer0_handle_link",
                                     link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }
}

void pushDrawer(vkc::ActionSeq &actions, const std::string &robot)
{
    PlaceAction::Ptr place_action;

    /** open drawer **/
    // action 1: pick the drawer handle
    {
        actions.emplace_back(
            make_shared<PickAction>(robot, "attach_drawer0_handle_link"));
        (*actions.rbegin())->RequireInitTraj(false);
    }

    // action 2: open drawer
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        joint_objectives.emplace_back("drawer0_base_drawer_joint", 0.0);
        place_action =
            make_shared<PlaceAction>(robot, "attach_drawer0_handle_link",
                                     link_objectives, joint_objectives, false);
        place_action->setOperationObjectType(false);

        place_action->RequireInitTraj(true);

        actions.emplace_back(place_action);
    }
}

void baseline_reach(vkc::ActionSeq &actions, const std::string &robot, Eigen::VectorXd base_pose, Eigen::Isometry3d ee_pose)
{

    /** move base **/
    // action 1: move base to target
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        Eigen::Isometry3d tf;
        tf.setIdentity();
        tf.translation() += Eigen::Vector3d(base_pose[0], base_pose[1], 0.145);
        tf.linear() = Eigen::Quaterniond(1., 0., 0., 0.).matrix();

        link_objectives.emplace_back("base_link", tf);

        actions.emplace_back(
            make_shared<GotoAction>("base", link_objectives, joint_objectives));
    }

    // action 2: move arm to target
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        link_objectives.emplace_back("robotiq_arg2f_base_link", ee_pose);

        actions.emplace_back(
            make_shared<GotoAction>("arm", link_objectives, joint_objectives));
    }
}

void moveBase(vkc::ActionSeq &actions, const std::string &robot, Eigen::VectorXd base_pose)
{

    /** move base **/
    // action 1: move base to target
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        Eigen::Isometry3d tf;
        tf.setIdentity();
        tf.translation() += Eigen::Vector3d(base_pose[0], base_pose[1], 0.145);
        tf.linear() = Eigen::Quaterniond(1., 0., 0., 0.).matrix();

        link_objectives.emplace_back("base_link", tf);

        actions.emplace_back(
            make_shared<GotoAction>("base", link_objectives, joint_objectives));
    }
}

void moveArm(vkc::ActionSeq &actions, const std::string &robot, Eigen::Isometry3d ee_pose)
{

    /** move arm **/
    // action 1: move arm to target
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;

        link_objectives.emplace_back("robotiq_arg2f_base_link", ee_pose);

        actions.emplace_back(
            make_shared<GotoAction>("arm", link_objectives, joint_objectives));
    }
}

Eigen::VectorXd sampleBasePose(vkc::VKCEnvBasic &env, Eigen::Isometry3d ee_goal, double offset)
{
    bool init_base_position = false;
    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    Eigen::VectorXd base_values = Eigen::Vector2d(0, 0);
    Eigen::VectorXd init_base_values = Eigen::Vector2d(0, 0);
    init_base_values[0] = env.getVKCEnv()->getTesseract()->getCurrentJointValues()[0];
    init_base_values[1] = env.getVKCEnv()->getTesseract()->getCurrentJointValues()[1];
    tesseract_collision::ContactResultMap contact_results;

    while (!init_base_position)
    {
        init_base_position = true;

        double r = (rand() % 1000) / 1000.0 * 0.8;
        double a = (rand() % 1000) / 1000.0 * 3.14 + offset;

        base_values[0] = ee_goal.translation()[0] - r * sin(a);
        base_values[1] = ee_goal.translation()[1] + r * cos(a);

        env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
        env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
            contact_results, tesseract_collision::ContactTestType::ALL);
        for (auto &collision : contact_results)
        {
            // std::cout << collision.first.first << " and " << collision.first.second << " are in collision." << std::endl;

            if (collision.first.first == "base_link" || collision.first.second == "base_link")
            {
                init_base_position = false;
                break;
            }
        }
        contact_results.clear();
    }

    env.getVKCEnv()->getTesseract()->setState(base_joints, init_base_values);

    return base_values;
}

int main(int argc, char **argv)
{
    srand(time(NULL));

    console_bridge::setLogLevel(
        console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

    ros::init(argc, argv, "open_door_env_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    ROS_INFO("Initializaing environment node...");

    bool plotting = true;
    bool rviz = true;
    int steps = 30;
    int n_iter = 1000;
    int nruns = 5;
    int envid = 1;
    std::string robot{"vkc"};

    // Get ROS Parameters
    pnh.param<std::string>("robot", robot, robot);
    pnh.param("plotting", plotting, plotting);
    pnh.param("rviz", rviz, rviz);
    pnh.param<int>("steps", steps, steps);
    pnh.param<int>("niter", n_iter, n_iter);
    pnh.param<int>("nruns", nruns, nruns);
    pnh.param<int>("envid", envid, envid);

    BenchmarkEnv env(nh, plotting, rviz, steps, envid);

    vector<TesseractJointTraj> joint_trajs;

    Eigen::Isometry3d door_handle_pose_close;
    door_handle_pose_close.setIdentity();
    door_handle_pose_close.translation() += Eigen::Vector3d(4.725, -0.45, 0.95);
    door_handle_pose_close.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

    Eigen::Isometry3d door_handle_pose_push;
    door_handle_pose_push.setIdentity();
    door_handle_pose_push.translation() += Eigen::Vector3d(5.97977, 0.254997, 0.94999);
    door_handle_pose_push.linear() = Eigen::Quaterniond(0.0000, 0.0000, 0.707106781186548, 0.707106781186548).matrix();

    Eigen::Isometry3d door_handle_pose_pull;
    door_handle_pose_pull.setIdentity();
    door_handle_pose_pull.translation() += Eigen::Vector3d(4.02, 0.804998, 0.95);
    door_handle_pose_pull.linear() = Eigen::Quaterniond(0.707106781186548, 0.707106781186548, 0.0000, 0.0000).matrix();

    Eigen::Isometry3d drawer_handle_pose_close;
    drawer_handle_pose_close.setIdentity();
    drawer_handle_pose_close.translation() += Eigen::Vector3d(3.28, 2.5, 0.9);
    drawer_handle_pose_close.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

    Eigen::Isometry3d drawer_handle_pose_open;
    drawer_handle_pose_open.setIdentity();
    drawer_handle_pose_open.translation() += Eigen::Vector3d(2.48001, 2.49998, 0.899996);
    drawer_handle_pose_open.linear() = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();

    ActionSeq actions;
    // pullDoor(actions, robot);
    // pushDoor(actions, robot);
    // pullDrawer(actions, robot);
    // pushDrawer(actions, robot);
    // run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

    // Eigen::Isometry3d pick_pose_world_transform =
    //   env.getVKCEnv()->getTesseract()->getLinkTransform(
    //       attach_location_ptr->link_name_) *
    //   attach_location_ptr->local_joint_origin_transform;

    {
        // open door baseline 1
        std::unordered_map<std::string, double> door_joint_target;
        door_joint_target["door_north_door_joint"] = M_PI_2;
        std::unordered_map<std::string, double> door_joint_init;
        door_joint_init["door_north_door_joint"] = 0.0;

        baseline_reach(actions, robot, sampleBasePose(env, door_handle_pose_close, 0), door_handle_pose_close);
        run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

        env.getVKCEnv()->getTesseract()->setState(door_joint_target);

        actions.clear();
        moveBase(actions, robot, sampleBasePose(env, door_handle_pose_push, M_PI_2));
        run(joint_trajs, env, actions, steps, n_iter, rviz, nruns);

        for (int i = 0; i < steps; i++)
        {
            actions.clear();
        }
    }
}