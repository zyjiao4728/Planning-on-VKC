// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for parsing task plan file
#include "vkc/action/TaskPlanParser.h"


#include "vkc/env/arena_env.h"
#include "vkc/env/vkc_env_basic.h"
#include "vkc/planner/prob_generator.h"
#include "vkc_example/utils.h"
#include "vkc/action/actions.h"

#include <ros/package.h>
#include <iomanip>



using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;
// using namespace vkc_example;

static SceneObjects &GetSceneObjects()
{
    static SceneObjects scene_objects = SceneObjects();
    return scene_objects;
}



void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled, int nruns,
         vector<tesseract_common::JointTrajectory>& joint_trajs)
{
    ProbGenerator prob_generator;
    // ROSPlottingPtr plotter;

    CostInfo cost;


    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    vector<double> base_values({0, 0});

    env.getVKCEnv()->getTesseract()->getEnvironment()->setState(base_joints, base_values);

    int env_revision = 0;
    env_revision = env.getVKCEnv()->getTesseract()->getEnvironment()->getRevision();

    int i = 0;
    vkc::ActionBase::Ptr pre_act = nullptr;
    for (auto &action : actions)
    {
        // switch(action->getActionType())
        // {
        //     case vkc::ActionType::PlaceAction:
                // if(pre_act && ActionType::PickAction == pre_act->getActionType())
                // {
                //     env.attachObject(std::dynamic_pointer_cast<PickAction>(pre_act)->getAttachedObject());

                //     // Now update rviz environment
                //     if (!env.sendRvizChanges_(env_revision))
                //         return;

                //     env.getVKCEnv()->getTesseract()->getEnvironment()->setState(joint_trajs.back().joint_names,
                //                          joint_trajs.back().trajectory.bottomRows(1).transpose());

                //     env_revision = env.getVKCEnv()->getTesseract()->getEnvironment()->getRevision();
                // }


        //     break;
        // }


        PlannerResponse response;
        TrajOptProb::Ptr prob_ptr = nullptr;
        // plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

        bool converged = false;
        int tries = 0;
        std::chrono::duration<double> elapsed_seconds;

        std::cout << __func__ << ": " << action << std::endl;
        while (!converged && tries < 1)
        {
            prob_ptr = prob_generator.genProb(env, action, n_steps);

            // 6: is half size of the map 
            bool on_map_border = (abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(0)) - 6) < 1e-6 
            || abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(1)) - 6) < 1e-6);
            if (on_map_border && string::npos == env.getEndEffectorLink().find("cabinet"))
            {
                //std::cout << prob_ptr->GetInitTraj().bottomRows(1)(0) << " " << prob_ptr->GetInitTraj().bottomRows(1)(1) << std::endl;
                if (tries < 4)
                {
                    ROS_INFO("unfornately got here before optimizing the trajectory!");
                    continue;
                }
            }

            // ROS_WARN("motion planning for action: %d", i);
            // ROS_WARN("solve problem has tried %d times", tries);
            // if (rviz_enabled)
            // {
            //     ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
            //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // }

            auto start = std::chrono::system_clock::now();
            cost = solveProb_cost(prob_ptr, response, n_iter, false);
            elapsed_seconds = std::chrono::system_clock::now() - start;
            
            converged = sco::OptStatus::OPT_CONVERGED == response.status.value();


            // update try counter
            tries += 1;
        }

        
        // record planning result
        joint_trajs.push_back(response.joint_trajectory);

        // // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.joint_trajectory.trajectory.leftCols(static_cast<long>(prob_ptr->GetKin()->getJointNames().size()));
        refineTrajectory(refined_traj);

        //std::cout << "Refined traj:" << std::endl;
        //std::cout << refined_traj << std::endl;

        // // plot current `action` result
        //  plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

        // if (rviz_enabled)
        // {
        //     ROS_WARN("Update Env");
        //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        // }

        // // update env according to the action
        env.updateEnv(response.joint_trajectory.joint_names, response.joint_trajectory.trajectory.bottomRows(1).transpose(), action);
        //plotter->clear();

        // switch(action->getActionType())
        // {
        // case vkc::ActionType::PlaceAction:
        //     ROS_WARN("detach object: %s", std::dynamic_pointer_cast<PlaceAction>(action)->getDetachedObject().c_str());
        //     ROS_WARN("before detach current tip link: %s", env.getVKCEnv()->getTesseract()->getInvKinematicsManager()->getInvKinematicSolver("vkc")->getTipLinkName().c_str());
        //     env.detachObject(std::dynamic_pointer_cast<PlaceAction>(action)->getDetachedObject());
        //     ROS_WARN("after detach current tip link: %s", env.getVKCEnv()->getTesseract()->getInvKinematicsManager()->getInvKinematicSolver("vkc")->getTipLinkName().c_str());

        //     // Now update rviz environment
        //     if (!env.sendRvizChanges_(env_revision))
        //         return;

        //     env.getVKCEnv()->getTesseract()->getEnvironment()->setState(joint_trajs.back().joint_names,
        //                                                                 joint_trajs.back().trajectory.bottomRows(1).transpose());
        //     env_revision = env.getVKCEnv()->getTesseract()->getEnvironment()->getRevision();

        //     break;
        // }

        ++i;
        pre_act = action;
    }
}

void TrajectoryVisualize(ArenaEnv& env,
                         ActionSeq &actions,
                         vector<tesseract_common::JointTrajectory> &joint_trajs)
{
        ROS_INFO("actions size: %d, traj size: %d", actions.size(), joint_trajs.size());

        // for(auto& action : actions)
        // {
        //     std::cout << action << std::endl;
        // }

        long int max_traj_len{0};
        for(auto& traj : joint_trajs)
        {
            max_traj_len = traj.trajectory.size() > max_traj_len ? traj.trajectory.size() : max_traj_len;
            // std::cout << ">> traj details: " << std::endl
            //           << traj << std::endl;
        }

        // plot current `action` result
        auto action_iter = actions.begin();
        auto joint_traj_iter = joint_trajs.begin();

        ROSPlottingPtr plotter;
        for (; joint_traj_iter != joint_trajs.end(); ++action_iter, ++joint_traj_iter)
        {

            ROS_INFO("joints names number: %d, joint number: %d, joint states number: %d",
                     joint_traj_iter->joint_names.size(), joint_traj_iter->trajectory.cols(), joint_traj_iter->trajectory.rows());
            tesseract_common::TrajArray refined_traj = joint_traj_iter->trajectory.leftCols(static_cast<long>(joint_traj_iter->joint_names.size()));
            refineTrajectory(refined_traj);

            plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());
            plotter->plotTrajectory(joint_traj_iter->joint_names, refined_traj);
            usleep( (int)(joint_traj_iter->trajectory.size() * 3000000.0 / max_traj_len));

            // ROS_INFO("%s: Update Env, action: ", __func__);
            // std::cout << *action_iter << "\t";


            //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // update env according to the action
            //env.updateEnv(*joint_name_iter, joint_traj_iter->bottomRows(1).transpose(), *action_iter);
            plotter->clear();
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_door_env_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    // console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

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

    // added: wanglei@bigai.ai
    // time: 2021-08-17
    // get specified task plan file
    std::string plan_file_path{ros::package::getPath("vkc_example") + "/task_plan/"};
    pnh.param("plan_file_path", plan_file_path, plan_file_path);

    std::string plan_file{"my_plan"};
    pnh.param("plan_file", plan_file, plan_file);
    ROS_INFO("task action file path: %s", (plan_file_path + plan_file).c_str());

    ActionSeq actions;
    TaskPlanParser plan_parser(GetSceneObjects());
    plan_parser.Parse(actions, plan_file_path + plan_file);
    std::cout << actions << std::endl;
    //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    // cache the planning result for replaying
    vector<tesseract_common::JointTrajectory> joint_trajs;

    ArenaEnv env(nh, plotting, rviz, steps);
    // plan motion trajectory according to given task actions
    {
        run(env, actions, steps, n_iter, rviz, nruns, joint_trajs);
    }

    // visualize the trajectory as planned
    {
        //ArenaEnv env(nh, plotting, rviz, steps);
        TrajectoryVisualize(env, actions, joint_trajs);
    }
}
