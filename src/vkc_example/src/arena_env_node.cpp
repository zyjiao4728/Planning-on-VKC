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


void run(VKCEnvBasic &env, ActionSeq actions, int n_steps, int n_iter, bool rviz_enabled, int nruns)
{
    ProbGenerator prob_generator;
    ROSPlottingPtr plotter;

    CostInfo cost;

    vector<vector<string>> joint_names_record;
    vector<PlannerResponse> planner_responses;

    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
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

            if ((abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(0)) - 6) < 1e-6 || abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(1)) - 6) < 1e-6) && env.getEndEffectorLink().find("cabinet") == string::npos)
            {
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
            ROS_WARN("solve problem 1");
            cost = solveProb_cost(prob_ptr, response, n_iter);
            ROS_WARN("solve problem 2");
            elapsed_seconds = std::chrono::system_clock::now() - start;

            if (response.status.value() == 0 || action->getActionType() == ActionType::PlaceAction)
            {
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_door_env_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    // console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);
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

    ArenaEnv env(nh, plotting, rviz, steps);


    // added: wanglei@bigai.ai
    // time: 2021-08-17
    // get specified task plan file 
    std::string plan_file_path{ros::package::getPath("vkc_example") + "/task_plan/"};
    pnh.param("plan_file_path", plan_file_path, plan_file_path);

    std::string plan_file{"my_plan"};
    pnh.param("plan_file", plan_file, plan_file);
    ROS_INFO("plan file path: %s", (plan_file_path + plan_file).c_str());

    ActionSeq actions;
    TaskPlanParser plan_parser(GetSceneObjects());
    plan_parser.Parse(actions, plan_file_path + plan_file);

    std::cout << actions << std::endl;
    run(env, actions, steps, n_iter, rviz, nruns);

}
