// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for parsing task plan file
#include "vkc/action/TaskPlanParser.h"

#include "vkc/env/arena_env.h"
#include "vkc/env/vkc_env_basic.h"
#include "vkc/planner/prob_generator.h"
#include "vkc_example/utils.h"
#include "vkc/action/actions.h"
#include "vkc/planner/prob_translator.h"

#include <ros/package.h>
#include <iomanip>

#include <tesseract_collision/core/types.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_motion_planners/ompl/chain_ompl_interface.h>

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;
// using namespace vkc_example;

const static int SEED = 1;
const static bool PLANNER_VERBOSE = false;

static SceneObjects &GetSceneObjects()
{
    static SceneObjects scene_objects = SceneObjects();
    return scene_objects;
}

void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled, int nruns,
         vector<tesseract_common::JointTrajectory> &joint_trajs)
{
    // init base position of robot
    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    vector<double> base_values({0, 0});
    env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);

    ProbGenerator prob_generator;

    // create OMPL planner specified problem translator
    vkc::OmplPlanParameters params; // use all default parameters for our beginning
    params.plan_params.n_steps = n_steps;
    params.inv_attp_max = 1000;
    ProbTranslator prob_translator(params);
    for (auto &action : actions)
    {
        PlannerResponse response;
        ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

        ROS_INFO("[%s]current end effector: %s! action type: %s, init_traj_required: %s",
                 __func__,
                 env.getEndEffectorLink().c_str(),
                 action->Name().c_str(),
                 (action->RequireInitTraj() ? "yes" : "no"));

        // articulated body planning together with robot is still has problem
        if (ActionType::PlaceAction != action->getActionType() || std::dynamic_pointer_cast<PlaceAction>(action)->isRigidObject())
        {
            // solve by OMPL to get an init trajectory
            std::vector<std::vector<double>> res_traj;
            prob_translator.transProb(env, action);
            prob_translator.solveProblem(response, res_traj);
            if (response.status_code)
            {
                ROS_INFO("[%s]OMPL planning plan successfully!", __func__);
                action->setInitTrajectory(response.trajectory);
            }
            else
            {
                ROS_INFO("OMPL planning plan failed");
            }
        }
        else
        {
            ROS_INFO("Skipped OMPL planning stage");
        }

        ROS_INFO("[%s]press Enter key to go on...", __func__);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        int tries = 0;
        bool converged = false;
        while (!converged && tries++ < 5)
        {
            TrajOptProb::Ptr prob_ptr = prob_generator.genRequest(env, action, n_steps);

            // 6: is half size of the map
            bool on_map_border = (abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(0)) - 6) < 1e-6 || abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(1)) - 6) < 1e-6);
            if (on_map_border && string::npos == env.getEndEffectorLink().find("cabinet"))
            {
                // std::cout << prob_ptr->GetInitTraj().bottomRows(1)(0) << " " << prob_ptr->GetInitTraj().bottomRows(1)(1) << std::endl;
                if (tries < 4)
                {
                    ROS_INFO("unfornately got here before optimizing the trajectory!");
                    continue;
                }
            }

            ROS_WARN("[%s]tried %d times to solve problem", __func__, tries);
            // if (rviz_enabled)
            // {
            //     ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
            //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // }

            auto start = std::chrono::system_clock::now();
            CostInfo cost = solveProb_cost(prob_ptr, response, n_iter, false);
            std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
            ROS_INFO("it takes %f sec optimizing the trajectory!", elapsed_seconds.count());

            converged = sco::OptStatus::OPT_CONVERGED == response.status_code;

            // update try counter
            tries += 1;
        }

        // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.trajectory.leftCols(response.joint_names.size());
        refineTrajectory(refined_traj);

        // std::cout << "Refined traj:" << std::endl;
        // std::cout << refined_traj << std::endl;

        // record planning result
        tesseract_common::JointTrajectory joint_traj{response.joint_names, refined_traj};
        joint_trajs.push_back(joint_traj);

        // plot current `action` result
        plotter->plotTrajectory(response.joint_names, refined_traj);

        ROS_INFO("[%s]press Enter key to continue...", __func__);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // // update env according to the action
        env.updateEnv(response.joint_names, response.trajectory.bottomRows(1).transpose(), action);
        plotter->clear();
    }
}

int main(int argc, char **argv)
{
    srand(time(NULL));
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
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // cache the planning result for replaying
    vector<tesseract_common::JointTrajectory> joint_trajs;

    ArenaEnv env(nh, plotting, rviz, steps);
    // plan motion trajectory according to given task actions
    run(env, actions, steps, n_iter, rviz, nruns, joint_trajs);

    // visualize the trajectory as planned
    TrajectoryVisualize(env, actions, joint_trajs);
}
