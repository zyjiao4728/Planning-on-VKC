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


// class BlendingPath
// {
// public:
//     BlendingPath(){}
//     void AddPath(const std::vector<std::string>& joint_names, const tesseract_common::TrajArray& path)
//     {
//         std::vector<std::string> tmp_joint_names(joint_names);
        
//         // get to know how many path points to extend
//         size_t rows = path.rows();

//         for(auto iter = joint_names_.begin(); iter != joint_names_.end(); ++it)
//         {
//             // get the joint path to update
//             std::vector<double> &update_joint_path = joint_paths_[iter - joint_names_.begin()];


//             auto find_it = find(tmp_joint_names.begin(), temp_joint_names.end(), *iter);
//             // first update the existing joint paths
//             if(tmp_joint_names.end() != find_it)
//             {
//                 // the joint alread recorded has changes
//                 int c = find_it - tmp_joint_names.begin();
//                 update_joint_path.insert(update_joint_path.end(), path.block<rows, 1>(0, path_index).begin(), path.block<rows, 1>(0, path_index).end());

//                 *find_it = "";  // means flag this joint deleted
//             }
//             else  // the joint alread recorded has no change
//             {
//                 // the joint has no updated
//                 update_joint_path.insert(update_joint_path.end(), rows, *update_joint_path.rbegin());
//             }
//         }



//         for(auto iter = tmp_joint_names.begin(); iter != tmp_joint_names.end(); ++iter)
//         {
//             if("" == *iter)
//             {
//                 continue;
//             }

//             auto new_path_section = path.block<rows, 1>(0, iter - tmp_joint_names.begin());
//             if(0 == joint_paths_.size())
//             {
//                 std::vector<float> joint_path(new_path_section.begin(), new_path_section.end());
//                 joint_paths_.emplace_back(joint_path);
//                 joint_names_.emplace_back(*iter);
//             }
//             else
//             {
//                 std::vector<float> joint_path(joint_paths_[0].size() - rows, joint_new_path[0]);
//                 joint_path.insert(joint_path.end(), new_path_section.begin(), new_path_section.end());
//                 joint_paths_.emplace_back(joint_path);
//                 joint_names_.emplace_back(*iter);
//             }
//         }


//     }
//     void GetPath(std::vector<std::string>& joint_names, tesseract_common::TrajArray& path)
//     {
//         if(0 != joint_paths_.size())
//         {
//             // the first dimension is waypoints' number, the second dimension is joints' number
//             path.resize(joint_paths_[0].size(), joint_paths_.size());
//             path = Map<Matrix<float, joint_paths_[0].size(), joint_paths_.size(), RowMajor> >(joint_paths_.data());

//             joint_names = joint_names_;
//         }
//     }

// private:
//     std::vector<std::string> joint_names_;
//     // TODO: list maybe much better
//     std::vector<std::vector<float>> joint_paths_;
// }

void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled, int nruns,
         vector<vector<string>>& joint_names_record,
         vector<PlannerResponse>& planner_responses,
         vector<tesseract_common::TrajArray>& joint_trajs_record)
{
    ProbGenerator prob_generator;
    // ROSPlottingPtr plotter;

    CostInfo cost;


    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    vector<double> base_values({0, 0});

    env.getVKCEnv()->getTesseract()->getEnvironment()->setState(base_joints, base_values);

    int i = 0;
    for (auto &action : actions)
    {
        PlannerResponse response;
        TrajOptProb::Ptr prob_ptr = nullptr;
        // plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());

        bool converged = false;
        int tries = 0;
        std::chrono::duration<double> elapsed_seconds;

        std::cout << __func__ << ": " << action << std::endl;
        while (!converged && tries < 5)
        {
            prob_ptr = prob_generator.genProb(env, action, n_steps);

            // 6: is half size of the map 
            if ((abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(0)) - 6) < 1e-6 || abs(abs(prob_ptr->GetInitTraj().bottomRows(1)(1)) - 6) < 1e-6) && env.getEndEffectorLink().find("cabinet") == string::npos)
            {
                //std::cout << prob_ptr->GetInitTraj().bottomRows(1)(0) << " " << prob_ptr->GetInitTraj().bottomRows(1)(1) << std::endl;
                if (tries < 4)
                    continue;
            }

            // ROS_WARN("motion planning for action: %d", i);
            // ROS_WARN("solve problem has tried %d times", tries);
            // if (rviz_enabled)
            // {
            //     ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
            //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // }

            auto start = std::chrono::system_clock::now();
            cost = solveProb_cost(prob_ptr, response, n_iter);
            elapsed_seconds = std::chrono::system_clock::now() - start;
            
            converged = sco::OptStatus::OPT_CONVERGED == response.status.value();


            // update try counter
            tries += 1;
        }

        
        // record planning result
        planner_responses.push_back(response);
        // record optimized joint names in this step
        joint_names_record.push_back(prob_ptr->GetKin()->getJointNames());

        // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.joint_trajectory.trajectory.leftCols(static_cast<long>(prob_ptr->GetKin()->getJointNames().size()));
        refineTrajectory(refined_traj);
        joint_trajs_record.push_back(refined_traj);

        //std::cout << "Refined traj:" << std::endl;
        //std::cout << refined_traj << std::endl;

        // // plot current `action` result
        // plotter->plotTrajectory(prob_ptr->GetKin()->getJointNames(), refined_traj);

        // if (rviz_enabled)
        // {
        //     ROS_WARN("Update Env");
        //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        // }

        // // update env according to the action
        env.updateEnv(joint_names_record.back(), response, action);
        // plotter->clear();
        ++i;
    }
}

void TrajectoryVisualize(ArenaEnv& env,
                         ActionSeq &actions,
                         vector<vector<string>> &joint_names_record,
                         vector<PlannerResponse> &planner_responses,
                         vector<tesseract_common::TrajArray> &joint_trajs_record)
{
        ROS_INFO("actions size: %d, joints motion names size: %d, traj size: %d, planner response size: %d",
                 actions.size(), planner_responses.size(), joint_names_record.size(), joint_trajs_record.size());

        for(auto& action : actions)
        {
            std::cout << action << std::endl;
        }

        long int max_traj_len{0};
        for(auto& traj : joint_trajs_record)
        {
            max_traj_len = traj.size() > max_traj_len ? traj.size() : max_traj_len;
            std::cout << ">> traj details: " << std::endl
                      << traj << std::endl;
        }

        // plot current `action` result
        auto action_iter = actions.begin();
        auto response_iter = planner_responses.begin();
        auto joint_name_iter = joint_names_record.begin();
        auto joint_traj_iter = joint_trajs_record.begin();

        ROSPlottingPtr plotter;
        for (; joint_traj_iter != joint_trajs_record.end(); ++action_iter, ++response_iter, ++joint_name_iter, ++joint_traj_iter)
        {
            plotter = std::make_shared<ROSPlotting>(env.getVKCEnv()->getTesseract()->getEnvironment());
            plotter->plotTrajectory(*joint_name_iter, *joint_traj_iter);

            // ROS_INFO("%s: Update Env, action: ", __func__);
            // std::cout << *action_iter << "\t";


            //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // update env according to the action
            env.updateEnv(*joint_name_iter, *response_iter, *action_iter);
            plotter->clear();
            usleep( (int)(joint_traj_iter->size() * 1500000.0 / max_traj_len));
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
    vector<vector<string>> joint_names_record;
    vector<PlannerResponse> planner_responses;
    vector<tesseract_common::TrajArray> joint_trajs_record;

        ArenaEnv env(nh, plotting, rviz, steps);
    // plan motion trajectory according to given task actions
    {
        run(env, actions, steps, n_iter, rviz, nruns,
            joint_names_record, planner_responses, joint_trajs_record);
    }

    // visualize the trajectory as planned
    {
        //ArenaEnv env(nh, plotting, rviz, steps);
        TrajectoryVisualize(env, actions, joint_names_record, planner_responses, joint_trajs_record);
    }
}
