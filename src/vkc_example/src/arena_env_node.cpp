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

#include <tesseract_motion_planners/ompl/chain_ompl_interface.h>
#include <tesseract_motion_planners/ompl/conversions.h>
#include <tesseract_collision/core/types.h>
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

void genRandState(tesseract_kinematics::ForwardKinematics::Ptr kin, Eigen::VectorXd &seed)
{
    Eigen::MatrixX2d joint_limits = kin->getLimits();
    
    for (int i = 0; i < seed.size(); ++i)
    {
        double f = (double)rand() / RAND_MAX;
        seed[i] = joint_limits(i, 0) + f * (joint_limits(i, 1) - joint_limits(i, 0));
    }
}

bool planByOMPL(tesseract_common::JointTrajectory& traj, VKCEnvBasic &env, ActionBase::Ptr action)
{
    const std::string manipulator{"vkc"};

    tesseract_motion_planners::OmplPlanParameters param;
    param.n_steps = 30;
    param.planning_time = 5.0;
    param.simplify = true;



    ompl::RNG::setSeed(SEED);
    tesseract::Tesseract::Ptr tesseract = env.getVKCEnv()->getTesseract();
    auto kin = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);

    
    // configure link objectives and joint objectives
    std::vector<LinkDesiredPose> l_objs;   // link ojectives
    std::vector<JointDesiredPose> j_objs;  // joint objectives

    switch (action->getActionType())
    {
    case ActionType::PickAction:
    {
        std::string attach_link{std::dynamic_pointer_cast<PickAction>(action)->getAttachedObject()};
        l_objs.emplace_back(LinkDesiredPose(env.getEndEffectorLink(), env.getAttachLocation(attach_link)->world_joint_origin_transform));
    }
    break;

    case ActionType::PlaceAction:
    {
        l_objs = std::dynamic_pointer_cast<PlaceAction>(action)->getLinkObjectives();
        j_objs = std::dynamic_pointer_cast<PlaceAction>(action)->getJointObjectives();
    }
    break;

    default:
        ROS_ERROR("unexpected action type: %d", action->getActionType());
        return false;
    }
    LinkDesiredPose link_obj = l_objs[0];

    // cache the start state  
    Eigen::VectorXd cur_joint_state = tesseract->getEnvironment()->getCurrentJointValues(kin->getJointNames());
    auto start = std::make_shared<tesseract_motion_planners::JointWaypoint>(cur_joint_state, kin->getJointNames());

    // calculate end joint state by solving a kinematics problem
    auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(tesseract->getEnvironmentConst()->getSceneGraph(),
                                                                         kin->getActiveLinkNames(),
                                                                         tesseract->getEnvironmentConst()->getCurrentState()->transforms);


    bool satisfied = false;
    int attempts = 0;
    const unsigned short MAX_ATTEMPTS = 100;
    tesseract_collision::ContactResultMap collisions;

    auto inv_kin_solver = tesseract->getInvKinematicsManager()->getInvKinematicSolver(manipulator);
    Eigen::VectorXd seeds(inv_kin_solver->numJoints());
    Eigen::VectorXd solutions(inv_kin_solver->numJoints());  // allocate numJoints buffer
    tesseract_environment::StateSolver::Ptr state_solver = tesseract->getEnvironmentConst()->getStateSolver();
    auto collision_manager = tesseract->getEnvironmentConst()->getDiscreteContactManager();
    collision_manager->setActiveCollisionObjects(adj_map->getActiveLinkNames());
    while (attempts++ < MAX_ATTEMPTS)
    {
        if (inv_kin_solver->calcInvKin(solutions, l_objs[0].tf, seeds))
        {
            state_solver->setState(kin->getJointNames(), solutions);

            tesseract_environment::EnvState::Ptr state = state_solver->getState(kin->getJointNames(), solutions);
            for (const auto &link_name : collision_manager->getActiveCollisionObjects())
            {
                collision_manager->setCollisionObjectsTransform(link_name, state->transforms[link_name]);
            }

            collisions.clear();
            collision_manager->contactTest(collisions, tesseract_collision::ContactTestType::FIRST);
            if (collisions.size() > 0)
            {
                genRandState(kin, seeds);
            }
            else
            {
                satisfied = true;
                break;
            }
        }
    }

    if(!satisfied)
    {
        // TODO
        ROS_ERROR("[%s]exceed maximum inverser kinematics solving attempts, max_attempts: %d", __func__, MAX_ATTEMPTS);
        return false;
    }

    
    auto end = std::make_shared<tesseract_motion_planners::JointWaypoint>(solutions, kin->getJointNames());

    tesseract_motion_planners::ChainOmplInterface::Ptr ompl_planner =
        std::make_shared<ChainOmplInterface>(env.getVKCEnv()->getTesseract()->getEnvironmentConst(), kin);
    
    ompl_planner->setAdjacencyMap();
    ompl_planner->ss_->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(ompl_planner->spaceInformation()));



    boost::optional<ompl::geometric::PathGeometric> maybe_path  = ompl_planner->plan(start->getPositions(), end->getPositions(), param);
    if(maybe_path)
    {
        CONSOLE_BRIDGE_logError("[%s]no solution found from OMPL", __func__);
        return false;
    }

    traj.joint_names = kin->getJointNames();
    traj.trajectory = vkc::toTrajArray(*maybe_path);

    return true;
}


void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter, bool rviz_enabled, int nruns,
         vector<tesseract_common::JointTrajectory>& joint_trajs)
{
    // init base position of robot
    vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
    vector<double> base_values({0, 0});
    env.getVKCEnv()->getTesseract()->getEnvironment()->setState(base_joints, base_values);


    CostInfo cost;
    ProbGenerator prob_generator;
    vkc::ActionBase::Ptr pre_act = nullptr;

    // create OMPL planner specified problem translator
    vkc::OmplPlanParameters params; // use all default parameters for our beginning
    params.plan_params.n_steps = n_steps;
    params.inv_attp_max = 10000;
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
        if(ActionType::PlaceAction != action->getActionType() || string::npos == env.getEndEffectorLink().find("cabinet"))
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
        else
        {
            ROS_INFO("Skipped OMPL planning stage");
        }

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        int tries = 0;
        bool converged = false;
        while (!converged && tries < 1)
        {
            TrajOptProb::Ptr prob_ptr = prob_generator.genProb(env, action, n_steps);

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

            // ROS_WARN("solve problem has tried %d times", tries);
            // if (rviz_enabled)
            // {
            //     ROS_WARN("Created optimization problem. Press <Enter> to start optimization");
            //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            // }

            auto start = std::chrono::system_clock::now();
            cost = solveProb_cost(prob_ptr, response, n_iter, false);
            std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
                    ROS_INFO("it takes %f sec optimizing the trajectory!", elapsed_seconds.count());
        
            converged = sco::OptStatus::OPT_CONVERGED == response.status_code;


            // update try counter
            tries += 1;
        }

        
        // record planning result
        tesseract_common::JointTrajectory joint_traj{response.joint_names, response.trajectory}; 
        joint_trajs.push_back(joint_traj);

        // refine the orientation of the move base
        tesseract_common::TrajArray refined_traj =
            response.trajectory.leftCols(response.joint_names.size());
        refineTrajectory(refined_traj);

        //std::cout << "Refined traj:" << std::endl;
        //std::cout << refined_traj << std::endl;

        // plot current `action` result
         plotter->plotTrajectory(response.joint_names, refined_traj);


        // // update env according to the action
        env.updateEnv(response.joint_names, response.trajectory.bottomRows(1).transpose(), action);
        plotter->clear();


        pre_act = action;
    }
}

void TrajectoryVisualize(ArenaEnv& env,
                         ActionSeq &actions,
                         vector<tesseract_common::JointTrajectory> &joint_trajs)
{
        ROS_INFO("[%s]actions size: %d, traj size: %d", __func__, actions.size(), joint_trajs.size());

        // reset rviz
        ROS_INFO("[%s]please reset rviz and presss ENTER to go on...", __func__);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        env.reInit();


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


        // plot current `action` trajectory data
        for (; joint_traj_iter != joint_trajs.end(); ++action_iter, ++joint_traj_iter)
        {
            ROS_INFO("joints names number: %d, joint number: %d, joint states number: %d, revision: %d",
                     joint_traj_iter->joint_names.size(), 
                     joint_traj_iter->trajectory.cols(),
                     joint_traj_iter->trajectory.rows());
            tesseract_common::TrajArray refined_traj = joint_traj_iter->trajectory.leftCols(static_cast<long>(joint_traj_iter->joint_names.size()));
            refineTrajectory(refined_traj);

            ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(env.getPlotVKCEnv()->getTesseract()->getEnvironment());
            plotter->plotTrajectory(joint_traj_iter->joint_names, refined_traj);
            usleep((useconds_t)(joint_traj_iter->trajectory.size() * 3000000.0 / max_traj_len));

            // ROS_INFO("%s: Update Env, action: ", __func__);
            // std::cout << *action_iter << "\t";

            // update env according to the action
            //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            env.updatePlotEnv(joint_traj_iter->joint_names, joint_traj_iter->trajectory.bottomRows(1).transpose(), *action_iter);
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
