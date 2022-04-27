// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for parsing task plan file
#include <ros/package.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

#include <iomanip>

#include "vkc/action/TaskPlanParser.h"
#include "vkc/action/actions.h"
#include "vkc/env/arena_env.h"
#include "vkc/env/vkc_env_basic.h"
#include "vkc/planner/prob_generator.h"
#include "vkc/planner/prob_translator.h"
#include "vkc_example/utils.h"

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
// using namespace vkc_example;

const static int SEED = 1;
const static bool PLANNER_VERBOSE = false;

static SceneObjects &GetSceneObjects() {
  static SceneObjects scene_objects = SceneObjects();
  return scene_objects;
}

void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter,
         bool rviz_enabled, int nruns,
         vector<tesseract_common::JointTrajectory> &joint_trajs) {
  // init base position of robot
  vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
  Eigen::Vector2d base_values({0, 0});
  env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);

  ProbGenerator prob_generator;

  // create OMPL planner specified problem translator
  vkc::OmplPlanParameters
      params;  // use all default parameters for our beginning
  // params.plan_params.n_steps = n_steps;
  params.inv_attp_max = 1000;
  ProbTranslator prob_translator(params);
  for (auto &action : actions) {
    PlannerResponse response;

    ROS_INFO(
        "[%s]current end effector: %s! action type: %s, init_traj_required: %s",
        __func__, env.getEndEffectorLink().c_str(), action->Name().c_str(),
        (action->RequireInitTraj() ? "yes" : "no"));

    env.getPlotter()->waitForInput("press Enter key to go on...");
    // ROS_INFO("[%s]press Enter key to go on...", __func__);
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    int tries = 0;
    bool converged = false;
    while (tries++ < nruns) {
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      env.getPlotter()->waitForInput(
          "request generation success, press enter to continue");

      ROS_WARN("[%s]tried %d times to solve problem", __func__, tries);

      auto start = std::chrono::system_clock::now();

      solveProb(prob_ptr, response, n_iter);

      std::chrono::duration<double> elapsed_seconds =
          std::chrono::system_clock::now() - start;
      ROS_INFO("it takes %f sec optimizing the trajectory!",
               elapsed_seconds.count());

      if (response.status.value() ==
          TrajOptMotionPlannerStatusCategory::SolutionFound) {
        ROS_INFO("solution found");
        break;
      } else {
        ROS_WARN(
            "[%s]optimization could not converge, response code: %d, "
            "description: %s",
            __func__, response.status.value(),
            response.status.message().c_str());
      }
    }
    const auto &ci = response.results;

    // refine the orientation of the move base
    tesseract_common::JointTrajectory refined_traj = toJointTrajectory(ci);
    // refineTrajectory(refined_traj);

    // std::cout << "Refined traj:" << std::endl;
    // std::cout << refined_traj << std::endl;

    // record planning result
    // tesseract_common::JointTrajectory joint_traj{response.joint_names,
    //                                              refined_traj};
    // joint_trajs.push_back(joint_traj);

    // plot current `action` result
    if (env.getPlotter() != nullptr) {
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

    // // update env according to the action
    env.updateEnv(refined_traj.back().joint_names, refined_traj.back().position,
                  action);
    if (env.getPlotter() != nullptr) {
      env.getPlotter()->clear();
    }
  }
}

int main(int argc, char **argv) {
  srand(time(NULL));
  ros::init(argc, argv, "open_door_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN);
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);

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
  std::string plan_file_path{ros::package::getPath("vkc_example") +
                             "/task_plan/"};
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
