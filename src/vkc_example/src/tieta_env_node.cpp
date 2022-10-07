#include <ros/package.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

using namespace vkc;
using namespace tesseract_planning;
void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter,
         bool rviz_enabled, unsigned int nruns) {
  int window_size = 3;
  LongHorizonSeedGenerator seed_generator(n_steps, n_iter, window_size);
  ProbGenerator prob_generator;

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  Eigen::Isometry3d cabinet_handle_pose;
  cabinet_handle_pose.setIdentity();
  cabinet_handle_pose.translation() =
      Eigen::Vector3d(1.56603773475, -2.12803775693, 1.10999932578); //1.10999932578
  cabinet_handle_pose.linear() = Eigen::Quaterniond(-0.00417585092715, -0.00241137511214,
                                       -0.000891939974928, 0.999987975919)
                        .matrix();
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "closet_base_joint", cabinet_handle_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d table_pose;
  table_pose.setIdentity();
  table_pose.translation() =
      Eigen::Vector3d(-10.128499117271, -2.12167326143, 0.741853607259);
  table_pose.linear() = Eigen::Quaterniond(0.707105521577, 0.000190096986757,
                                       6.89455638473e-05, 0.70710801188)
                        .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d drawer_pose;
  drawer_pose.setIdentity();
  drawer_pose.translation() =
      Eigen::Vector3d(1.60919134161, -2.29974505471, 0.74223606381);
  drawer_pose.linear() = Eigen::Quaterniond(-0.000927817441917, 0.00145746889854, -0.00129275269822,
                                       0.999997671862)
                        .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "drawer_world_joint", drawer_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    seed_generator.generate(env, sub_actions);
    action->switchCandidate();

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      if (rviz_enabled) {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }
      solveProb(prob_ptr, response, n_iter);

      // break;
      if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
          response.status.value())  // optimization converges
      {
        converged = true;
        break;
      } else {
        ROS_WARN(
            "[%s]optimization could not converge, response code: %d, "
            "description: %s",
            __func__, response.status.value(),
            response.status.message().c_str());
        action->switchCandidate();
      }
    }
    const auto &ci = response.results;

    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    tesseract_common::JointTrajectory refined_traj = trajectory;
    // refineTrajectory(refined_traj, env);

    if (rviz_enabled && env.getPlotter() != nullptr) {
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
    auto current_time =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto t = std::localtime(&current_time);
    char buf[80];
    std::strftime(buf, sizeof(buf), "%T", t);
    std::string save_path = ros::package::getPath("vkc_example") +
                            "/trajectory/tieta_env_" + action->Name() + buf +
                            ".csv";

    std::cout << "saving path to: " << save_path << std::endl;

    toDelimitedFile(ci, save_path, ',');

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);
    CONSOLE_BRIDGE_logInform("update env finished");

    if (env.getPlotter() != nullptr && rviz_enabled) env.getPlotter()->clear();
  }
}

ActionSeq getTietaEnvSeq(const std::string robot) {
  ActionSeq actions;

  // action1: pick closet handle
  {
    auto pick_action =
        std::make_shared<PickAction>(robot, "attach_closet_right_handle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action2: place closet handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("closet_bottom_right_door_joint", -1.570796);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives, false);
    actions.emplace_back(place_action);
  }

  // action3: pick drawer handle
  {
    auto pick_action =
        std::make_shared<PickAction>(robot, "attach_drawer_handle1");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action4: place drawer handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer_base_drawer1_joint", -0.22);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_drawer_handle1",
                                      link_objectives, joint_objectives, false);
    actions.emplace_back(place_action);
  }

  return actions;
}

void genEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                        UrdfSceneEnv::InverseChainsInfos &inverse_chains) {
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_closet_right_handle",
      "closet_bottom_right_handle",
      "closet_base_link",
      {0.26, 0.000, 0.00},
      {0.6532815, 0.2705981, -0.6532815, -0.2705981},
      // {0.923879532511287,0,0,0.382683432365090},
      // {0.5,0.5,-0.5,-0.5},
      true});
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_drawer_handle1",
      "drawer_handle1",
      "drawer_base_link",
      {0.26, 0.000, 0.00},
      // {0.707106781186548, 0, -0.707106781186548, 0},
      {0.653281482438188,-0.270598050073098,-0.653281482438188,0.270598050073099},
      // {1,0,0,0},
      // {0.5,0.5,-0.5,-0.5},
      true});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "closet_base_link", "closet_bottom_right_handle"});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "drawer_base_link", "drawer_handle1"});
  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

int main(int argc, char **argv) {
  srand((unsigned)time(NULL));
  ros::init(argc, argv, "tieta_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  setupLog(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);

  bool plotting = true;
  bool rviz = true;
  int steps = 10;
  int n_iter = 1000;
  int nruns = 5;
  std::string robot{"vkc"};

  pnh.param<std::string>("robot", robot, robot);
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);

  UrdfSceneEnv::AttachObjectInfos attaches;
  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  genEnvironmentInfo(attaches, inverse_chains);

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
  ActionSeq actions = getTietaEnvSeq("vkc");

  run(env, actions, steps, n_iter, rviz, nruns);
}
