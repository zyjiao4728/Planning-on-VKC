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
  cabinet_handle_pose.translation() = Eigen::Vector3d(1.24748926979, 0.0795272607703, 1.15487884514);
  
  cabinet_handle_pose.linear() =
      Eigen::Quaterniond(-0.0276394300716, -0.00732565483015, 0.00295412146665, 0.999586749539)
          .matrix();
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "closet_base_joint", cabinet_handle_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d table_pose;
  table_pose.setIdentity();
  table_pose.translation() =
      Eigen::Vector3d(-0.505623316002, 0.0720372811975, 0.767919456786);
  table_pose.linear() = Eigen::Quaterniond(0.999995378251, -0.000870022109151, -0.000279317347915, 0.00289974475127)
                            .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d drawer_pose;
  drawer_pose.setIdentity();
  drawer_pose.translation() =
      Eigen::Vector3d(1.2955805008, -0.0981322452166, 0.788374883425);
  
  drawer_pose.linear() =
      Eigen::Quaterniond(0.000381468008155, -0.00647296044046, -0.0020959194287, 0.999976780924)
          .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "drawer_world_joint", drawer_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();
  box_pose.translation() =
      Eigen::Vector3d(-0.488343376063, 0.221549065267, 0.8100664763);
  box_pose.linear() = Eigen::Quaterniond(0.513140008661, -0.484234159589, -0.514466554295, -0.487369238576)
                          .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "box_box_base_joint", box_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    // seed_generator.generate(env, sub_actions);
    // action->switchCandidate();

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
        // action->switchCandidate();
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

    joint_objectives.emplace_back("closet_bottom_right_door_joint", -1.9198621771937625);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives, false);
    // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
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
    // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  // action5: pick box
  {
    auto pick_action = std::make_shared<PickAction>(robot, "attach_box");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action6: place box
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() = Eigen::Vector3d(1.19828555737, -0.109178547221, 0.801900872831 +0.15);
    destination.linear() = Eigen::Quaterniond(0.527083066682, 0.535179179479, -0.436224629928, 0.495454093649).matrix();
    link_objectives.push_back(
        LinkDesiredPose("box_box_base_link", destination));

    auto place_action = std::make_shared<PlaceAction>(
        robot, "attach_box", link_objectives, joint_objectives, false);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  // action7: pick drawer handle
  {
    auto pick_action =
        std::make_shared<PickAction>(robot, "attach_drawer_handle1");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action8: place drawer handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer_base_drawer1_joint", 0.0);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_drawer_handle1",
                                      link_objectives, joint_objectives, false);
    // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  // action9: pick closet handle
  {
    auto pick_action =
        std::make_shared<PickAction>(robot, "attach_closet_right_handle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(pick_action);
  }

  // action10: place closet handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("closet_bottom_right_door_joint", 0.0);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives, false);
    // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
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
      {0.653281482438188, -0.270598050073098, -0.653281482438188,
       0.270598050073099},
      // {1,0,0,0},
      // {0.5,0.5,-0.5,-0.5},
      true});
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_box",
      "box_box_base_link",
      "box_box_base_link",
      {0.20, 0.000, 0.00},
      // {0.707106781186548, 0, -0.707106781186548, 0},
      {0.6532815, 0.2705981, -0.6532815, -0.2705981},
      // {1,0,0,0},
      // {0.5,0.5,-0.5,-0.5},
      false});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "closet_base_link", "closet_bottom_right_handle"});
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"drawer_base_link", "drawer_handle1"});
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
