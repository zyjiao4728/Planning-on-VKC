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
  cabinet_handle_pose.translation() = Eigen::Vector3d(1.45919438429, 0.0497998412466, 1.16019205937);
  
  cabinet_handle_pose.linear() =
      Eigen::Quaterniond(-0.00271719060289, -0.00795289109614, 0.00189318902276, 0.999962891428)
          .matrix();
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "closet_base_joint", cabinet_handle_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d table_pose;
  table_pose.setIdentity();
  table_pose.translation() =
      Eigen::Vector3d(-0.256890942294, 0.06, 0.771871020222);
  table_pose.linear() = Eigen::Quaterniond(0.999982951639, -0.000148883592137, -0.00413967365723, 0.00411550333488)
                            .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d drawer_pose;
  drawer_pose.setIdentity();
  drawer_pose.translation() =
      Eigen::Vector3d(1.50129680353, -0.13389706573, 0.793880839785);
  
  drawer_pose.linear() =
      Eigen::Quaterniond(-0.000409283756189, -0.00015641417181, -0.000803996852475, 0.999999580805)
          .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "drawer_world_joint", drawer_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);

  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();
  box_pose.translation() =
      Eigen::Vector3d(-0.251640627693, -0.0127567861381, 0.817309686325);
  box_pose.linear() = Eigen::Quaterniond(0.488104597967, 0.506340876355, -0.493232594684, 0.511951585521)
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
    destination.translation() = Eigen::Vector3d(1.41349036801, -0.132021123133, 0.957045261838);
    destination.linear() = Eigen::Quaterniond(0.52374149293, 0.516252535022, -0.47064074553, 0.487519699412).matrix();
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
    // pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
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
      {0.653281482438188, -0.270598050073098, -0.653281482438188,
       0.270598050073099},
      // {1,0,0,0},
      // {0.5,0.5,-0.5,-0.5},
      true});
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_box",
      "box_box_base_link",
      "box_box_base_link",
      {0.22, 0.000, 0.00},
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
