#include <ros/package.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>

using namespace vkc;
using namespace tesseract_planning;

void setObjectPose(VKCEnvBasic &env, std::string base_link_name,
                   std::string world_joint_name, Eigen::Isometry3d adjust) {
  Eigen::Isometry3d base_link_tf =
      env.getVKCEnv()->getTesseract()->getLinkTransform(base_link_name);
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      world_joint_name, base_link_tf * adjust.inverse());
  env.getVKCEnv()->getTesseractNonInverse()->applyCommand(cmd);
  return;
}

void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter,
         bool rviz_enabled, unsigned int nruns) {
  int window_size = 2;
  LongHorizonSeedGenerator seed_generator(n_iter, window_size, 9);
  ProbGenerator prob_generator;
  seed_generator.setMapInfo(6, 6, 0.3);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  Eigen::Isometry3d cabinet_handle_pose;
  cabinet_handle_pose.setIdentity();
  cabinet_handle_pose.translation() =
      Eigen::Vector3d(1.6008290705, -0.119028821604, 1.16077205044);

  cabinet_handle_pose.linear() =
      Eigen::Quaterniond(0.01213719228, 0.00147056921893, 0.00352577932278, 0.999919044158)
          .matrix();
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "closet_base_joint", cabinet_handle_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  Eigen::Isometry3d InverseModelPoseAdjust;
  InverseModelPoseAdjust.setIdentity();
  InverseModelPoseAdjust.translation() = Eigen::Vector3d(0.5745, 1.1395, 0.085);
  InverseModelPoseAdjust.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();
  setObjectPose(env, "closet_base_link", "closet_base_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d table_pose;
  table_pose.setIdentity();
  table_pose.translation() =
      Eigen::Vector3d(-0.000741231206823, -0.00100066802074, 0.778162612167);
  table_pose.linear() = Eigen::Quaterniond(0.999926188341, -0.00112456813974, -2.28605265373e-05, -0.0120976317792)
                            .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "table_table_base_link", "table_table_base_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d drawer_pose;
  drawer_pose.setIdentity();
  drawer_pose.translation() =
      Eigen::Vector3d(1.63689526462, -0.277244723554, 0.797022783282);

  drawer_pose.linear() = Eigen::Quaterniond(0.0166118400848, -0.00847124317006, -0.00158642556722, 0.999824868696)
                             .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "drawer_world_joint", drawer_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "drawer_base_link", "drawer_world_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();
  box_pose.translation() =
      Eigen::Vector3d(0.0220869188096, 0.131105779323, 0.820314461283);
  box_pose.linear() = Eigen::Quaterniond(0.498370590928, 0.4978297229, -0.501032788889, 0.502750898113)
                          .matrix();
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "box_box_base_joint", box_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "box_box_base_link", "box_box_base_joint",
                InverseModelPoseAdjust);
  int j = 0;

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    seed_generator.generate(env, sub_actions);
    action->switchCandidate();

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      tesseract_planning::PlannerRequest prob_ptr;
      if (j == 2 || j == 8) {
        prob_ptr = prob_generator.genRequest(env, action, 50, n_iter);
      } else {
        prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);
      }

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

    j++;
  }
}

ActionSeq getTietaEnvSeq(const std::string robot) {
  ActionSeq actions;

  Eigen::VectorXd pick_coeff(9);
  pick_coeff << 1, 1, 1, 1, 1, 1, 1, 1, 1;

  Eigen::VectorXd place_coeff(9);
  place_coeff << 4, 4, 4, 8, 1, 1, 10, 1, 1;
  // place_coeff << 2, 2, 5, 8, 1, 1, 10, 1, 1;

  // action1: pick closet handle
  {
    auto pick_action =
        std::make_shared<PickAction>(robot, "attach_closet_right_handle");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

    pick_action->setIKCostCoeff(pick_coeff);
    actions.emplace_back(pick_action);
  }

  // action2: place closet handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("closet_bottom_right_door_joint",
                                  -1.7198621771937625);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives, false);
    // place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    place_action->setIKCostCoeff(place_coeff);
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
    place_action->setIKCostCoeff(place_coeff);
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
    destination.translation() =
        Eigen::Vector3d(1.520242745, -0.267358451194, 0.810395824685 + 0.15);
    destination.linear() = Eigen::Quaterniond(0.509883484332, -0.499930970934, -0.491384342801, -0.498627400327)
                               .matrix();
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
    pick_action->setIKCostCoeff(pick_coeff);
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
    place_action->setIKCostCoeff(place_coeff);
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

    joint_objectives.emplace_back("closet_bottom_right_door_joint", -0.05);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives, false);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    place_action->setIKCostCoeff(place_coeff);
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
      {0.25, 0.000, 0.00},
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
      {0.21, 0.000, 0.00},
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

  Commands cmds;
  cmds.clear();
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "coke_can0_coke_can", "kortex_robotiq_arg2f_base_link", "Never"));


  env.getVKCEnv()->getTesseract()->applyCommands(cmds);


  ActionSeq actions = getTietaEnvSeq("vkc");

  run(env, actions, steps, n_iter, rviz, nruns);
}
