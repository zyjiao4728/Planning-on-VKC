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
  std::string base_link = env.getVKCEnv()
                              ->getTesseractNonInverse()
                              ->getJoint(world_joint_name)
                              ->child_link_name;
  auto transform = env.getVKCEnv()->getTesseract()->getRelativeLinkTransform(
      "world", base_link);
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      world_joint_name, transform * adjust.inverse());

  // Eigen::Isometry3d base_link_tf =
  //     env.getVKCEnv()->getTesseract()->getLinkTransform(base_link_name);
  // auto cmd =
  // std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
  //     world_joint_name, base_link_tf * adjust.inverse());

  env.getVKCEnv()->getTesseractNonInverse()->applyCommand(cmd);
  std::cout << transform.translation() << std::endl;

  std::cout << "non inverse: \n"
            << env.getVKCEnv()
                   ->getTesseractNonInverse()
                   ->getLinkTransform(base_link)
                   .translation()
            << std::endl;
  return;
}

void setJointAngle(VKCEnvBasic &env, std::string joint_name,
                   std::string world_joint, double joint_val) {
  Eigen::VectorXd joint_values(1);
  joint_values[0] = joint_val;
  env.getVKCEnv()->getTesseract()->setState({joint_name}, joint_values);
  env.getVKCEnv()->getTesseractNonInverse()->setState({joint_name},
                                                      -1 * joint_values);
  CONSOLE_BRIDGE_logDebug("update env success");
  auto tip_link =
      env.getVKCEnv()->getTesseract()->getJoint(world_joint)->child_link_name;
  std::cout << tip_link << std::endl << std::endl;
  auto base_link = env.getVKCEnv()
                       ->getTesseractNonInverse()
                       ->getJoint(world_joint)
                       ->child_link_name;
  auto transform =
      env.getVKCEnv()->getTesseractNonInverse()->getLinkTransform(tip_link);
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      world_joint, transform);
  std::cout << tip_link << std::endl;
  std::cout << transform.translation() << std::endl;
  std::cout << env.getVKCEnv()
                   ->getTesseract()
                   ->getLinkTransform(tip_link)
                   .translation()
            << std::endl;
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
}

ActionSeq getDemoSeq(const std::string robot) {
  ActionSeq actions;
  // pick stick
  {
    auto action = std::make_shared<PickAction>(robot, "attach_stick");
    actions.emplace_back(action);
  }
  {
    auto action = std::make_shared<PickAction>(robot, "attach_closet_top");
    actions.push_back(action);
  }
  return actions;
}

void run(VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter,
         bool rviz_enabled, unsigned int nruns) {
  int window_size = 1;
  LongHorizonSeedGenerator seed_generator(n_iter, window_size, 9);
  ProbGenerator prob_generator;
  seed_generator.setMapInfo(6, 6, 0.3);

  Eigen::Isometry3d cabinet_handle_pose;
  cabinet_handle_pose.setIdentity();
  cabinet_handle_pose.translation() =
      Eigen::Vector3d(1.8322661808, -0.111083378972, 1.15892128103 + 1.17);

  cabinet_handle_pose.linear() =
      Eigen::Quaterniond(0.0373512515579, 0.000793829050033, 0.00416189144533,
                         0.99929321648)
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
  std::cout << env.getVKCEnv()
                   ->getTesseractNonInverse()
                   ->getLinkTransform("closet_top_right_handle")
                   .translation()
            << std::endl;
  // setJointAngle(env, "closet_top_right_door_joint", "closet_base_joint",
  // -1.5);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  //   Eigen::Isometry3d table_interactive_pose;
  //   table_interactive_pose.setIdentity();
  //   table_interactive_pose.translation() =
  //       Eigen::Vector3d(0.171295140475, -3.43593165704, 1.04112748758);

  //   table_interactive_pose.linear() =
  //       Eigen::Quaterniond(0.575062976058, 0.412359288219, -0.410486054582,
  //       0.5751205004)
  //           .matrix();
  //   auto cmd =
  //   std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
  //       "foldable_table_table_base_joint", table_interactive_pose);
  //   env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  //   Eigen::Isometry3d InverseModelPoseAdjust;
  //   InverseModelPoseAdjust.setIdentity();
  // //   InverseModelPoseAdjust.translation() = Eigen::Vector3d(0.5745, 1.1395,
  // 0.085);
  // //   InverseModelPoseAdjust.linear() = Eigen::Quaterniond(1, 0, 0,
  // 0).matrix();
  //   setObjectPose(env, "foldable_table_base_link",
  //   "foldable_table_table_base_joint",
  //                 InverseModelPoseAdjust);
  // Eigen::Isometry3d chair_interactive_pose;
  // chair_interactive_pose.setIdentity();
  // chair_interactive_pose.translation() =
  //     Eigen::Vector3d(-0.593504205005, -1.89276314108, 0.764137691402);

  // chair_interactive_pose.linear() =
  //     Eigen::Quaterniond(-0.0072840637032, 0.00620284798162, 0.0018296048984,
  //                        0.999952558694)
  //         .matrix();
  // auto cmd =
  // std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
  //     "world_chair_chair", chair_interactive_pose);
  // env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  // Eigen::Isometry3d InverseModelPoseAdjust;
  // InverseModelPoseAdjust.setIdentity();

  // setObjectPose(env, "chair_base_link", "world_chair_chair",
  //               InverseModelPoseAdjust);

  int j = 0;

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    seed_generator.generate(env, sub_actions);

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      tesseract_planning::PlannerRequest prob_ptr;
      if (j == 2 || j == 8) {
        prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);
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
                            "/trajectory/tieta_tablechairenv_" +
                            action->Name() + buf + ".csv";

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
  place_coeff << 1, 1, 1, 1, 1, 1, 1, 1, 1;
  // place_coeff << 2, 2, 5, 8, 1, 1, 10, 1, 1;

  // action1: pick
  {
    auto pick_action = std::make_shared<PickAction>(robot, "attach_chair");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

    pick_action->setIKCostCoeff(pick_coeff);
    actions.emplace_back(pick_action);
  }

  // action2: place
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() =
        Eigen::Vector3d(-0.303397584258, -1.6476948177, 0.761020196958);
    destination.linear() = Eigen::Quaterniond(0.707213263773, 0.000927994070908,
                                              6.1097000184e-05, 0.706999670889)
                               .matrix();
    link_objectives.push_back(LinkDesiredPose("chair_base_link", destination));

    auto place_action = std::make_shared<PlaceAction>(
        robot, "attach_chair", link_objectives, joint_objectives, false);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  // action3: pick
  {
    auto pick_action = std::make_shared<PickAction>(robot, "attach_chair");
    pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

    pick_action->setIKCostCoeff(pick_coeff);
    actions.emplace_back(pick_action);
  }

  // action4: place
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    Eigen::Isometry3d destination;
    destination.setIdentity();
    destination.translation() =
        Eigen::Vector3d(-0.248122554727, -2.86443359278, 0.751175945417);
    destination.linear() = Eigen::Quaterniond(0.729212060959, -0.00107033903696,
                                              0.000662264311468, 0.684286625569)
                               .matrix();
    link_objectives.push_back(LinkDesiredPose("chair_base_link", destination));

    auto place_action = std::make_shared<PlaceAction>(
        robot, "attach_chair", link_objectives, joint_objectives, false);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    actions.emplace_back(place_action);
  }

  return actions;
}

void genEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                        UrdfSceneEnv::InverseChainsInfos &inverse_chains) {
  //   attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
  //       "attach_closet_right_handle",
  //       "closet_bottom_right_handle",
  //       "closet_base_link",
  //       {0.25, 0.000, 0.00},
  //       {0.6532815, 0.2705981, -0.6532815, -0.2705981},
  //       // {0.923879532511287,0,0,0.382683432365090},
  //       // {0.5,0.5,-0.5,-0.5},
  //       true});
  // attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
  //     "attach_chair",
  //     "chair_chair_interactive_part",
  //     "chair_base_link",
  //     {0.22, 0.0, 0.00},
  //     {0.6532815, 0.2705981, -0.6532815, -0.2705981},
  //     // {0.923879532511287,0,0,0.382683432365090},
  //     // {0.5,0.5,-0.5,-0.5},
  //     true});
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_stick",
      "stick_base",
      "stick_base_link",
      {0.12, 0.26, 0.},
      {0.6532815, 0.6532815, 0.2705981, -0.2705981}});
  attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
      "attach_closet_top",
      "closet_top_right_handle",
      "closet_base_link",
      {0.11, 0.25, -0.35},  // dist to closet plane / offset from handle /
                            // height from handle
      {0, 0.8225021, 0, -0.5687621}});  // xyz:[ x: 3.1415927, y: -1.21, z: 0 ]
  //   attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
  //       "attach_drawer_handle1",
  //       "drawer_handle1",
  //       "drawer_base_link",
  //       {0.26, 0.000, 0.00},
  //       // {0.707106781186548, 0, -0.707106781186548, 0},
  //       {0.653281482438188, -0.270598050073098, -0.653281482438188,
  //        0.270598050073099},
  //       // {1,0,0,0},
  //       // {0.5,0.5,-0.5,-0.5},
  //       true});
  //   attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
  //       "attach_box",
  //       "box_box_base_link",
  //       "box_box_base_link",
  //       {0.21, 0.000, 0.00},
  //       // {0.707106781186548, 0, -0.707106781186548, 0},
  //       {0.6532815, 0.2705981, -0.6532815, -0.2705981},
  //       // {1,0,0,0},
  //       // {0.5,0.5,-0.5,-0.5},
  //       false});
  //   inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
  //       "closet_base_link", "closet_bottom_right_handle"});
  //   inverse_chains.emplace_back(
  //       UrdfSceneEnv::InverseChainsInfo{"drawer_base_link",
  //       "drawer_handle1"});
  //   inverse_chains.emplace_back(
  //       UrdfSceneEnv::InverseChainsInfo{"foldable_table_base_link",
  //       "foldable_table_table_interactive"});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "closet_base_link", "closet_top_right_handle"});
  inverse_chains.emplace_back(
      UrdfSceneEnv::InverseChainsInfo{"stick_base_link", "stick_base"});
  // inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
  //     "chair_base_link", "chair_chair_interactive_part"});
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
      "right_gripper_finger_1_link_0", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_1_link_1", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_1_link_2", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_1_link_3", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_2_link_0", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_2_link_1", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_2_link_2", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_2_link_3", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_middle_link_0", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_middle_link_1", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_middle_link_2", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_finger_middle_link_3", "chair_chair_interactive_part",
      "Always"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "right_gripper_palm", "chair_chair_interactive_part", "Always"));

  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_0", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_1", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_2", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_3", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_0", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_1", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_2", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_3", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_0", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_1", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_2", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_3", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_palm", "chair_base_link", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_flange", "chair_base_link", "Always"));

  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_0", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_1", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_2", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_1_link_3", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_0", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_1", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_2", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_2_link_3", "foldable_table_table_plane_below",
  //       "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_0",
  //       "foldable_table_table_plane_below", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_1",
  //       "foldable_table_table_plane_below", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_2",
  //       "foldable_table_table_plane_below", "Always"));
  //   cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
  //       "right_gripper_finger_middle_link_3",
  //       "foldable_table_table_plane_below", "Always"));

  env.getVKCEnv()->getTesseract()->applyCommands(cmds);

  // ActionSeq actions = getTietaEnvSeq("vkc");
  ActionSeq actions = getDemoSeq("vkc");

  run(env, actions, steps, n_iter, rviz, nruns);
}
