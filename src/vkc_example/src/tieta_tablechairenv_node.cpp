#include <ros/package.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

using namespace vkc;
using namespace tesseract_planning;

Eigen::Isometry3d get_object_pose(ros::NodeHandle nh, std::string object_name)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(0.5).sleep();

  while (nh.ok())
  {
    geometry_msgs::TransformStamped trans_world2obj;
    try
    {
      trans_world2obj = tfBuffer.lookupTransform("world", "vicon/" + object_name + "/" + object_name,
                                                 ros::Time(0));
      std::cout << 'transformation is :' << trans_world2obj << std::endl;
      Eigen::Isometry3d object_eigen3d;
      object_eigen3d.setIdentity();
      object_eigen3d.translation() = Eigen::Vector3d(trans_world2obj.transform.translation.x,
                                                     trans_world2obj.transform.translation.y, trans_world2obj.transform.translation.z);
      object_eigen3d.linear() = Eigen::Quaterniond(trans_world2obj.transform.rotation.w, trans_world2obj.transform.rotation.x,
                                                   trans_world2obj.transform.rotation.y, trans_world2obj.transform.rotation.z)
                                    .matrix();
      return object_eigen3d;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
}

void setObjectPose(VKCEnvBasic &env, std::string base_link_name,
                   std::string world_joint_name, Eigen::Isometry3d adjust)
{
  Eigen::Isometry3d base_link_tf =
      env.getVKCEnv()->getTesseract()->getLinkTransform(base_link_name);
  auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      world_joint_name, base_link_tf * adjust.inverse());
  env.getVKCEnv()->getTesseractNonInverse()->applyCommand(cmd);
  return;
}

void run(ros::NodeHandle nh, VKCEnvBasic &env, ActionSeq &actions, int n_steps, int n_iter,
         bool rviz_enabled, unsigned int nruns)
{
  int window_size = 2;
  LongHorizonSeedGenerator seed_generator(n_iter, window_size, 9);
  ProbGenerator prob_generator;
  seed_generator.setMapInfo(6, 6, 0.3);

  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  Eigen::Isometry3d cabinet_handle_pose;
  cabinet_handle_pose = get_object_pose(nh, "closet_hl");
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
  table_pose = get_object_pose(nh, "table");
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "table_table_base_link", "table_table_base_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d table2_pose;
  table_pose = get_object_pose(nh, "table2");
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table2_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "table2_table_base_link", "table2_table_base_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d table3_pose;
  table_pose = get_object_pose(nh, "table3");
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "table3_table_base_joint", table_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "table3_table_base_link", "table3_table_base_joint",
                InverseModelPoseAdjust);

  Eigen::Isometry3d cup_pose;
  cup_pose = get_object_pose(nh, "cup");
  cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
      "cup_cup_base_joint", cup_pose);
  env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  InverseModelPoseAdjust.setIdentity();
  setObjectPose(env, "cup_cup_base_link", "cup_cup_base_joint",
                InverseModelPoseAdjust);

  //   Eigen::Isometry3d drawer_pose;
  //   drawer_pose.setIdentity();
  //   drawer_pose.translation() =
  //       Eigen::Vector3d(1.63689526462, -0.277244723554, 0.797022783282);

  //   drawer_pose.linear() = Eigen::Quaterniond(0.0166118400848, -0.00847124317006, -0.00158642556722, 0.999824868696)
  //                              .matrix();
  //   cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(
  //       "drawer_world_joint", drawer_pose);
  //   env.getVKCEnv()->getTesseract()->applyCommand(cmd);
  //   InverseModelPoseAdjust.setIdentity();
  //   setObjectPose(env, "drawer_base_link", "drawer_world_joint",
  //                 InverseModelPoseAdjust);

  int j = 0;

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++)
  {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    // seed_generator.generate(env, sub_actions);

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns)
    {
      tesseract_planning::PlannerRequest prob_ptr;
      if (j == 2 || j == 8)
      {
        prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);
      }
      else
      {
        prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);
      }

      if (rviz_enabled)
      {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }
      solveProb(prob_ptr, response, n_iter);

      // break;
      if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
          response.status.value()) // optimization converges
      {
        converged = true;
        break;
      }
      else
      {
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

    if (rviz_enabled && env.getPlotter() != nullptr)
    {
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

    if (env.getPlotter() != nullptr && rviz_enabled)
      env.getPlotter()->clear();

    j++;
  }
}

ActionSeq getTietaEnvSeq(const std::string robot)
{
  ActionSeq actions;

  Eigen::VectorXd pick_coeff(9);
  pick_coeff << 1, 1, 1, 1, 1, 1, 1, 1, 1;

  Eigen::VectorXd place_coeff(9);
  place_coeff << 3, 3, 3, 8, 1, 1, 10, 1, 1;
  // place_coeff << 2, 2, 5, 8, 1, 1, 10, 1, 1;

  {
    Eigen::VectorXd state(9);
    state << 0.70321, -0.673876, 1.3821, 1.17036, 0.903486, -1.52721, 1.97366, -0.804571, 2.78667;
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"};
    auto action = std::make_shared<GotoAction>(robot, joint_names, state, "goto handle");
    actions.emplace_back(action);
  }

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
                                  -1.5);
    auto place_action =
        std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
                                      link_objectives, joint_objectives);
    place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
    place_action->setIKCostCoeff(place_coeff);
    place_action->loadTrajectorySeed("/home/y/Documents/WYY_META/Planning-on-VKC/src/vkc_example/trajectory/tieta_env_PlaceAction0.csv");
    actions.emplace_back(place_action);
  }

  {
    Eigen::VectorXd state(9);
    state << 1.0237,-1.529, 1.61232, 2.42, 0.28, -2.3, 1.40, 0.03, 1.15;
    std::vector<std::string> joint_names = {"base_y_base_x", "base_theta_base_y", "base_link_base_theta", "right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"};
    auto action = std::make_shared<GotoAction>(robot, joint_names, state, "goto byeye");
    actions.emplace_back(action);
  }

  // // action3: pick cup
  // {
  //   auto pick_action = std::make_shared<PickAction>(robot, "attach_cup");
  //   pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //   actions.emplace_back(pick_action);
  // }

  // // action4: place cup
  // {
  //   std::vector<LinkDesiredPose> link_objectives;
  //   std::vector<JointDesiredPose> joint_objectives;
  //   Eigen::Isometry3d destination;
  //   destination.setIdentity();
  //   destination.translation() =
  //       Eigen::Vector3d(1.47093204225, -0.20089658102, 1.06764651278);
  //   destination.linear() = Eigen::Quaterniond(0.13752651062, 0.0074258581583, 0.00112351909497, 0.990469612463)
  //                              .matrix();
  //   link_objectives.push_back(
  //       LinkDesiredPose("cup_cup_base_link", destination));

  //   auto place_action = std::make_shared<PlaceAction>(
  //       robot, "attach_cup", link_objectives, joint_objectives);
  //   place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //   actions.emplace_back(place_action);
  // }

  //   // action3: pick drawer handle
  //   {
  //     auto pick_action =
  //         std::make_shared<PickAction>(robot, "attach_drawer_handle1");
  //     pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //     actions.emplace_back(pick_action);
  //   }

  //   // action4: place drawer handle
  //   {
  //     std::vector<LinkDesiredPose> link_objectives;
  //     std::vector<JointDesiredPose> joint_objectives;

  //     joint_objectives.emplace_back("drawer_base_drawer1_joint", -0.22);
  //     auto place_action =
  //         std::make_shared<PlaceAction>(robot, "attach_drawer_handle1",
  //                                       link_objectives, joint_objectives, false);
  //     place_action->setIKCostCoeff(place_coeff);
  //     place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //     actions.emplace_back(place_action);
  //   }

  //   // action7: pick drawer handle
  //   {
  //     auto pick_action =
  //         std::make_shared<PickAction>(robot, "attach_drawer_handle1");
  //     pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //     pick_action->setIKCostCoeff(pick_coeff);
  //     actions.emplace_back(pick_action);
  //   }

  //   // action8: place drawer handle
  //   {
  //     std::vector<LinkDesiredPose> link_objectives;
  //     std::vector<JointDesiredPose> joint_objectives;

  //     joint_objectives.emplace_back("drawer_base_drawer1_joint", 0.0);
  //     auto place_action =
  //         std::make_shared<PlaceAction>(robot, "attach_drawer_handle1",
  //                                       link_objectives, joint_objectives, false);

  //     place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");

  //     place_action->setIKCostCoeff(place_coeff);
  //     actions.emplace_back(place_action);
  //   }

  // // action9: pick closet handle
  // {
  //   auto pick_action =
  //       std::make_shared<PickAction>(robot, "attach_closet_right_handle");
  //   pick_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //   actions.emplace_back(pick_action);
  // }

  // // action10: place closet handle
  // {
  //   std::vector<LinkDesiredPose> link_objectives;
  //   std::vector<JointDesiredPose> joint_objectives;

  //   joint_objectives.emplace_back("closet_bottom_right_door_joint", -0.05);
  //   auto place_action =
  //       std::make_shared<PlaceAction>(robot, "attach_closet_right_handle",
  //                                     link_objectives, joint_objectives);
  //   place_action->setBaseJoint("base_y_base_x", "base_theta_base_y");
  //   place_action->setIKCostCoeff(place_coeff);
  //   actions.emplace_back(place_action);
  // }

  return actions;
}

void genEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                        UrdfSceneEnv::InverseChainsInfos &inverse_chains)
{
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
      "attach_cup",
      "cup_cup_base_link",
      "cup_cup_base_link",
      {0.21, 0.000, 0.00},
      // {0.707106781186548, 0, -0.707106781186548, 0},
      {0.6532815, 0.2705981, -0.6532815, -0.2705981},
      // {1,0,0,0},
      // {0.5,0.5,-0.5,-0.5},
      false});
  //   attaches.emplace_back(UrdfSceneEnv::AttachObjectInfo{
  //       "attach_drawer_handle1",
  //       "drawer_handle1",
  //       "drawer_base_link",
  //       {0.26, 0.000, 0.00},
  //       // {0.707106781186548, 0, -0.707106781186548, 0},
  //       {0.653281482438188, -0.270598050073098, -0.653281482438188,
  //        0.270598050073099},
  // {1,0,0,0},
  // {0.5,0.5,-0.5,-0.5},
  //   true});
  inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
      "closet_base_link", "closet_bottom_right_handle"});
  //   inverse_chains.emplace_back(
  //       UrdfSceneEnv::InverseChainsInfo{"drawer_base_link", "drawer_handle1"});
  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

int main(int argc, char **argv)
{
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
  //   std::string object_name="closet_hl";
  //   Eigen::Isometry3d object_pose = get_object_pose(nh, object_name);

  UrdfSceneEnv::AttachObjectInfos attaches;
  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  genEnvironmentInfo(attaches, inverse_chains);

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);

  ActionSeq actions = getTietaEnvSeq("vkc");

  run(nh, env, actions, steps, n_iter, rviz, nruns);
}
