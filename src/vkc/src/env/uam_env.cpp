#include <vkc/env/uam_env.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

// URDF and SRDF file describes environment and robot
const std::string ENV_DESCRIPTION_PARAM = "env_description";
const std::string ENV_SEMANTIC_PARAM = "env_description_semantic";

// Link name defined as end effector
const std::string END_EFFECTOR_LINK = "end_effector_link";

// RVIZ service
const std::string GET_ENVIRONMENT_CHANGES_SERVICE =
    "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

namespace vkc {
UAMEnv::UAMEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps)
    : VKCEnvBasic(nh, plotting, rviz, steps_) {
  // Set Log Level
  util::gLogLevel = util::LevelDebug;

  loadRobotModel(ENV_DESCRIPTION_PARAM, ENV_SEMANTIC_PARAM, END_EFFECTOR_LINK);

  initTesseractConfig();

  // set robot initial pose in scene graph
  setHomePose();

  ROS_INFO("Sucessfully load the robot model, now creating environment...");

  createEnvironment();

  ROS_INFO(
      "Sucessfully create the environment, now creating optimization "
      "problem...");
}

bool UAMEnv::createEnvironment() {
  vkc::BaseMarker marker0("lightbulb0", 0.05);
  marker0.createObject();
  marker0.createWorldJoint(Eigen::Vector4d(1, 1, 0.5, 0.0));
  marker0.inverseRootTip("world", marker0.getName() + "_marker_link");

  vkc::BaseTable table0("table0", 0.45, 1, 1);
  table0.createObject();
  table0.createWorldJoint(Eigen::Vector4d(1,1,0,0));

  vkc::BaseWall wall_top("wall_top", 1, 1, 0.1);
  wall_top.createObject();
  wall_top.createWorldJoint(Eigen::Vector4d(1, 1, 2.1, 0));

  marker0.addToEnvironment(tesseract_->getTesseract());
  table0.addToEnvironment(tesseract_->getTesseract());
  wall_top.addToEnvironment(tesseract_->getTesseract());

  updateAttachLocations(marker0.getAttachLocations());

  Commands cmds;
  cmds.clear();
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "arm_link_II", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "crazyflie11/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "crazyflie22/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "crazyflie33/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_I", "crazyflie44/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_II", "crazyflie11/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_II", "crazyflie33/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "arm_link_II", "gripper_base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "base_link", "crazyflie11/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "base_link", "crazyflie22/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "base_link", "crazyflie33/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "base_link", "crazyflie44/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie11/base_link", "crazyflie22/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie11/base_link", "crazyflie33/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie11/base_link", "crazyflie44/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie11/base_link", "gripper_base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie22/base_link", "crazyflie33/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie22/base_link", "crazyflie44/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie33/base_link", "crazyflie44/base_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "crazyflie33/base_link", "gripper_base_link", "Never"));

  tesseract_->getTesseract()->applyCommands(cmds);

  return true;
}
}  // namespace vkc
