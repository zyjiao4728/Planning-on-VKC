#include <vkc/env/open_door_env.h>

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
OpenDoorEnv::OpenDoorEnv(ros::NodeHandle nh, bool plotting, bool rviz,
                         int steps)
    : VKCEnvBasic(nh, plotting, rviz, steps) {
  // Set Log Level
  util::gLogLevel = util::LevelInfo;

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

bool OpenDoorEnv::createEnvironment() {
  double arena_x = 10.0;
  double arena_y = 10.0;
  double arena_z = 2.06;

  double wall_thickness = 0.1;
  double door_width = 1.06;

  vkc::BaseWall wall_north_left("wall_north_left", wall_thickness,
                                (arena_y - door_width) / 2.0, arena_z);
  wall_north_left.createObject();
  wall_north_left.createWorldJoint(Eigen::Vector4d(
      arena_x / 2.0, (arena_y - door_width) / 4.0 + door_width / 2.0, 0, 0));

  vkc::BaseWall wall_north_right("wall_north_right", wall_thickness,
                                 (arena_y - door_width - 0.1) / 2.0, arena_z);
  wall_north_right.createObject();
  wall_north_right.createWorldJoint(Eigen::Vector4d(
      arena_x / 2.0, -((arena_y - door_width) / 4.0 + door_width / 2.0), 0, 0));

  vkc::BaseWall wall_west("wall_west", arena_x + wall_thickness, wall_thickness,
                          arena_z);
  wall_west.createObject();
  wall_west.createWorldJoint(Eigen::Vector4d(0, arena_y / 2.0, 0, 0));

  vkc::BaseWall wall_east("wall_east", arena_x + wall_thickness, wall_thickness,
                          arena_z);
  wall_east.createObject();
  wall_east.createWorldJoint(Eigen::Vector4d(0, -arena_y / 2.0, 0, 0));

  vkc::BaseWall wall_south("wall_south", wall_thickness, arena_y, arena_z);
  wall_south.createObject();
  wall_south.createWorldJoint(Eigen::Vector4d(-arena_x / 2.0, 0, 0, 0));

  vkc::BaseDoor door_north("door_north", door_width, "right");
  door_north.createObject();
  door_north.createWorldJoint(
      Eigen::Vector4d(arena_x / 2.0, door_width / 2, 0.0, 0));
  door_north.inverseRootTip("world", door_north.getName() + "_handle_link");

  // vkc::BaseDrawer drawer0("drawer0");
  // drawer0.createObject();
  // drawer0.createWorldJoint(Eigen::Vector4d(2, -arena_y / 2.0 + 1.3,
  // 0.2, 1.57)); drawer0.inverseRootTip("world", drawer0.getName() +
  // "_handle_link");

  addAttachLocations(door_north.getAttachLocation());

  ROS_INFO("add attach location success.");

  wall_north_left.addToEnvironment(tesseract_->getTesseract());
  wall_north_right.addToEnvironment(tesseract_->getTesseract());
  wall_west.addToEnvironment(tesseract_->getTesseract());
  wall_east.addToEnvironment(tesseract_->getTesseract());
  wall_south.addToEnvironment(tesseract_->getTesseract());
  door_north.addToEnvironment(tesseract_->getTesseract());

  ROS_INFO("add objects to environment success");

  // drawer0.addToEnvironment(tesseract_->getTesseract());

  plotter_->waitForInput("wait for environment to update");

  // Disable minor collision detection

  Commands cmds;
  cmds.clear();
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_north_door_link", "wall_north_right_wall_link", "Never"));

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_north_door_link", "wall_north_left_wall_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_north_door_link", "door_north_handle_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "robotiq_arg2f_base_link", "door_north_handle_link", "Never"));

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "wall_west_wall_link", "wall_north_left_wall_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "wall_east_wall_link", "wall_north_right_wall_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "wall_west_wall_link", "wall_south_wall_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "wall_east_wall_link", "wall_south_wall_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "door_north_handle_link", "left_gripper_palm", "Never"));

  ROS_INFO("applying add collision commands");

  tesseract_->getTesseract()->applyCommands(cmds);

  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_handle_left",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_handle_right",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_handle_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_left_wall",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_right_wall",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_top_wall",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_bottom_wall",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("drawer0_back_wall",
  // false);

  // tesseract_->getTesseract()->setLinkCollisionEnabled("ur_arm_ee_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("user_rail_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("top_chassis_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("top_plate_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("front_bumper_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("front_left_wheel_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("front_right_wheel_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("rear_bumper_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("rear_left_wheel_link",
  // false);
  // tesseract_->getTesseract()->setLinkCollisionEnabled("rear_right_wheel_link",
  // false);

  return true;
}
}  // namespace vkc