#include <vkc/env/arena_env.h>

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
ArenaEnv::ArenaEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps)
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

bool ArenaEnv::createEnvironment() {
  // double arena_x = 8.0;
  double arena_y = 8.0;
  // double arena_z = 2.1;

  vkc::BaseTable table0("table0", 0.8, 3, 1.6);
  table0.createObject();
  table0.createWorldJoint(Eigen::Vector4d(-0.5, arena_y / 2.0 - 0.5, 0, 0));

  vkc::BaseCabinet cabinet0("cabinet0");
  cabinet0.createObject();
  cabinet0.createWorldJoint(Eigen::Vector4d(0, -arena_y / 2.0 + 1.7, 0, -1.57));
  cabinet0.inverseRootTip("world", cabinet0.getName() + "_handle_link");

  vkc::BaseStick stick0("stick0", 0.5);
  stick0.createObject();
  stick0.createWorldJoint(Eigen::Vector4d(2.5, 1, 0.8, -1.57));
  stick0.inverseRootTip("world", stick0.getName() + "_stick_link");

  vkc::BaseMarker marker0("marker0", 0.2);
  marker0.createObject();
  marker0.createWorldJoint(Eigen::Vector4d(-1, 4, 0.1, 1.57));
  marker0.inverseRootTip("world", marker0.getName() + "_marker_link");

  vkc::BaseTable table1("table1", 0.72, 1, 2);
  table1.createObject();
  table1.createWorldJoint(Eigen::Vector4d(2.5, 1, 0, 0));

  //   vkc::BaseWall obs1("obs1", 2, 3, 2.1);
  //   obs1.setColor(Eigen::Vector4d(0.5, 0.5, 0.5, 1));
  //   obs1.createObject();
  //   obs1.createWorldJoint(Eigen::Vector4d(2, -2, 0, 0));

  ROS_ERROR("something wrong");
  table0.addToEnvironment(tesseract_->getTesseract());
  cabinet0.addToEnvironment(tesseract_->getTesseract());
  stick0.addToEnvironment(tesseract_->getTesseract());
  marker0.addToEnvironment(tesseract_->getTesseract());
  table1.addToEnvironment(tesseract_->getTesseract());

  updateAttachLocations(cabinet0.getAttachLocations());
  updateAttachLocations(marker0.getAttachLocations());
  updateAttachLocations(stick0.getAttachLocations());

  Commands cmds;
  cmds.clear();
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_upper_arm_link", "ur_arm_wrist_1_link", "Never"));

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_upper_arm_link", "ur_arm_wrist_2_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_upper_arm_link", "ur_arm_wrist_3_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_forearm_link", "ur_arm_wrist_3_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_base_link", "ur_arm_upper_arm_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_door_link", "ur_arm_wrist_3_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "stick0_stick_link", "ur_arm_wrist_3_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_ee_link", "ur_arm_upper_arm_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "ur_arm_ee_link", "ur_arm_forearm_link", "Never"));

  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "cabinet0_knob1_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "cabinet0_knob2_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "cabinet0_handle_link", false));
  ;

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_bottom_link", "cabinet0_cabinet_door_link", "Never"));

  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_cabinet_bottom_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_level1_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_level2_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_level3_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_back_link", "cabinet0_top_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_bottom_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_bottom_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level1_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level2_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level3_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level1_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level2_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level3_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_top_link", "cabinet0_cabinet_right_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level1_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level2_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_level3_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_top_link", "cabinet0_cabinet_left_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_left_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_right_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_top_link", "cabinet0_cabinet_door_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_door_link", "cabinet0_knob1_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_cabinet_door_link", "cabinet0_knob2_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_handle_link", "cabinet0_knob1_link", "Never"));
  cmds.push_back(std::make_shared<AddAllowedCollisionCommand>(
      "cabinet0_handle_link", "cabinet0_knob2_link", "Never"));

  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "ur_arm_ee_link", false));
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "user_rail_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "top_chassis_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "top_plate_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "front_bumper_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "front_left_wheel_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "front_right_wheel_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "rear_bumper_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "rear_left_wheel_link", false));
  ;
  cmds.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(
      "rear_right_wheel_link", false));
  ;

  tesseract_->getTesseract()->applyCommands(cmds);

  return true;
}
}  // namespace vkc
