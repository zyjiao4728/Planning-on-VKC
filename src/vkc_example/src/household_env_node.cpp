#include <fmt/ranges.h>
#include <ros/package.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <vkc/action/actions.h>
#include <vkc/env/urdf_scene_env.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/planner/long_horizon.h>
#include <vkc/planner/prob_generator.h>
#include <vkc_example/utils.h>
// motion planning via OMPL
#include <tesseract_collision/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

#include <iostream>
#include <string>
#include <trajopt_utils/eigen_conversions.hpp>
#include <vector>

#include "vkc/planner/prob_translator.h"

using namespace std;
using namespace vkc;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;
using TesseractJointTraj = tesseract_common::JointTrajectory;
// using namespace vkc_example;

std::vector<double> run(vector<TesseractJointTraj> &joint_trajs,
                        VKCEnvBasic &env, ActionSeq &actions, int n_steps,
                        int n_iter, bool rviz_enabled, unsigned int nruns,
                        bool longhorizon = false) {
  int window_size = 3;
  LongHorizonSeedGenerator seed_generator(n_iter, window_size, 9);
  ProbGenerator prob_generator;
  seed_generator.setMapInfo(8, 8, 0.2);

  int j = 0;

  std::vector<double> elapsed_time;
  // env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);

  for (auto ptr = actions.begin(); ptr < actions.end(); ptr++) {
    auto action = *ptr;
    ActionSeq sub_actions(ptr, actions.end());
    auto start = chrono::steady_clock::now();
    if (longhorizon) {
      seed_generator.generate(env, sub_actions);
    }
    auto end = chrono::steady_clock::now();

    double seed_time =
        chrono::duration_cast<chrono::milliseconds>(end - start).count() /
        1000.;

    PlannerResponse response;
    unsigned int try_cnt = 0;
    bool converged = false;
    while (try_cnt++ < nruns) {
      start = chrono::steady_clock::now();
      auto prob_ptr = prob_generator.genRequest(env, action, n_steps, n_iter);

      if (rviz_enabled) {
        env.getPlotter()->waitForInput(
            "optimization is ready. Press <Enter> to process the request.");
      }

      solveProb(prob_ptr, response, n_iter);
      end = chrono::steady_clock::now();
      // break;
      if (TrajOptMotionPlannerStatusCategory::SolutionFound ==
          response.status.value())  // optimization converges
      {
        converged = true;
        elapsed_time.emplace_back(
            seed_time +
            chrono::duration_cast<chrono::milliseconds>(end - start).count() /
                1000.);
        break;
      } else {
        ROS_WARN(
            "[%s]optimization could not converge, response code: %d, "
            "description: %s",
            __func__, response.status.value(),
            response.status.message().c_str());
        if (longhorizon) action->switchCandidate();
      }
    }

    if (!converged) {
      elapsed_time.emplace_back(-1.0);
    }

    const auto &ci = response.results;

    tesseract_common::JointTrajectory trajectory = toJointTrajectory(ci);
    tesseract_common::JointTrajectory refined_traj = trajectory;
    joint_trajs.emplace_back(trajectory);

    if (rviz_enabled && env.getPlotter() != nullptr) {
      ROS_INFO("plotting result");
      refineTrajectory(refined_traj, env);
      tesseract_common::Toolpath toolpath =
          toToolpath(ci, *env.getVKCEnv()->getTesseract());
      env.getPlotter()->plotMarker(
          tesseract_visualization::ToolpathMarker(toolpath));
      env.getPlotter()->plotTrajectory(
          refined_traj, *env.getVKCEnv()->getTesseract()->getStateSolver());
      env.getPlotter()->waitForInput(
          "Finished optimization. Press <Enter> to start next action");
    }

    toDelimitedFile(ci,
                    "/home/jiao/BIGAI/vkc_ws/Planning-on-VKC/src/vkc_example/"
                    "trajectory/household_env_" +
                        std::to_string(j) + ".csv",
                    ',');

    env.updateEnv(trajectory.back().joint_names, trajectory.back().position,
                  action);
    CONSOLE_BRIDGE_logInform("update env finished");
    // const tesseract_srdf::KinematicsInformation kin_info =
    // env.getVKCEnv()->getTesseract()->getKinematicsInformation(); auto cmd =
    // std::make_shared<AddKinematicsInformationCommand>(kin_info);
    // env.getVKCEnv()->getTesseract()->applyCommand(cmd);

    if (env.getPlotter() != nullptr && rviz_enabled) env.getPlotter()->clear();
    ++j;
  }
  return elapsed_time;
}

void sampleInitBasePose(vkc::VKCEnvBasic &env) {
  bool init_base_position = false;
  vector<string> base_joints({"base_y_base_x", "base_theta_base_y"});
  Eigen::VectorXd base_values = Eigen::Vector2d(0., 0.);
  tesseract_collision::ContactResultMap contact_results;

  // env.getVKCEnv()->getTesseract()->setState(base_joints, Eigen::Vector2d(2,-2));
  // return;
  while (!init_base_position) {
    init_base_position = true;
    contact_results.clear();
    base_values = Eigen::Vector2d(rand() % 100 / 99. * 2. + 0.5,
                                  -rand() % 100 / 99. * 3. + 0.5);
    env.getVKCEnv()->getTesseract()->setState(base_joints, base_values);
    env.getVKCEnv()->getTesseract()->getDiscreteContactManager()->contactTest(
        contact_results, tesseract_collision::ContactTestType::ALL);

    for (auto &collision : contact_results) {
      if (collision.first.first == "base_link" ||
          collision.first.second == "base_link") {
        init_base_position = false;
        break;
      }
    }
  }
}

Eigen::VectorXd getPickCoeff(int size = 9) {
  Eigen::VectorXd coeff;
  coeff.setOnes(size);
  return coeff;
}

Eigen::VectorXd getPlaceCoeff(int size = 10) {
  Eigen::VectorXd coeff;
  coeff.setOnes(size);
  coeff[2] = 3.;
  coeff[3] = 3.;
  coeff[6] = 5.;
  coeff[9] = 0;
  return coeff;
}

void genOpenFridgeSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action 1: pick fridge handle
  {
    auto action = make_shared<PickAction>(robot, "attach_fridge_handle");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: open fridge door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint", -1.6);
    auto action =
        make_shared<PlaceAction>(robot, "attach_fridge_handle", link_objectives,
                                 joint_objectives, false);

    setBaseJoint(action);
    auto place_coeff = getPlaceCoeff();
    action->setIKCostCoeff(place_coeff);
    actions.emplace_back(action);
  }
}

void genCloseFridgeSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action 1: pick fridge handle
  {
    auto action = make_shared<PickAction>(robot, "attach_fridge_handle");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: close fridge door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("fridge_0001_dof_rootd_Aa002_r_joint", 0.0);
    auto action =
        make_shared<PlaceAction>(robot, "attach_fridge_handle", link_objectives,
                                 joint_objectives, false);

    setBaseJoint(action);
    auto place_coeff = getPlaceCoeff();
    action->setIKCostCoeff(place_coeff);
    actions.emplace_back(action);
  }
}

void genOpenDoorSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action 1: pick door handle
  {
    auto action = make_shared<PickAction>(robot, "attach_door_handle");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: open door door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_8966_joint_1", 1.52);
    auto action = make_shared<PlaceAction>(
        robot, "attach_door_handle", link_objectives, joint_objectives, false);

    setBaseJoint(action);
    auto place_coeff = getPlaceCoeff();
    action->setIKCostCoeff(place_coeff);

    actions.emplace_back(action);
  }
}

void genCloseDoorSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action 1: pick door handle
  {
    auto action = make_shared<PickAction>(robot, "attach_door_handle");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: close door door
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("door_8966_joint_1", 0.0);
    auto action = make_shared<PlaceAction>(
        robot, "attach_door_handle", link_objectives, joint_objectives, false);

    setBaseJoint(action);
    auto place_coeff = getPlaceCoeff();
    action->setIKCostCoeff(place_coeff);

    actions.emplace_back(action);
  }
}

void genMoveCupSeq(vkc::ActionSeq &actions, const std::string &robot,
                   Eigen::Isometry3d destination) {
  // action 1: pick cup
  {
    auto action = make_shared<PickAction>(robot, "attach_cup");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: place cup on the table
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    link_objectives.push_back(
        LinkDesiredPose("cup_cup_base_link", destination));

    auto action = make_shared<PlaceAction>(robot, "attach_cup", link_objectives,
                                           joint_objectives, true);

    setBaseJoint(action);
    actions.emplace_back(action);
  }
}

void genThrowTrashSeq(vkc::ActionSeq &actions, const std::string &robot,
                      Eigen::Isometry3d destination) {
  // action 1: pick up trash
  {
    auto action = make_shared<PickAction>(robot, "attach_trash_2");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action 2: throw trash
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    link_objectives.push_back(LinkDesiredPose("trash_base_link", destination));

    auto action = make_shared<PlaceAction>(
        robot, "attach_trash_2", link_objectives, joint_objectives, true);

    setBaseJoint(action);
    actions.emplace_back(action);
  }
}

void genOpenDrawerSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action1: pick drawer handle
  {
    auto action = std::make_shared<PickAction>(robot, "attach_drawer");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action2: place drawer handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("drawer_base_drawer1_joint", -0.22);
    auto action = std::make_shared<PlaceAction>(
        robot, "attach_drawer", link_objectives, joint_objectives, false);
    setBaseJoint(action);
    actions.emplace_back(action);
  }
}

void genOpenCabinetSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action1: pick cabinet handle
  {
    auto action = std::make_shared<PickAction>(robot, "attach_cabinet");
    setBaseJoint(action);
    action->setIKCostCoeff(getPickCoeff());
    actions.emplace_back(action);
  }

  // action2: place cabinet handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("cabinet_48479_joint_0", -0.4);
    auto action = std::make_shared<PlaceAction>(
        robot, "attach_cabinet", link_objectives, joint_objectives, false);
    // setBaseJoint(action);
    actions.emplace_back(action);
  }
}

void genOpenDishwasherSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // action1: pick dishwasher handle
  {
    auto action = std::make_shared<PickAction>(robot, "attach_dishwasher");
    auto pick_coeff = getPickCoeff();
    setBaseJoint(action);
    pick_coeff.setZero(9);
    pick_coeff[0] = 1.;
    pick_coeff[1] = 1.;
    action->setIKCostCoeff(pick_coeff);
    actions.emplace_back(action);
  }

  // action2: place dishwasher handle
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;

    joint_objectives.emplace_back("dishwasher_joint_2", -0.8);
    auto action = std::make_shared<PlaceAction>(
        robot, "attach_dishwasher", link_objectives, joint_objectives, false);
    // action->setIKCostCoeff(cost_coeff);
    setBaseJoint(action);
    actions.emplace_back(action);
  }
}

void genUseBroomSeq(vkc::ActionSeq &actions, const std::string &robot) {
  // pick broom
  {
    auto action = std::make_shared<PickAction>(robot, "attach_broom");
    auto coeff = getPickCoeff();
    setBaseJoint(action);
    coeff.setZero(9);
    coeff[0] = 1.;
    coeff[1] = 1.;
    action->setIKCostCoeff(coeff);
    actions.emplace_back(action);
  }
  // pick trash
  {
    auto action = std::make_shared<PickAction>(robot, "attach_trash");
    auto coeff = getPickCoeff();
    actions.emplace_back(action);
  }
  // place trash
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d transform;
    transform.setIdentity();
    transform.translation() = Eigen::Vector3d(2.32, 1.25, 0.05);
    link_objectives.emplace_back("trash_base_link", transform);
    auto action =
        std::make_shared<PlaceAction>(robot, "attach_trash", link_objectives,
                                      joint_objectives, "place trash");
    auto place_coeff = getPlaceCoeff();
    place_coeff.setOnes();
    place_coeff[4] = 10;
    action->setIKCostCoeff(place_coeff);
    actions.emplace_back(action);
  }
  // place broom
  {
    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    Eigen::Isometry3d transform;
    transform.setIdentity();
    transform.translation() = Eigen::Vector3d(3.28, 0.5, 0.0);
    transform.linear() =
        Eigen::Quaterniond(0.5000, 0.5000, -0.5000, -0.5000).matrix();
    link_objectives.emplace_back("broom_base_link", transform);
    auto action = std::make_shared<PlaceAction>(
        robot, "attach_broom", link_objectives, joint_objectives, true);
    actions.emplace_back(action);
  }
}

void genEnvironmentInfo(
    UrdfSceneEnv::AttachObjectInfos &attaches,
    UrdfSceneEnv::InverseChainsInfos &inverse_chains,
    const UrdfSceneEnv::AttachObjectInfo &attach_object_info, int baseline) {
  attaches.emplace_back(attach_object_info);
  if (baseline == 0)
    inverse_chains.emplace_back(UrdfSceneEnv::InverseChainsInfo{
        attach_object_info.base_link, attach_object_info.attach_link});
}

void genTRODemoEnvironmentInfo(UrdfSceneEnv::AttachObjectInfos &attaches,
                               UrdfSceneEnv::InverseChainsInfos &inverse_chains,
                               int task_id, int baseline = 0) {
  Eigen::VectorXd constraint_plane(6);
  constraint_plane << 0, 0, 10, 10, 10, 0;
  switch (task_id) {
    case 0:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                         "fridge_0001_dof_rootd_Aa002_r",
                                         "fridge_0001",
                                         {0.65, -0.25, -0.65},
                                         {0.5, -0.5, 0.5, 0.5},
                                         Eigen::VectorXd()},
          baseline);
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_cup",
                                         "cup_cup_link",
                                         "cup_cup_base_link",
                                         {0.04, 0.05, -0.12},
                                         {0.707106781, 0, 0, 0.707106781}},
          baseline);
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo{"attach_door_handle",
                                                        "door_8966_link_2",
                                                        "door_8966_base",
                                                        {0.0, 0.0, 0.25},
                                                        {0, 0, 1, 0},
                                                        Eigen::VectorXd()},
                         baseline);
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_broom",
                                         "broom_base",
                                         "broom_base_link",
                                         {0, 1.0, 0.11},
                                         {0, 0.7071068, 0.7071068, 0}},
          baseline);
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_trash",
                                         "trash_base",
                                         "trash_base_link",
                                         {-0.1, 0.18, 0.02},
                                         {0, 0, 0.258819, 0.9659258}},
          baseline);
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo{"attach_trash_2",
                                                        "trash_base",
                                                        "trash_base_link",
                                                        {0.0, 0.0, 0.15},
                                                        {0, 0, 1, 0}},
                         1);
      break;
    case 1:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_fridge_handle",
                                         "fridge_0001_dof_rootd_Aa002_r",
                                         "fridge_0001",
                                         {0.65, -0.25, -0.65},
                                         {0.5, -0.5, 0.5, 0.5},
                                         Eigen::VectorXd()},
          baseline);
      break;
    case 2:
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo{"attach_door_handle",
                                                        "door_8966_link_2",
                                                        "door_8966_base",
                                                        {0.0, 0.0, 0.25},
                                                        {0, 0, 1, 0},
                                                        Eigen::VectorXd()},
                         baseline);
      break;
    case 3:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_cup",
                                         "cup_cup_link",
                                         "cup_cup_base_link",
                                         {0.04, 0.05, -0.12},
                                         {0.707106781, 0, 0, 0.707106781}},
          baseline);
      break;
    case 4:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_drawer",
                                         "drawer_handle1",
                                         "drawer_base_link",
                                         {0.16, 0.000, 0.00},
                                         {0.5000, -0.5000, -0.5000, 0.5000},
                                         Eigen::VectorXd()},
          baseline);
      break;
    case 5:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_cabinet",
                                         "cabinet_48479_handle0",
                                         "cabinet_48479_base_link",
                                         {0.0, 0.0, 0.15},
                                         {0, -0.707106781, -0.707106781, 0},
                                         Eigen::VectorXd()},
          baseline);
      break;
    case 6:
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo{
                             "attach_dishwasher",
                             "dishwasher_link_2",
                             "dishwasher",
                             {0.45, 0.55, 0.70},
                             {-0.382683432365090, 0.923879532511287, 0.0, 0.0},
                             Eigen::VectorXd()},
                         baseline);
      break;
    case 7:
      genEnvironmentInfo(
          attaches, inverse_chains,
          UrdfSceneEnv::AttachObjectInfo{"attach_broom",
                                         "broom_base",
                                         "broom_base_link",
                                         {0, 1.0, 0.11},
                                         {0, 0.7071068, 0.7071068, 0}},
          baseline);
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo(
                             "attach_trash", "trash_base", "trash_base_link",
                             {-0.1, 0.18, 0.02}, {0, 0, 0.258819, 0.9659258},
                             constraint_plane),
                         baseline);
      break;
    case 8:
      genEnvironmentInfo(attaches, inverse_chains,
                         UrdfSceneEnv::AttachObjectInfo{"attach_trash_2",
                                                        "trash_base",
                                                        "trash_base_link",
                                                        {0.0, 0.0, 0.15},
                                                        {0, 0, 1, 0}},
                         baseline);
      break;
    default:
      break;
  }

  CONSOLE_BRIDGE_logDebug("environment info generation success");
}

void genTRODemoSeq(VKCEnvBasic &env, vkc::ActionSeq &actions,
                   const std::string &robot, int task_id) {
  Eigen::Isometry3d onKitchenTable;
  onKitchenTable.setIdentity();
  onKitchenTable.translation() = Eigen::Vector3d(-0.6, -1.6, 1.11);
  onKitchenTable.linear() =
      Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();

  Eigen::Isometry3d onDesk;
  onDesk.setIdentity();
  onDesk.translation() = Eigen::Vector3d(-1.9, 0.4, 0.755);
  onDesk.linear() =
      Eigen::Quaterniond(0.5000, -0.5000, -0.5000, 0.5000).matrix();

  Eigen::Isometry3d inTrashCan;
  inTrashCan.setIdentity();
  inTrashCan.translation() = Eigen::Vector3d(3.2, 1.1, 0.5);
  inTrashCan.linear() = Eigen::Quaterniond(1, 0, 0, 0).matrix();

  switch (task_id) {
    case 0:
      genOpenFridgeSeq(actions, robot);
      genMoveCupSeq(actions, robot, onKitchenTable);
      genCloseFridgeSeq(actions, robot);
      genOpenDoorSeq(actions, robot);
      genMoveCupSeq(actions, robot, onDesk);
      genCloseDoorSeq(actions, robot);
      genUseBroomSeq(actions, robot);
      genThrowTrashSeq(actions, robot, inTrashCan);
      break;
    case 1:
      genOpenFridgeSeq(actions, robot);
      break;
    case 2:
      genOpenDoorSeq(actions, robot);
      break;
    case 3:
      genMoveCupSeq(actions, robot, onDesk);
      // Set door to open for individual task
      {
        vector<string> joint_names({"door_8966_joint_1"});
        Eigen::VectorXd joint_values;
        joint_values.setZero(1);
        joint_values[0] = -1.5;
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
    case 4:
      genOpenDrawerSeq(actions, robot);
      // Set door to open for individual task
      {
        vector<string> joint_names({"door_8966_joint_1"});
        Eigen::VectorXd joint_values;
        joint_values.setZero(1);
        joint_values[0] = -1.5;
        env.getVKCEnv()->getTesseract()->setState(joint_names, joint_values);
      }
      break;
    case 5:
      genOpenCabinetSeq(actions, robot);
      break;
    case 6:
      genOpenDishwasherSeq(actions, robot);
      break;
    case 7:
      genUseBroomSeq(actions, robot);
      break;
    case 8:
      genThrowTrashSeq(actions, robot, inTrashCan);
      break;
    default:
      throw std::logic_error("no task id supported");
  }
}

void setTrajSeed(vkc::ActionSeq &actions) {
  int idx = 0;
  for (auto act : actions) {
    // if (idx == 16) return;
    act->loadTrajectorySeed(ros::package::getPath("vkc_example") +
                            "/trajectory/household_env_" + std::to_string(idx) +
                            ".csv");
    idx++;
  }
}

int main(int argc, char **argv) {
  srand((unsigned)time(
      NULL));  // for generating waypoint randomly motion planning
  ros::init(argc, argv, "household_env_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
  setupLog(console_bridge::CONSOLE_BRIDGE_LOG_DEBUG);
  ROS_INFO("Initializaing environment node...");

  bool plotting = true;
  bool rviz = true;
  bool longhorizon = false;
  int baseline = 0;
  int steps = 10;
  int n_iter = 1000;
  int nruns = 5;
  int taskid = 1;
  std::string robot{"vkc"};

  // Get ROS Parameters
  pnh.param<std::string>("robot", robot, robot);
  pnh.param<bool>("plotting", plotting, plotting);
  pnh.param<bool>("rviz", rviz, rviz);
  pnh.param<bool>("longhorizon", longhorizon, longhorizon);
  pnh.param<int>("steps", steps, steps);
  pnh.param<int>("niter", n_iter, n_iter);
  pnh.param<int>("nruns", nruns, nruns);
  pnh.param<int>("taskid", taskid, taskid);
  pnh.param<int>("baseline", baseline, baseline);

  UrdfSceneEnv::AttachObjectInfos attaches;
  UrdfSceneEnv::InverseChainsInfos inverse_chains;
  genTRODemoEnvironmentInfo(attaches, inverse_chains, taskid, baseline);

  UrdfSceneEnv env(nh, plotting, rviz, steps, attaches, inverse_chains);
  env.updateEnv(std::vector<std::string>(), Eigen::VectorXd(), nullptr);
  sampleInitBasePose(env);
  vector<TesseractJointTraj> joint_trajs;
  ActionSeq actions;
  std::vector<double> data;

  genTRODemoSeq(env, actions, robot, taskid);
  // setTrajSeed(actions);
  auto elapsed_time =
      run(joint_trajs, env, actions, steps, n_iter, rviz, nruns, longhorizon);
  interpVKCData(data, elapsed_time, joint_trajs);
  std::string save_path =
      ros::package::getPath("vkc_example") + "/trajectory/household_env_" +
      std::to_string(taskid) + "_" + to_string(longhorizon) + ".csv";
  saveDataToFile(data, save_path);
}
