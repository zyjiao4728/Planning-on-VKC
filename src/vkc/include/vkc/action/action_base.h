#ifndef VKC_ACTION_BASE_H
#define VKC_ACTION_BASE_H

#include <tesseract_command_language/command_language.h>

#include <Eigen/Eigen>
#include <string>

// added: wanglei@bigai.ai
// time: 2021-08-17
#include <AStar.hpp>
#include <iostream>
#include <memory>

namespace vkc {
using VKCTraj =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

enum class ActionType { GotoAction, PickAction, PlaceAction, UseAction };

struct LinkDesiredPose {
  std::string link_name;
  Eigen::Isometry3d tf;

  LinkDesiredPose(std::string lname, Eigen::Isometry3d& tf_input)
      : link_name(lname), tf(tf_input) {}
};

// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for output Eigen::Isometry to std::ostream
std::ostream& operator<<(std::ostream& oss, Eigen::Isometry3d& tf);

// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for output LinkDesiredPose to std::ostream easily
std::ostream& operator<<(std::ostream& oss,
                         std::vector<vkc::LinkDesiredPose>& poses);

struct JointDesiredPose {
  std::string joint_name;
  double joint_angle;

  JointDesiredPose(std::string jname, double jval) {
    joint_name = jname;
    joint_angle = jval;
  }
};

// added: wanglei@bigai.ai
// time: 2021-08-17
// reason: for output JointDesiredPose to std::ostream easily
std::ostream& operator<<(std::ostream& oss,
                         std::vector<vkc::JointDesiredPose>& poses);

class ActionBase {
 public:
  using Ptr = std::shared_ptr<ActionBase>;

  ActionBase(ActionType action_type, const std::string& manipulator_id,
             const std::string& name)
      : action_type_(action_type),
        manipulator_id_(manipulator_id),
        name_(name),  // to print action's name easily,  added:
                      // wanglei@bigai.ai, time: 2021-10-22
        init_traj_required_(
            false)  // initial trajectory for this action is required,  added:
                    // wanglei@bigai.ai, time: 2021-08-27
  {}

  virtual ~ActionBase() = default;

  tesseract_planning::CompositeInstruction seed;
  Eigen::VectorXd joint_candidate;
  std::vector<Eigen::VectorXd> joint_candidates;

  ActionType getActionType() { return action_type_; }

  std::string getActionName() {
    switch (action_type_) {
      case ActionType::GotoAction:
        return "Goto";
      case ActionType::PickAction:
        return "Pick";
      case ActionType::PlaceAction:
        return "Place";
      case ActionType::UseAction:
        return "Use";
      default:
        return "Unknown Action";
    }
    throw std::runtime_error("unknown action");
  }

  std::string getManipulatorID() { return manipulator_id_; }

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  bool RequireInitTraj(bool is_required) {
    bool pre_state = init_traj_required_;
    init_traj_required_ = is_required;

    return pre_state;
  }

  bool RequireInitTraj() const { return init_traj_required_; }

  std::string Name() const { return name_; }

  bool setInitTrajectory(VKCTraj& init_traj) {
    init_traj_ = init_traj;
    init_traj_required_ = true;

    return true;
  }

  const VKCTraj& getInitTraj() const { return init_traj_; }

  std::pair<std::string, std::string> getBaseJoint() const {
    return base_joint_;
  }

  void setBaseJoint(std::string base_joint_x, std::string base_joint_y) {
    base_joint_ = std::make_pair<std::string, std::string>(
        std::move(base_joint_x), std::move(base_joint_y));
  }

  void clearBaseJoint() { base_joint_ = std::make_pair("", ""); }

  void switchCandidate() {
    joint_candidate = joint_candidates.front();
    joint_candidates.erase(joint_candidates.begin());
    // std::remove(joint_candidates.begin(), joint_candidates.end(),
    //             joint_candidate);
  }

  AStar::Generator astar_generator; // TODO: change to shared_ptr and get/set
  bool astar_init;

 protected:
  ActionType action_type_;
  std::string manipulator_id_;

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  std::string name_;  // action name string value
  VKCTraj init_traj_;
  bool init_traj_required_;
  std::pair<std::string, std::string> base_joint_;
};

// added: wanglei@bigai.ai
// time: 2021-08-17
std::ostream& operator<<(std::ostream& oss, ActionBase::Ptr p_act);

}  // end of namespace vkc

#endif
