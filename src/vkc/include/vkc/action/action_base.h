#ifndef VKC_ACTION_BASE_H
#define VKC_ACTION_BASE_H

#include <console_bridge/console.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/utils.h>
#include <vkc/utils.h>

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
  {
    ik_cost_coeff_ = Eigen::VectorXd();
  }

  virtual ~ActionBase() = default;

  tesseract_planning::CompositeInstruction seed;

  ActionType getActionType() { return action_type_; }

  std::string getActionName() {
    std::string str;
    switch (action_type_) {
      case ActionType::GotoAction:
        str = "Goto";
        break;
      case ActionType::PickAction:
        str = "Pick";
        break;
      case ActionType::PlaceAction:
        str = "Place";
        break;
      case ActionType::UseAction:
        return "Use";
      default:
        throw std::runtime_error("unknown action");
    }
    return str + ": " + name_;
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

  void setBaseJoint(std::string base_joint_x, std::string base_joint_y) {
    base_joint_ = std::make_pair<std::string, std::string>(
        std::move(base_joint_x), std::move(base_joint_y));
  }
  std::pair<std::string, std::string> getBaseJoint() const {
    return base_joint_;
  }

  void setIKCostCoeff(Eigen::VectorXd ik_cost_coeff) {
    ik_cost_coeff_ = ik_cost_coeff;
  }
  Eigen::VectorXd getIKCostCoeff() { return ik_cost_coeff_; }

  void clearBaseJoint() { base_joint_ = std::make_pair("", ""); }

  void setJointCandidates(
      const std::vector<Eigen::VectorXd>& joint_candidates) {
    joint_candidates_ = joint_candidates;
    joint_candidate_index = 0;
  };

  std::vector<Eigen::VectorXd>& getJointCandidates() {
    return joint_candidates_;
  }

  Eigen::VectorXd getJointCandidate() {
    if (joint_candidate_index < 0) {
      CONSOLE_BRIDGE_logWarn(
          "no joint candidate found, returning empty eigen vector");
      return Eigen::VectorXd();
    }
    return joint_candidates_[joint_candidate_index];
  };

  void switchCandidate() {
    joint_candidate_index += 1;
    if (joint_candidate_index == joint_candidates_.size()) {
      CONSOLE_BRIDGE_logWarn(
          "no joint candidate left, setting joint candidate index to -1");
      joint_candidate_index = -1;
    }
    auto joint_candidate = getJointCandidate();
    std::stringstream ss;
    ss << joint_candidate.transpose();
    CONSOLE_BRIDGE_logDebug("current joint candidate: %s", ss.str().c_str());
    // std::remove(joint_candidates.begin(), joint_candidates.end(),
    //             joint_candidate);
  }

  AStar::Generator astar_generator;  // TODO: change to shared_ptr and get/set
  bool astar_init;

  std::unordered_map<std::string, Eigen::Isometry3d>& getLinkObjectives() {
    return link_objectives_;
  }
  void addLinkObjectives(LinkDesiredPose link_pose) {
    link_objectives_[link_pose.link_name] = link_pose.tf;
    return;
  }

  std::unordered_map<std::string, double>& getJointObjectives() {
    return joint_objectives_;
  }
  void addJointObjectives(std::string joint_name, double joint_val) {
    joint_objectives_[joint_name] = joint_val;
    return;
  }

  void loadTrajectorySeed(std::string file_path) {
    std::ifstream csv_file(file_path);

    std::vector<std::string> joint_names;
    std::vector<Eigen::VectorXd> joint_states;
    if (!csv_file.is_open())
      throw std::runtime_error("Could not open csv file");

    std::string line;
    bool is_header = true;

    while (std::getline(csv_file, line)) {
      std::vector<std::string> tokens;
      boost::split(tokens, line, boost::is_any_of(","),
                   boost::token_compress_on);
      if (is_header) {
        is_header = false;
        for (const auto& t : tokens) {
          joint_names.push_back(t);
        }
        continue;
      }
      if (!tesseract_common::isNumeric(tokens[0]))
        throw std::runtime_error("loadTrajectorySeed: Invalid format");
      std::vector<double> state_vector;
      for (const auto& t : tokens) {
        double value = 0;
        tesseract_common::toNumeric<double>(t, value);
        state_vector.push_back(value);
      }
      Eigen::VectorXd joint_state =
          Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(state_vector.data(),
                                                        state_vector.size());
      joint_states.push_back(joint_state);
    }
    trajectory_seed_.first = joint_names;
    trajectory_seed_.second = joint_states;
  }

  std::pair<std::vector<std::string>, std::vector<Eigen::VectorXd>>&
  getTrajectorySeed() {
    return trajectory_seed_;
  }

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
  Eigen::VectorXd ik_cost_coeff_;
  std::unordered_map<std::string, Eigen::Isometry3d> link_objectives_;
  std::unordered_map<std::string, double> joint_objectives_;
  std::vector<Eigen::VectorXd> joint_candidates_;
  int joint_candidate_index = -1;
  std::pair<std::vector<std::string>, std::vector<Eigen::VectorXd>>
      trajectory_seed_;
};

// added: wanglei@bigai.ai
// time: 2021-08-17
std::ostream& operator<<(std::ostream& oss, ActionBase::Ptr p_act);

}  // end of namespace vkc

#endif
