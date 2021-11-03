#ifndef VKC_ACTION_BASE_H
#define VKC_ACTION_BASE_H

#include <string>
#include <Eigen/Eigen>

// added: wanglei@bigai.ai 
// time: 2021-08-17 
#include <memory>
#include <iostream>




namespace vkc
{
using VKCTraj = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

enum class ActionType
{
  GotoAction,
  PickAction,
  PlaceAction,
  UseAction
};

struct LinkDesiredPose{
  std::string link_name;
  Eigen::Isometry3d tf;

  LinkDesiredPose(std::string lname, Eigen::Isometry3d &tf_input)
    : link_name(lname), tf(tf_input)
  {}
};

// added: wanglei@bigai.ai 
// time: 2021-08-17
// reason: for output Eigen::Isometry to std::ostream 
std::ostream& operator << (std::ostream& oss, Eigen::Isometry3d& tf);

// added: wanglei@bigai.ai 
// time: 2021-08-17 
// reason: for output LinkDesiredPose to std::ostream easily
std::ostream& operator << (std::ostream& oss, std::vector<vkc::LinkDesiredPose>& poses);


struct JointDesiredPose{
  std::string joint_name;
  double joint_angle;

  JointDesiredPose(std::string jname, double jval){
    joint_name = jname;
    joint_angle = jval;
  }
};

// added: wanglei@bigai.ai 
// time: 2021-08-17 
// reason: for output JointDesiredPose to std::ostream easily
std::ostream& operator << (std::ostream& oss, std::vector<vkc::JointDesiredPose>& poses);


class ActionBase
{
public:
  using Ptr = std::shared_ptr<ActionBase>;

  ActionBase(ActionType action_type,
             const std::string &manipulator_id,
             const std::string &name)
      : action_type_(action_type),
        manipulator_id_(manipulator_id),
        init_traj_required_(false), // initial trajectory for this action is required,  added: wanglei@bigai.ai, time: 2021-08-27
        name_(name) // to print action's name easily,  added: wanglei@bigai.ai, time: 2021-10-22 
  {
  }

  virtual ~ActionBase() = default;

  ActionType getActionType()
  {
    return action_type_;
  }

  std::string getManipulatorID()
  {
    return manipulator_id_;
  }

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  bool RequireInitTraj(bool is_required)
  {
    bool pre_state = init_traj_required_;
    init_traj_required_ = is_required;

    return pre_state;
  }

  bool RequireInitTraj()const
  {
    std::cout << __func__ << "@" << __FILE__ << ": " << init_traj_required_ << std::endl;
    return init_traj_required_;
  }

  std::string Name()const
  {
    return name_;
  }

  bool setInitTrajectory(VKCTraj& init_traj)
  {
    init_traj_ = init_traj;
    init_traj_required_ = true;
  }

  const VKCTraj& getInitTraj()const 
  {
    return init_traj_;
  }

protected:
  ActionType action_type_;
  std::string manipulator_id_;

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  bool init_traj_required_;
  std::string name_;   // action name string value
  VKCTraj init_traj_;
};

// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, ActionBase::Ptr p_act);

} // end of namespace vkc

#endif
