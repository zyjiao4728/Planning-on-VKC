#ifndef VKC_ACTION_BASE_H
#define VKC_ACTION_BASE_H

#include <string>
#include <Eigen/Eigen>

namespace vkc
{

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

struct JointDesiredPose{
  std::string joint_name;
  double joint_angle;

  JointDesiredPose(std::string jname, double jval){
    joint_name = jname;
    joint_angle = jval;
  }
};

class ActionBase
{
public:
  using Ptr = std::shared_ptr<ActionBase>;

  ActionBase(ActionType action_type, std::string manipulator_id)
  {
    action_type_ = action_type;
    manipulator_id_ = manipulator_id;
  }

  virtual ~ActionBase() = default;

public:
  ActionType getActionType()
  {
    return action_type_;
  }

  std::string getManipulatorID()
  {
    return manipulator_id_;
  }

protected:
  ActionType action_type_;
  std::string manipulator_id_;
};

} // end of namespace vkc

#endif