#ifndef VKC_GOTO_ACTION_H
#define VKC_GOTO_ACTION_H

#include <vkc/action/action_base.h>

#include <vector>


namespace vkc
{

class GotoAction: public ActionBase
{
public:
  using Ptr = std::shared_ptr<GotoAction>;

  GotoAction(
    std::string manipulator_id,
    std::vector<LinkDesiredPose> link_objectives,
    std::vector<JointDesiredPose> joint_objectives
  ):
    ActionBase(ActionType::GotoAction, manipulator_id),
    link_objectives_(link_objectives),
    joint_objectives_(joint_objectives)
  {}

  ~GotoAction() = default;

  std::vector<LinkDesiredPose>& getLinkObjectives()
  {
    return link_objectives_;
  }

  std::vector<JointDesiredPose>& getJointObjectives()
  {
    return joint_objectives_;
  }

private:
  std::vector<LinkDesiredPose> link_objectives_;
  std::vector<JointDesiredPose> joint_objectives_;
};

} // end of namespace vkc

#endif