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


// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, GotoAction& act)
{
    oss << ">>>" << std::endl
        << "action: GtoAction" << std::endl
        << "manipulator: " << act.getManipulatorID() << std::endl
        << "joint objectives: " << std::endl
        << act.getLinkObjectives() << std::endl
        << "link target_pose: " << std::endl 
        << act.getJointObjectives() << std::endl;
        
    return oss;
}
} // end of namespace vkc

#endif
