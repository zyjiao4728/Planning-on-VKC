#ifndef VKC_PLACE_ACTION_H
#define VKC_PLACE_ACTION_H

#include <vkc/action/action_base.h>


namespace vkc
{

class PlaceAction: public ActionBase
{
public:
  using Ptr = std::shared_ptr<PlaceAction>;

  PlaceAction(
    std::string manipulator_id, 
    std::string detached_object_id,
    std::vector<LinkDesiredPose> link_objectives,
    std::vector<JointDesiredPose> joint_objectives
    // LinkDesiredPose base_objective
  ):
    ActionBase(ActionType::PlaceAction, manipulator_id), 
    detached_object_id_(detached_object_id),
    link_objectives_(link_objectives),
    joint_objectives_(joint_objectives)
    // base_objective_(base_objective)
  {}

  PlaceAction(
      std::string manipulator_id,
      std::string detached_object_id,
      std::vector<LinkDesiredPose> link_objectives,
      std::vector<JointDesiredPose> joint_objectives,
      bool init_traj_required
      // LinkDesiredPose base_objective
      ) : ActionBase(ActionType::PlaceAction, manipulator_id),
          detached_object_id_(detached_object_id),
          link_objectives_(link_objectives),
          joint_objectives_(joint_objectives)
  // base_objective_(base_objective)
  {
    init_traj_required_ = init_traj_required;
  }

  ~PlaceAction() = default;

  std::string getDetachedObject()
  {
    return detached_object_id_;
  }

  std::vector<LinkDesiredPose>& getLinkObjectives()
  {
    return link_objectives_;
  }

  void addLinkObjectives(LinkDesiredPose link_pose)
  {
    link_objectives_.push_back(link_pose);
    return;
  }

  // LinkDesiredPose& getBaseObjective()
  // {
  //   return base_objective_;
  // }

  std::vector<JointDesiredPose>& getJointObjectives()
  {
    return joint_objectives_;
  }

private:
  std::string detached_object_id_;
  std::vector<LinkDesiredPose> link_objectives_;
  std::vector<JointDesiredPose> joint_objectives_;
  // LinkDesiredPose base_objective_;
};


// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, vkc::PlaceAction& act)
{
    oss << ">>>" << std::endl
        << "action: PlaceAction" << std::endl
        << "detached_link: " << act.getDetachedObject() << std::endl
        << "manipulator: " << act.getManipulatorID() << std::endl
        << "joint objectives: " << std::endl
        << act.getJointObjectives() << std::endl
        << "link target_pose: " << std::endl 
        << act.getLinkObjectives() << std::endl;
        
    return oss;
}
} // end of namespace vkc

#endif
