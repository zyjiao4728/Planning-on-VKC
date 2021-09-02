#ifndef VKC_PICK_ACTION_H
#define VKC_PICK_ACTION_H

#include <vkc/action/action_base.h>


namespace vkc
{

class PickAction: public ActionBase
{
public:
  using Ptr = std::shared_ptr<PickAction>;

  PickAction(std::string manipulator_id, std::string attached_object_id)
    :ActionBase(ActionType::PickAction, manipulator_id), attached_object_id_(attached_object_id)
  {}

  PickAction(std::string manipulator_id, std::string attached_object_id, bool init_traj_required)
      : ActionBase(ActionType::PickAction, manipulator_id), attached_object_id_(attached_object_id)
  {
    init_traj_required_ = init_traj_required;
  }

  ~PickAction(){};

  std::string getAttachedObject()
  {
    return attached_object_id_;
  }

private:
  std::string attached_object_id_;
};


// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, PickAction& act)
{
    oss << ">>>" << std::endl
        << "action: PickAction" << std::endl
        << "manipulator: " << act.getManipulatorID() << std::endl
        << "attached_link: " << act.getAttachedObject() << std::endl;
        
    return oss;
}

} // end of namespace vkc

#endif
