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

  ~PickAction(){};

  std::string getAttachedObject()
  {
    return attached_object_id_;
  }

private:
  std::string attached_object_id_;
};

} // end of namespace vkc

#endif