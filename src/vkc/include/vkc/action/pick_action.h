#ifndef VKC_PICK_ACTION_H
#define VKC_PICK_ACTION_H

#include <vkc/action/action_base.h>

namespace vkc {

class PickAction : public ActionBase {
 public:
  using Ptr = std::shared_ptr<PickAction>;

  PickAction(std::string manipulator_id, std::string attached_object_id)
      : ActionBase(ActionType::PickAction, manipulator_id, "PickAction"),
        attached_object_id_(attached_object_id),
        is_rigid_object_(true) {
    init_traj_required_ = false;
  }

  PickAction(std::string manipulator_id, std::string attached_object_id,
             bool init_traj_required)
      : ActionBase(ActionType::PickAction, manipulator_id, "PickAction"),
        attached_object_id_(attached_object_id),
        is_rigid_object_(true) {
    init_traj_required_ = init_traj_required;
  }

  ~PickAction(){};

  std::string getAttachedObject() { return attached_object_id_; }

  void setOperationObjectType(const bool is_rigid) {
    is_rigid_object_ = is_rigid;
  }

  bool isRigidObject() const { return is_rigid_object_; }

  // added: wanglei@bigai.ai
  // time: 2021-08-17
  friend std::ostream &operator<<(std::ostream &oss, PickAction &act) {
    oss << ">>>" << std::endl
        << "\taction: PickAction" << std::endl
        << "\tmanipulator: " << act.getManipulatorID() << std::endl
        << "\tattached_link: " << act.getAttachedObject();

    return oss;
  }

 private:
  std::string attached_object_id_;
  bool is_rigid_object_;
};

}  // end of namespace vkc

#endif
