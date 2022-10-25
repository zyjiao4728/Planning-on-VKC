#ifndef VKC_PLACE_ACTION_H
#define VKC_PLACE_ACTION_H

#include <vkc/action/action_base.h>

namespace vkc {

class PlaceAction : public ActionBase {
 public:
  using Ptr = std::shared_ptr<PlaceAction>;

  PlaceAction(std::string manipulator_id, std::string detached_object_id,
              std::vector<LinkDesiredPose> link_objectives,
              std::vector<JointDesiredPose> joint_objectives,
              std::string name = "PlaceAction"
              // LinkDesiredPose base_objective
              )
      : ActionBase(ActionType::PlaceAction, manipulator_id, name),
        detached_object_id_(detached_object_id),
        is_rigid_object_(true)
  // base_objective_(base_objective)
  {
    for (auto& lo : link_objectives) {
      addLinkObjectives(lo);
    }
    for (auto& jo : joint_objectives) {
      addJointObjectives(jo.joint_name, jo.joint_angle);
    }
  }

  PlaceAction(std::string manipulator_id, std::string detached_object_id,
              std::vector<LinkDesiredPose> link_objectives,
              std::vector<JointDesiredPose> joint_objectives,
              bool init_traj_required, std::string name = "PlaceAction"
              // LinkDesiredPose base_objective
              )
      : ActionBase(ActionType::PlaceAction, manipulator_id, name),
        detached_object_id_(detached_object_id),
        is_rigid_object_(true)
  // base_objective_(base_objective)
  {
    init_traj_required_ = init_traj_required;
    for (auto& lo : link_objectives) {
      addLinkObjectives(lo);
    }
    for (auto& jo : joint_objectives) {
      addJointObjectives(jo.joint_name, jo.joint_angle);
    }
  }

  ~PlaceAction() = default;

  void setNewAttachObject(const std::string& obj_id) {
    attach_object_id_ = obj_id;
  }

  const std::string getNewAttachObject() const { return attach_object_id_; }

  std::string getDetachedObject() { return detached_object_id_; }

  // LinkDesiredPose& getBaseObjective()
  // {
  //   return base_objective_;
  // }

  void setOperationObjectType(const bool is_rigid) {
    is_rigid_object_ = is_rigid;
  }

  bool isRigidObject() const { return is_rigid_object_; }

  // added: wanglei@bigai.ai
  // time: 2021-08-17
  friend std::ostream& operator<<(std::ostream& oss, vkc::PlaceAction& act) {
    oss << ">>>" << std::endl
        << "action: PlaceAction" << std::endl
        << "detached_link: " << act.getDetachedObject() << std::endl
        << "manipulator: " << act.getManipulatorID() << std::endl
        << "object type: " << (act.isRigidObject() ? "rigid" : "articulate")
        << std::endl;
    // << "joint objectives: " << std::endl
    // << act.getJointObjectives() << std::endl
    // << "link target_pose: " << std::endl
    // << act.getLinkObjectives() << std::endl;

    return oss;
  }

 private:
  std::string detached_object_id_;
  std::string attach_object_id_;  // attach to a new object after detached

  bool is_rigid_object_;
  // LinkDesiredPose base_objective_;
};

}  // end of namespace vkc

#endif
