#ifndef VKC_GOTO_ACTION_H
#define VKC_GOTO_ACTION_H

#include <vkc/action/action_base.h>

#include <vector>

namespace vkc {

class GotoAction : public ActionBase {
 public:
  using Ptr = std::shared_ptr<GotoAction>;

  GotoAction(std::string manipulator_id,
             std::vector<LinkDesiredPose> link_objectives,
             std::vector<JointDesiredPose> joint_objectives)
      : ActionBase(ActionType::GotoAction, manipulator_id, "GotoAction") {
    for (auto &lo : link_objectives) {
      addLinkObjectives(lo);
    }
    for (auto &jo : joint_objectives) {
      addJointObjectives(jo.joint_name, jo.joint_angle);
    }
  }

  ~GotoAction() = default;

  // added: wanglei@bigai.ai
  // time: 2021-08-17
  friend std::ostream &operator<<(std::ostream &oss, GotoAction &act) {
    oss << ">>>" << std::endl
        << "action: GotoAction" << std::endl
        << "manipulator: " << act.getManipulatorID() << std::endl;
    // << "joint objectives: " << std::endl
    // << act.getLinkObjectives() << std::endl
    // << "link target_pose: " << std::endl
    // << act.getJointObjectives() << std::endl;

    return oss;
  }
};

}  // end of namespace vkc

#endif
