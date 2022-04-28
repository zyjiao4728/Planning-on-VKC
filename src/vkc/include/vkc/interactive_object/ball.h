#ifndef VKC_INTERACTIVE_BALL_H
#define VKC_INTERACTIVE_BALL_H
#include <map>
#include <string>
#include <vector>

#include "vkc/action/actions.h"
#include "vkc/interactive_object/interface_object.h"

namespace vkc {
class Ball : public IUsableObject {
 public:
  using Ptr = std::shared_ptr<Ball>;

  Ball(const std::string& name) : name_(name) {
    base_link_name_ = name_ + "_base_link";
    attach_link_name_ = std::string("attach_") + name + "_marker_link";

    // set default init_pose, where the ball is at the beginning
    double link_init_pose_[7]{0.1, 0,   0,  1.0,
                              0.0, 0.0, 0.0};  // 0~2: xyz  3~6: quateraion
    poses_.emplace(
        "loc_under_table1",
        std::vector<double>(link_init_pose_,
                            link_init_pose_ + sizeof(link_init_pose_) /
                                                  sizeof(link_init_pose_[0])));

    // set default fetch_pose, where the ball will be placed after feteched from
    // the under of the table
    double link_fetch_pose_[7]{-1,  1.5, 0.1, 0,
                               0.0, 0.0, 1.0};  // 0~2: xyz  3~6: quateraion
    poses_.emplace("loc_on_ground",
                   std::vector<double>(
                       link_fetch_pose_,
                       link_fetch_pose_ + sizeof(link_fetch_pose_) /
                                              sizeof(link_fetch_pose_[0])));

    // set default temp_place_pose, where the ball will be placed finally
    double link_target_pose_[7]{0,    -2,  1.1, 0.5,
                                -0.5, 0.5, 0.5};  // 0~2: xyz  3~6: quateraion
    poses_.emplace("loc_cabinet_room",
                   std::vector<double>(
                       link_target_pose_,
                       link_target_pose_ + sizeof(link_target_pose_) /
                                               sizeof(link_target_pose_[0])));

    // set the default current_pose, and it always tells where the ball stays
    // currently
    poses_.emplace("current_pose", std::vector<double>(7, 0.0));
  }

  virtual const std::string& Name() const { return name_; }

  void SetPose(const std::string& name, const std::vector<double>& pose) {
    poses_[name] = pose;
  }

  void CurrentPose(const std::vector<double>& pose) {
    poses_["current_pose"] = pose;
  }

  const std::vector<double>& CurrentPose() { return poses_["current_pose"]; }

  double* InitPose() { return poses_["init_pose"].data(); }
  double* Pose() { return poses_["fetch_pose"].data(); }
  double* TargetPose() { return poses_["target_pose"].data(); }

  std::string AttachLinkName() { return attach_link_name_; }

  std::string BaseLinkName() { return base_link_name_; }

  virtual PickAction::Ptr CreatePickAction(const std::string& manipulator) {
    return std::make_shared<PickAction>(manipulator, attach_link_name_, false);
  }

  virtual PlaceAction::Ptr CreatePlaceAction(
      const std::string& manipulator,
      const std::string& pose_name)  // name of the pose where to place the ball
  {
    if (0 == poses_.count(pose_name)) {
      return nullptr;
    }

    std::vector<double>& pose = poses_[pose_name];

    Eigen::Isometry3d tf;
    tf.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    tf.linear() =
        Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).matrix();

    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    link_objectives.emplace_back(base_link_name_, tf);

    return std::make_shared<PlaceAction>(manipulator, attach_link_name_,
                                         link_objectives, joint_objectives,
                                         false);
  }

  virtual UseAction::Ptr CreateUseAction(
      const std::string& manipulator,
      IUsableObject& applied_object)  // which the stick acts on
  {
    return nullptr;
  }

  virtual UseAction::Ptr CreateUseAction(const std::string& manipulator,
                                         const std::string& end_effect_link) {
    Eigen::Isometry3d tf;
    tf.translation() =
        Eigen::Vector3d(CurrentPose()[0], CurrentPose()[1], CurrentPose()[2]);
    tf.linear() = Eigen::Quaterniond(CurrentPose()[3], CurrentPose()[4],
                                     CurrentPose()[5], CurrentPose()[6])
                      .matrix();

    return std::make_shared<UseAction>(manipulator, attach_link_name_, tf,
                                       end_effect_link);
  }

  PlaceAction::Ptr CreateFetchAction(const std::string& manipulator) {
    std::vector<double>& pose = poses_["fetch_pose"];

    Eigen::Isometry3d tf;
    tf.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    tf.linear() =
        Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).matrix();

    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    link_objectives.emplace_back(base_link_name_, tf);

    return std::make_shared<PlaceAction>(manipulator, attach_link_name_,
                                         link_objectives, joint_objectives);
  }

  PlaceAction::Ptr CreatePlaceToCabinetAction(const std::string& manipulator) {
    std::vector<double>& pose = poses_["target_pose"];

    Eigen::Isometry3d tf;
    tf.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    tf.linear() =
        Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).matrix();

    std::vector<LinkDesiredPose> link_objectives;
    std::vector<JointDesiredPose> joint_objectives;
    link_objectives.emplace_back(base_link_name_, tf);

    return std::make_shared<PlaceAction>(manipulator, attach_link_name_,
                                         link_objectives, joint_objectives);
  }

 private:
  std::string name_;
  std::string base_link_name_;
  std::string attach_link_name_;
  std::map<std::string, std::vector<double>> poses_;
};

using Balls = std::vector<Ball>;
}  // namespace vkc
#endif  // VKC_INTERACTIVE_BALL_H
