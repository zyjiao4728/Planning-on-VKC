#ifndef VKC_BASIC_BALL_H
#define VKC_BASIC_BALL_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseBall : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseBall>;
  using ConstPtr = std::shared_ptr<const BaseBall>;

  BaseBall(std::string object_name, double radius)
      : BaseObject(object_name), radius_(radius) {
    color_ = Eigen::Vector4d(1, 0, 0, 1);
  }

  ~BaseBall() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");

    Link ball_link(object_name_ + "_ball_link");
    Visual::Ptr ball_link_visual = std::make_shared<Visual>();
    ball_link_visual->origin = Eigen::Isometry3d::Identity();
    ball_link_visual->geometry =
        std::make_shared<tesseract_geometry::Sphere>(radius_);
    material_name = ball_link.getName() + "_color";
    ball_link_visual->material = std::make_shared<Material>(material_name);
    ball_link_visual->material->color = color_;
    ball_link.visual.push_back(ball_link_visual);

    Collision::Ptr ball_link_collision = std::make_shared<Collision>();
    ball_link_collision->origin = ball_link_visual->origin;
    ball_link_collision->geometry = ball_link_visual->geometry;
    ball_link.collision.push_back(ball_link_collision);

    Joint ball_joint(object_name_ + "_ball_joint");
    ball_joint.parent_link_name = base_link.getName();
    ball_joint.child_link_name = ball_link.getName();
    ball_joint.type = JointType::FIXED;
    ball_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    ball_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, radius_ / 2.0);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link));
    link_map_[ball_link.getName()] =
        std::make_shared<Link>(std::move(ball_link));

    joint_map_[ball_joint.getName()] =
        std::make_shared<Joint>(std::move(ball_joint));

    // Add an attach location
    AttachLocation attach_location("attach_" + ball_link.getName(),
                                   ball_link.getName());
    attach_location.local_joint_origin_transform.translation() +=
        Eigen::Vector3d((radius_ + 0.1) * sqrt(2) / 2.0, 0.0,
                        (radius_ + 0.1) * sqrt(2) / 2.0);
    attach_location.local_joint_origin_transform.linear() =
        Eigen::Quaterniond(0.0, 0.3827, 0.0, -0.9239).matrix();
    attach_location.fixed_base = false;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    attach_location.connection.child_link_name = ball_link.getName();
    attach_location.connection.parent_link_name = "NULL";

    if (object_scene_graph_ == nullptr) {
      object_scene_graph_ = std::make_shared<SceneGraph>();
    }
    for (auto it : link_map_) {
      object_scene_graph_->addLink(*it.second);
    }
    for (auto it : joint_map_) {
      object_scene_graph_->addJoint(*it.second);
    }
    object_scene_graph_->setName(object_name_);
    object_scene_graph_->setRoot(base_link.getName());
    AttachLocation::Ptr attach_location_ptr =
        std::make_shared<AttachLocation>(std::move(attach_location));
    addAttachLocation(attach_location_ptr);

    return true;
  }

  void setColor(Eigen::Vector4d color) {
    color_ = color;
    return;
  }

 private:
  double radius_;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_BALL_H