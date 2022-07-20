#ifndef VKC_BASIC_MARKER_H
#define VKC_BASIC_MARKER_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseMarker : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseMarker>;
  using ConstPtr = std::shared_ptr<const BaseMarker>;

  BaseMarker(std::string object_name, double radius)
      : BaseObject(object_name), radius_(radius) {
    color_ = Eigen::Vector4d(1, 0, 0, 1);
  }

  ~BaseMarker() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");

    Link marker_link(object_name_ + "_marker_link");
    Visual::Ptr marker_link_visual = std::make_shared<Visual>();
    marker_link_visual->origin = Eigen::Isometry3d::Identity();
    marker_link_visual->geometry =
        std::make_shared<tesseract_geometry::Sphere>(radius_);
    material_name = marker_link.getName() + "_color";
    marker_link_visual->material = std::make_shared<Material>(material_name);
    marker_link_visual->material->color = color_;
    marker_link.visual.push_back(marker_link_visual);

    // Collision::Ptr marker_link_collision = std::make_shared<Collision>();
    // marker_link_collision->origin = marker_link_visual->origin;
    // marker_link_collision->geometry =
    //     std::make_shared<tesseract_geometry::Sphere>(radius_);
    // marker_link.collision.push_back(marker_link_collision);

    Joint marker_joint(object_name_ + "_marker_joint");
    marker_joint.parent_link_name = base_link.getName();
    marker_joint.child_link_name = marker_link.getName();
    marker_joint.type = JointType::FIXED;
    marker_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    marker_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, 0);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link.clone()));
    link_map_[marker_link.getName()] =
        std::make_shared<Link>(std::move(marker_link.clone()));

    joint_map_[marker_joint.getName()] =
        std::make_shared<Joint>(std::move(marker_joint.clone()));

    // Add an attach location
    // AttachLocation attach_location("attach_" + marker_link.getName(),
    // marker_link.getName());
    // attach_location.local_joint_origin_transform.translation() +=
    // Eigen::Vector3d(0, 0, 0);
    // attach_location.local_joint_origin_transform.linear() =
    // Eigen::Quaterniond(1, 0, 0, 0).matrix(); attach_location.fixed_base =
    // true;

    AttachLocation attach_location(fmt::format("attach_{}", marker_link.getName()), marker_link.getName());
    attach_location.local_joint_origin_transform.setIdentity();
    attach_location.local_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, 0.15);
    attach_location.local_joint_origin_transform.linear() =
        Eigen::Quaterniond(0.70710678, 0, -0.70710678, 0).matrix();
    // attach_location.local_joint_origin_transform.linear() =
    // Eigen::Quaterniond(1, 0, 0, 0).matrix();
    attach_location.fixed_base = false;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    attach_location.connection.child_link_name = marker_link.getName();
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

#endif  // VKC_BASIC_MARKER_H