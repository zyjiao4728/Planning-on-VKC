#ifndef VKC_BASIC_HOOK_H
#define VKC_BASIC_HOOK_H

#include <vkc/object/basic_object.h>

namespace vkc
{
class BaseHook : public BaseObject
{
public:
  using Ptr = std::shared_ptr<BaseHook>;
  using ConstPtr = std::shared_ptr<const BaseHook>;

  BaseHook(std::string object_name) : BaseObject(object_name)
  {
    color_ = Eigen::Vector4d(1.0, 153.0 / 255.0, 51.0 / 255.0, 1);
  }

  ~BaseHook() = default;

  bool createObject() override
  {
    std::string material_name;
    Link base_link(object_name_ + "_base_link");
    link_map_[base_link.getName()] = std::make_shared<Link>(std::move(base_link));

    Link hook_handle_link(object_name_ + "_hook_handle_link");
    Visual::Ptr hook_handle_link_visual = std::make_shared<Visual>();
    hook_handle_link_visual->origin = Eigen::Isometry3d::Identity();
    hook_handle_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(0.1, 0.9, 0.1);
    material_name = hook_handle_link.getName() + "_color";
    hook_handle_link_visual->material = std::make_shared<Material>(material_name);
    hook_handle_link_visual->material->color = color_;
    hook_handle_link.visual.push_back(hook_handle_link_visual);

    Collision::Ptr hook_handle_link_collision = std::make_shared<Collision>();
    hook_handle_link_collision->origin = hook_handle_link_visual->origin;
    hook_handle_link_collision->geometry = hook_handle_link_visual->geometry;
    hook_handle_link.collision.push_back(hook_handle_link_collision);

    Joint hook_handle_joint(object_name_ + "_hook_handle_joint");
    hook_handle_joint.parent_link_name = base_link.getName();
    hook_handle_joint.child_link_name = hook_handle_link.getName();
    hook_handle_joint.type = JointType::FIXED;
    hook_handle_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    hook_handle_joint.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, 0.05);

    link_map_[hook_handle_link.getName()] = std::make_shared<Link>(std::move(hook_handle_link));
    joint_map_[hook_handle_joint.getName()] = std::make_shared<Joint>(std::move(hook_handle_joint));

    Link hook_tip_link(object_name_ + "_hook_tip_link");
    Visual::Ptr hook_tip_link_visual = std::make_shared<Visual>();
    hook_tip_link_visual->origin = Eigen::Isometry3d::Identity();
    hook_tip_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(0.3, 0.1, 0.1);
    material_name = hook_tip_link.getName() + "_color";
    hook_tip_link_visual->material = std::make_shared<Material>(material_name);
    hook_tip_link_visual->material->color = color_;
    hook_tip_link.visual.push_back(hook_tip_link_visual);

    Collision::Ptr hook_tip_link_collision = std::make_shared<Collision>();
    hook_tip_link_collision->origin = hook_tip_link_visual->origin;
    hook_tip_link_collision->geometry = hook_tip_link_visual->geometry;
    hook_tip_link.collision.push_back(hook_tip_link_collision);

    Joint hook_tip_joint(object_name_ + "_hook_tip_joint");
    hook_tip_joint.parent_link_name = hook_handle_link.getName();
    hook_tip_joint.child_link_name = hook_tip_link.getName();
    hook_tip_joint.type = JointType::FIXED;
    hook_tip_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    hook_tip_joint.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0.15, 0.4, 0.0);

    link_map_[hook_tip_link.getName()] = std::make_shared<Link>(std::move(hook_tip_link));
    joint_map_[hook_tip_joint.getName()] = std::make_shared<Joint>(std::move(hook_tip_joint));

    AttachLocation attach_location("attach_" + hook_handle_link.getName(), hook_handle_link.getName());
    attach_location.local_joint_origin_transform.translation() += Eigen::Vector3d(0.0, -0.45, 0.0);
    attach_location.local_joint_origin_transform.linear() = Eigen::Quaterniond(0.6533, -0.2706, -0.2706, 0.6533).matrix();
    attach_location.fixed_base = true;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    attach_location.connection.child_link_name = hook_handle_link.getName();
    attach_location.connection.parent_link_name = "NULL";

    if (object_scene_graph_ == nullptr)
    {
      object_scene_graph_ = std::make_shared<SceneGraph>();
    }
    for (auto it : link_map_)
    {
      object_scene_graph_->addLink(*it.second);
    }
    for (auto it : joint_map_)
    {
      object_scene_graph_->addJoint(*it.second);
    }
    object_scene_graph_->setName(object_name_);
    object_scene_graph_->setRoot(base_link.getName());

    addAttachLocation(attach_location);

    return true;
  }

  void setColor(Eigen::Vector4d color)
  {
    color_ = color;
    return;
  }

private:
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_WALL_H