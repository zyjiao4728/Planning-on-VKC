#ifndef VKC_BASIC_STICK_H
#define VKC_BASIC_STICK_H

#include <vkc/object/basic_object.h>

namespace vkc
{
class BaseStick : public BaseObject
{
public:
  using Ptr = std::shared_ptr<BaseStick>;
  using ConstPtr = std::shared_ptr<const BaseStick>;

  BaseStick(std::string object_name, double length=1.0) : BaseObject(object_name)
  {
    color_ = Eigen::Vector4d(1.0, 153.0 / 255.0, 51.0 / 255.0, 1);
    length_ = length;
  }

  ~BaseStick() = default;

  bool createObject() override
  {
    std::string material_name;
    
    Link base_link(object_name_ + "_base_link");
    link_map_[base_link.getName()] = std::make_shared<Link>(std::move(base_link));

    Link stick_link(object_name_ + "_stick_link");
    Visual::Ptr stick_link_visual = std::make_shared<Visual>();
    stick_link_visual->origin = Eigen::Isometry3d::Identity();
    stick_link_visual->origin.translation() += Eigen::Vector3d(0, -length_ / 2, 0);
    stick_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(0.1, length_, 0.1);
    material_name = stick_link.getName() + "_color";
    stick_link_visual->material = std::make_shared<Material>(material_name);
    stick_link_visual->material->color = color_;
    stick_link.visual.push_back(stick_link_visual);
    
    Collision::Ptr stick_link_collision = std::make_shared<Collision>();
    stick_link_collision->origin = stick_link_visual->origin;
    stick_link_collision->geometry = stick_link_visual->geometry;
    stick_link.collision.push_back(stick_link_collision);

    Joint stick_link_joint(object_name_ + "_stick_link_joint");
    stick_link_joint.parent_link_name = base_link.getName();
    stick_link_joint.child_link_name = stick_link.getName();
    stick_link_joint.type = JointType::FIXED;
    stick_link_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    stick_link_joint.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, 0.05);

    link_map_[stick_link.getName()] = std::make_shared<Link>(std::move(stick_link));
    joint_map_[stick_link_joint.getName()] = std::make_shared<Joint>(std::move(stick_link_joint));


    // Link stick_tip_link(object_name_ + "_stick_tip_link");
    // Visual::Ptr stick_tip_link_visual = std::make_shared<Visual>();
    // stick_tip_link_visual->origin = Eigen::Isometry3d::Identity();
    // stick_tip_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(0.1, 0.1, 0.1);
    // material_name = stick_tip_link.getName() + "_color";
    // stick_tip_link_visual->material = std::make_shared<Material>(material_name);
    // stick_tip_link_visual->material->color = color_;
    // stick_tip_link.visual.push_back(stick_tip_link_visual);

    // Collision::Ptr stick_tip_link_collision = std::make_shared<Collision>();
    // stick_tip_link_collision->origin = stick_tip_link_visual->origin;
    // stick_tip_link_collision->geometry = stick_tip_link_visual->geometry;
    // stick_tip_link.collision.push_back(stick_tip_link_collision);

    // Joint stick_tip_joint(object_name_ + "_stick_tip_joint");
    // stick_tip_joint.parent_link_name = stick_link.getName();
    // stick_tip_joint.child_link_name = stick_tip_link.getName();
    // stick_tip_joint.type = JointType::FIXED;
    // stick_tip_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    // stick_tip_joint.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0.0, length_, 0.0);

    // link_map_[stick_tip_link.getName()] = std::make_shared<Link>(std::move(stick_tip_link));
    // joint_map_[stick_tip_joint.getName()] = std::make_shared<Joint>(std::move(stick_tip_joint));


    AttachLocation attach_location("attach_" + stick_link.getName(), stick_link.getName());
    attach_location.local_joint_origin_transform.translation() += Eigen::Vector3d(0.0, -length_-0.1, 0.0);
    attach_location.local_joint_origin_transform.linear() = Eigen::Quaterniond(0.6533, -0.2706, -0.2706, 0.6533).matrix();
    attach_location.fixed_base = false;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    attach_location.connection.child_link_name = stick_link.getName();
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
  double length_;
};
}  // namespace vkc

#endif  // VKC_BASIC_WALL_H