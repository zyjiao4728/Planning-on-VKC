#ifndef VKC_BASIC_WALL_H
#define VKC_BASIC_WALL_H

#include <vkc/object/basic_object.h>

namespace vkc
{
class BaseWall : public BaseObject
{
public:
  using Ptr = std::shared_ptr<BaseWall>;
  using ConstPtr = std::shared_ptr<const BaseWall>;

  BaseWall(std::string object_name, double x, double y, double z) : BaseObject(object_name), x_(x), y_(y), z_(z)
  {
    color_ = Eigen::Vector4d(0.99, 0.99, 0.99, 1);
  }

  ~BaseWall() = default;

  bool createObject() override
  {
    std::string material_name;
    
    Link base_link(object_name_ + "_base_link");

    Link wall_link(object_name_ + "_wall_link");
    Visual::Ptr wall_link_visual = std::make_shared<Visual>();
    wall_link_visual->origin = Eigen::Isometry3d::Identity();
    wall_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(x_, y_, z_);
    material_name = wall_link.getName() + "_color";
    wall_link_visual->material = std::make_shared<Material>(material_name);
    wall_link_visual->material->color = color_;
    wall_link.visual.push_back(wall_link_visual);

    Collision::Ptr wall_link_collision = std::make_shared<Collision>();
    wall_link_collision->origin = wall_link_visual->origin;
    wall_link_collision->geometry = wall_link_visual->geometry;
    wall_link.collision.push_back(wall_link_collision);

    Joint wall_joint(object_name_ + "_wall_joint");
    wall_joint.parent_link_name = base_link.getName();
    wall_joint.child_link_name = wall_link.getName();
    wall_joint.type = JointType::FIXED;
    wall_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    wall_joint.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, z_ / 2.0);

    link_map_[base_link.getName()] = std::make_shared<Link>(std::move(base_link));
    link_map_[wall_link.getName()] = std::make_shared<Link>(std::move(wall_link));

    joint_map_[wall_joint.getName()] = std::make_shared<Joint>(std::move(wall_joint));

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

    return true;
  }

  void setColor(Eigen::Vector4d color)
  {
    color_ = color;
    return;
  }

private:
  double x_;
  double y_;
  double z_;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_WALL_H