#ifndef VKC_BASIC_OBS_H
#define VKC_BASIC_OBS_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseObs : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseObs>;
  using ConstPtr = std::shared_ptr<const BaseObs>;

  BaseObs(std::string object_name, double r, double h)
      : BaseObject(object_name), r_(r), h_(h) {
    color_ = Eigen::Vector4d(1, 0, 0, 0);
  }

  ~BaseObs() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");

    Link obs_link(object_name_ + "_obs_link");
    Visual::Ptr obs_link_visual = std::make_shared<Visual>();
    obs_link_visual->origin = Eigen::Isometry3d::Identity();
    obs_link_visual->geometry =
        std::make_shared<tesseract_geometry::Cylinder>(r_, h_);
    material_name = obs_link.getName() + "_color";
    obs_link_visual->material = std::make_shared<Material>(material_name);
    obs_link_visual->material->color = color_;
    obs_link.visual.push_back(obs_link_visual);

    Collision::Ptr obs_link_collision = std::make_shared<Collision>();
    obs_link_collision->origin = obs_link_visual->origin;
    obs_link_collision->geometry = obs_link_visual->geometry;
    obs_link.collision.push_back(obs_link_collision);

    Joint obs_joint(object_name_ + "_obs_joint");
    obs_joint.parent_link_name = base_link.getName();
    obs_joint.child_link_name = obs_link.getName();
    obs_joint.type = JointType::FIXED;
    obs_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    obs_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, h_ / 2.0);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link));
    link_map_[obs_link.getName()] = std::make_shared<Link>(std::move(obs_link));

    joint_map_[obs_joint.getName()] =
        std::make_shared<Joint>(std::move(obs_joint));

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

    return true;
  }

  void setColor(Eigen::Vector4d color) {
    color_ = color;
    return;
  }

 private:
  double r_;
  double h_;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_OBS_H