#ifndef VKC_BASIC_CHAIR_H
#define VKC_BASIC_CHAIR_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseChair : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseChair>;
  using ConstPtr = std::shared_ptr<const BaseChair>;

  BaseChair(std::string object_name, double h = 0.45, double w = 0.6,
            double d = 0.6)
      : BaseObject(object_name), h_(h), w_(w), d_(d) {
    color_ = Eigen::Vector4d(35.0 / 255.0, 26.0 / 255.0, 84.0 / 255.0, 1);
  }

  ~BaseChair() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");
    Inertial::Ptr base_link_inertial = std::make_shared<Inertial>();
    base_link_inertial->mass = 5.0;
    base_link_inertial->origin = Eigen::Isometry3d::Identity();
    base_link.inertial = base_link_inertial;

    Visual::Ptr base_link_visual = std::make_shared<Visual>();
    base_link_visual->origin = Eigen::Isometry3d::Identity();
    base_link_visual->origin.translation() +=
        Eigen::Vector3d(0, 0, chair_z + h_ - chair_top_thickness / 2);
    base_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, chair_top_thickness);
    material_name = base_link.getName() + "_color";
    base_link_visual->material = std::make_shared<Material>(material_name);
    base_link_visual->material->color = color_;
    base_link.visual.push_back(base_link_visual);

    Collision::Ptr base_link_collision = std::make_shared<Collision>();
    base_link_collision->origin = base_link_visual->origin;
    base_link_collision->geometry = base_link_visual->geometry;
    base_link.collision.push_back(base_link_collision);

    Link chair_rest_link(object_name_ + "_chair_rest_link");
    Inertial::Ptr chair_rest_link_inertial = std::make_shared<Inertial>();
    chair_rest_link_inertial->mass = 5.0;
    chair_rest_link_inertial->origin = Eigen::Isometry3d::Identity();
    chair_rest_link.inertial = chair_rest_link_inertial;

    Visual::Ptr chair_rest_link_visual = std::make_shared<Visual>();
    chair_rest_link_visual->origin = Eigen::Isometry3d::Identity();
    chair_rest_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    chair_rest_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, chair_top_thickness,
                                                  rest_height);
    material_name = chair_rest_link.getName() + "_color";
    chair_rest_link_visual->material =
        std::make_shared<Material>(material_name);
    chair_rest_link_visual->material->color = color_;
    chair_rest_link.visual.push_back(chair_rest_link_visual);

    Collision::Ptr chair_rest_link_collision = std::make_shared<Collision>();
    chair_rest_link_collision->origin = chair_rest_link_visual->origin;
    chair_rest_link_collision->geometry = chair_rest_link_visual->geometry;
    chair_rest_link.collision.push_back(chair_rest_link_collision);

    Link leg1_link(object_name_ + "_leg1_link");
    Inertial::Ptr leg1_link_inertial = std::make_shared<Inertial>();
    leg1_link_inertial->mass = 3.0;
    leg1_link_inertial->origin = Eigen::Isometry3d::Identity();
    leg1_link.inertial = leg1_link_inertial;

    Visual::Ptr leg1_link_visual = std::make_shared<Visual>();
    leg1_link_visual->origin = Eigen::Isometry3d::Identity();
    leg1_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        0.055, 0.055, h_ - chair_top_thickness);
    material_name = leg1_link.getName() + "_color";
    leg1_link_visual->material = std::make_shared<Material>(material_name);
    leg1_link_visual->material->color = color_;
    leg1_link.visual.push_back(leg1_link_visual);

    Collision::Ptr leg1_link_collision = std::make_shared<Collision>();
    leg1_link_collision->origin = leg1_link_visual->origin;
    leg1_link_collision->geometry = leg1_link_visual->geometry;
    leg1_link.collision.push_back(leg1_link_collision);

    Link leg2_link(object_name_ + "_leg2_link");
    Inertial::Ptr leg2_link_inertial = std::make_shared<Inertial>();
    leg2_link_inertial->mass = 3.0;
    leg2_link_inertial->origin = Eigen::Isometry3d::Identity();
    leg2_link.inertial = leg2_link_inertial;

    Visual::Ptr leg2_link_visual = std::make_shared<Visual>();
    leg2_link_visual->origin = Eigen::Isometry3d::Identity();
    leg2_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        0.055, 0.055, h_ - chair_top_thickness);
    material_name = leg2_link.getName() + "_color";
    leg2_link_visual->material = std::make_shared<Material>(material_name);
    leg2_link_visual->material->color = color_;
    leg2_link.visual.push_back(leg2_link_visual);

    Collision::Ptr leg2_link_collision = std::make_shared<Collision>();
    leg2_link_collision->origin = leg2_link_visual->origin;
    leg2_link_collision->geometry = leg2_link_visual->geometry;
    leg2_link.collision.push_back(leg2_link_collision);

    Link leg3_link(object_name_ + "_leg3_link");
    Inertial::Ptr leg3_link_inertial = std::make_shared<Inertial>();
    leg3_link_inertial->mass = 3.0;
    leg3_link_inertial->origin = Eigen::Isometry3d::Identity();
    leg3_link.inertial = leg3_link_inertial;

    Visual::Ptr leg3_link_visual = std::make_shared<Visual>();
    leg3_link_visual->origin = Eigen::Isometry3d::Identity();
    leg3_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        0.055, 0.055, h_ - chair_top_thickness);
    material_name = leg3_link.getName() + "_color";
    leg3_link_visual->material = std::make_shared<Material>(material_name);
    leg3_link_visual->material->color = color_;
    leg3_link.visual.push_back(leg3_link_visual);

    Collision::Ptr leg3_link_collision = std::make_shared<Collision>();
    leg3_link_collision->origin = leg3_link_visual->origin;
    leg3_link_collision->geometry = leg3_link_visual->geometry;
    leg3_link.collision.push_back(leg3_link_collision);

    Link leg4_link(object_name_ + "_leg4_link");
    Inertial::Ptr leg4_link_inertial = std::make_shared<Inertial>();
    leg4_link_inertial->mass = 3.0;
    leg4_link_inertial->origin = Eigen::Isometry3d::Identity();
    leg4_link.inertial = leg4_link_inertial;

    Visual::Ptr leg4_link_visual = std::make_shared<Visual>();
    leg4_link_visual->origin = Eigen::Isometry3d::Identity();
    leg4_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        0.055, 0.055, h_ - chair_top_thickness);
    material_name = leg4_link.getName() + "_color";
    leg4_link_visual->material = std::make_shared<Material>(material_name);
    leg4_link_visual->material->color = color_;
    leg4_link.visual.push_back(leg4_link_visual);

    Collision::Ptr leg4_link_collision = std::make_shared<Collision>();
    leg4_link_collision->origin = leg4_link_visual->origin;
    leg4_link_collision->geometry = leg4_link_visual->geometry;
    leg4_link.collision.push_back(leg4_link_collision);

    Joint chair_rest_joint(object_name_ + "_chair_rest_joint");
    chair_rest_joint.parent_link_name = base_link.getName();
    chair_rest_joint.child_link_name = chair_rest_link.getName();
    chair_rest_joint.type = JointType::FIXED;
    chair_rest_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    chair_rest_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, d_ / 2.0, chair_z + h_ + rest_height / 2.0);

    Joint leg1_joint(object_name_ + "_leg1_joint");
    leg1_joint.parent_link_name = base_link.getName();
    leg1_joint.child_link_name = leg1_link.getName();
    leg1_joint.type = JointType::FIXED;
    leg1_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    leg1_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0 + 1 * (w_ / 2 - 0.0375),
                        chair_y + 1 * (d_ / 2 - 0.0375), chair_z + h_ / 2);

    Joint leg2_joint(object_name_ + "_leg2_joint");
    leg2_joint.parent_link_name = base_link.getName();
    leg2_joint.child_link_name = leg2_link.getName();
    leg2_joint.type = JointType::FIXED;
    leg2_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    leg2_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0 - 1 * (w_ / 2 - 0.0375),
                        chair_y + 1 * (d_ / 2 - 0.0375), chair_z + h_ / 2);

    Joint leg3_joint(object_name_ + "_leg3_joint");
    leg3_joint.parent_link_name = base_link.getName();
    leg3_joint.child_link_name = leg3_link.getName();
    leg3_joint.type = JointType::FIXED;
    leg3_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    leg3_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0 + 1 * (w_ / 2 - 0.0375),
                        chair_y - 1 * (d_ / 2 - 0.0375), chair_z + h_ / 2);

    Joint leg4_joint(object_name_ + "_leg4_joint");
    leg4_joint.parent_link_name = base_link.getName();
    leg4_joint.child_link_name = leg4_link.getName();
    leg4_joint.type = JointType::FIXED;
    leg4_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    leg4_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0 - 1 * (w_ / 2 - 0.0375),
                        chair_y - 1 * (d_ / 2 - 0.0375), chair_z + h_ / 2);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link));
    link_map_[chair_rest_link.getName()] =
        std::make_shared<Link>(std::move(chair_rest_link));
    link_map_[leg1_link.getName()] =
        std::make_shared<Link>(std::move(leg1_link));
    link_map_[leg2_link.getName()] =
        std::make_shared<Link>(std::move(leg2_link));
    link_map_[leg3_link.getName()] =
        std::make_shared<Link>(std::move(leg3_link));
    link_map_[leg4_link.getName()] =
        std::make_shared<Link>(std::move(leg4_link));

    joint_map_[chair_rest_joint.getName()] =
        std::make_shared<Joint>(std::move(chair_rest_joint));
    joint_map_[leg1_joint.getName()] =
        std::make_shared<Joint>(std::move(leg1_joint));
    joint_map_[leg2_joint.getName()] =
        std::make_shared<Joint>(std::move(leg2_joint));
    joint_map_[leg3_joint.getName()] =
        std::make_shared<Joint>(std::move(leg3_joint));
    joint_map_[leg4_joint.getName()] =
        std::make_shared<Joint>(std::move(leg4_joint));

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
  double chair_y = 0.0;
  double chair_z = 0.0;
  double h_, w_, d_;
  double pi_ = 3.1415926535897931;
  double rest_height = 0.55;
  double chair_top_thickness = 0.02;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_CHAIR_H