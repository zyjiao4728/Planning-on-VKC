#ifndef VKC_BASIC_FRIDGE_H
#define VKC_BASIC_FRIDGE_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseFridge : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseFridge>;
  using ConstPtr = std::shared_ptr<const BaseFridge>;

  BaseFridge(std::string object_name, double h = 1, double w = 0.6,
             double d = 0.8)
      : BaseObject(object_name), h_(h), w_(w), d_(d) {
    color_ = Eigen::Vector4d(100.0 / 255.0, 100.0 / 255.0, 100.0 / 255.0, 1);
  }

  ~BaseFridge() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");

    Link fridge_bottom_link(object_name_ + "_fridge_bottom_link");
    Inertial::Ptr fridge_bottom_link_inertial = std::make_shared<Inertial>();
    fridge_bottom_link_inertial->mass = 5.0;
    fridge_bottom_link_inertial->origin = Eigen::Isometry3d::Identity();
    fridge_bottom_link.inertial = fridge_bottom_link_inertial;

    Visual::Ptr fridge_bottom_link_visual = std::make_shared<Visual>();
    fridge_bottom_link_visual->origin = Eigen::Isometry3d::Identity();
    fridge_bottom_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    fridge_bottom_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, bottom_thickness);
    material_name = fridge_bottom_link.getName() + "_color";
    fridge_bottom_link_visual->material =
        std::make_shared<Material>(material_name);
    fridge_bottom_link_visual->material->color = color_;
    fridge_bottom_link.visual.push_back(fridge_bottom_link_visual);

    Collision::Ptr fridge_bottom_link_collision = std::make_shared<Collision>();
    fridge_bottom_link_collision->origin = fridge_bottom_link_visual->origin;
    fridge_bottom_link_collision->geometry =
        fridge_bottom_link_visual->geometry;
    // fridge_bottom_link.collision.push_back(fridge_bottom_link_collision);

    Link level1_link(object_name_ + "_level1_link");
    Inertial::Ptr level1_link_inertial = std::make_shared<Inertial>();
    level1_link_inertial->mass = 1.0;
    level1_link_inertial->origin = Eigen::Isometry3d::Identity();
    level1_link.inertial = level1_link_inertial;

    Visual::Ptr level1_link_visual = std::make_shared<Visual>();
    level1_link_visual->origin = Eigen::Isometry3d::Identity();
    level1_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    level1_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, wall_thickness);
    material_name = level1_link.getName() + "_color";
    level1_link_visual->material = std::make_shared<Material>(material_name);
    level1_link_visual->material->color = color_;
    level1_link.visual.push_back(level1_link_visual);

    Collision::Ptr level1_link_collision = std::make_shared<Collision>();
    level1_link_collision->origin = level1_link_visual->origin;
    level1_link_collision->geometry = level1_link_visual->geometry;
    // level1_link.collision.push_back(level1_link_collision);

    Link top_link(object_name_ + "_top_link");
    Inertial::Ptr top_link_inertial = std::make_shared<Inertial>();
    top_link_inertial->mass = 1.0;
    top_link_inertial->origin = Eigen::Isometry3d::Identity();
    top_link.inertial = top_link_inertial;

    Visual::Ptr top_link_visual = std::make_shared<Visual>();
    top_link_visual->origin = Eigen::Isometry3d::Identity();
    top_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    top_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, wall_thickness);
    material_name = top_link.getName() + "_color";
    top_link_visual->material = std::make_shared<Material>(material_name);
    top_link_visual->material->color = color_;
    top_link.visual.push_back(top_link_visual);

    Collision::Ptr top_link_collision = std::make_shared<Collision>();
    top_link_collision->origin = top_link_visual->origin;
    top_link_collision->geometry = top_link_visual->geometry;
    // top_link.collision.push_back(top_link_collision);

    Link fridge_door_link(object_name_ + "_fridge_door_link");
    Inertial::Ptr fridge_door_link_inertial = std::make_shared<Inertial>();
    fridge_door_link_inertial->mass = 1.0;
    fridge_door_link_inertial->origin = Eigen::Isometry3d::Identity();
    fridge_door_link.inertial = fridge_door_link_inertial;

    Visual::Ptr fridge_door_link_visual = std::make_shared<Visual>();
    fridge_door_link_visual->origin = Eigen::Isometry3d::Identity();
    fridge_door_link_visual->origin.translation() =
        Eigen::Vector3d(0, d_ / 2.0, 0);
    fridge_door_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(wall_thickness, d_,
                                                  h_ - bottom_thickness);
    material_name = fridge_door_link.getName() + "_color";
    fridge_door_link_visual->material =
        std::make_shared<Material>(material_name);
    fridge_door_link_visual->material->color = color_;
    fridge_door_link.visual.push_back(fridge_door_link_visual);

    Collision::Ptr fridge_door_link_collision = std::make_shared<Collision>();
    fridge_door_link_collision->origin = fridge_door_link_visual->origin;
    fridge_door_link_collision->geometry = fridge_door_link_visual->geometry;
    fridge_door_link.collision.push_back(fridge_door_link_collision);

    Link fridge_back_link(object_name_ + "_fridge_back_link");
    Inertial::Ptr fridge_back_link_inertial = std::make_shared<Inertial>();
    fridge_back_link_inertial->mass = 1.0;
    fridge_back_link_inertial->origin = Eigen::Isometry3d::Identity();
    fridge_back_link.inertial = fridge_back_link_inertial;

    Visual::Ptr fridge_back_link_visual = std::make_shared<Visual>();
    fridge_back_link_visual->origin = Eigen::Isometry3d::Identity();
    fridge_back_link_visual->origin.translation() =
        Eigen::Vector3d(0, -d_ / 2.0, 0);
    fridge_back_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(wall_thickness, d_,
                                                  h_ - bottom_thickness);
    material_name = fridge_back_link.getName() + "_color";
    fridge_back_link_visual->material =
        std::make_shared<Material>(material_name);
    fridge_back_link_visual->material->color = color_;
    fridge_back_link.visual.push_back(fridge_back_link_visual);

    Collision::Ptr fridge_back_link_collision = std::make_shared<Collision>();
    fridge_back_link_collision->origin = fridge_back_link_visual->origin;
    fridge_back_link_collision->geometry = fridge_back_link_visual->geometry;
    // fridge_back_link.collision.push_back(fridge_back_link_collision);

    Link fridge_left_link(object_name_ + "_fridge_left_link");
    Inertial::Ptr fridge_left_link_inertial = std::make_shared<Inertial>();
    fridge_left_link_inertial->mass = 1.0;
    fridge_left_link_inertial->origin = Eigen::Isometry3d::Identity();
    fridge_left_link.inertial = fridge_left_link_inertial;

    Visual::Ptr fridge_left_link_visual = std::make_shared<Visual>();
    fridge_left_link_visual->origin = Eigen::Isometry3d::Identity();
    fridge_left_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, wall_thickness,
                                                  h_ - bottom_thickness);
    material_name = fridge_left_link.getName() + "_color";
    fridge_left_link_visual->material =
        std::make_shared<Material>(material_name);
    fridge_left_link_visual->material->color = color_;
    fridge_left_link.visual.push_back(fridge_left_link_visual);

    Collision::Ptr fridge_left_link_collision = std::make_shared<Collision>();
    fridge_left_link_collision->origin = fridge_left_link_visual->origin;
    fridge_left_link_collision->geometry = fridge_left_link_visual->geometry;
    // fridge_left_link.collision.push_back(fridge_left_link_collision);

    Link fridge_right_link(object_name_ + "_fridge_right_link");
    Inertial::Ptr fridge_right_link_inertial = std::make_shared<Inertial>();
    fridge_right_link_inertial->mass = 1.0;
    fridge_right_link_inertial->origin = Eigen::Isometry3d::Identity();
    fridge_right_link.inertial = fridge_right_link_inertial;

    Visual::Ptr fridge_right_link_visual = std::make_shared<Visual>();
    fridge_right_link_visual->origin = Eigen::Isometry3d::Identity();
    fridge_right_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, wall_thickness,
                                                  h_ - bottom_thickness);
    material_name = fridge_right_link.getName() + "_color";
    fridge_right_link_visual->material =
        std::make_shared<Material>(material_name);
    fridge_right_link_visual->material->color = color_;
    fridge_right_link.visual.push_back(fridge_right_link_visual);

    Collision::Ptr fridge_right_link_collision = std::make_shared<Collision>();
    fridge_right_link_collision->origin = fridge_right_link_visual->origin;
    fridge_right_link_collision->geometry = fridge_right_link_visual->geometry;
    // fridge_right_link.collision.push_back(fridge_right_link_collision);

    Link knob1_link(object_name_ + "_knob1_link");
    Inertial::Ptr knob1_link_inertial = std::make_shared<Inertial>();
    knob1_link_inertial->mass = 1.0;
    knob1_link_inertial->origin = Eigen::Isometry3d::Identity();
    knob1_link.inertial = knob1_link_inertial;

    Visual::Ptr knob1_link_visual = std::make_shared<Visual>();
    knob1_link_visual->origin = Eigen::Isometry3d::Identity();
    knob1_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(0.03, 0.015, 0.01);
    material_name = knob1_link.getName() + "_color";
    knob1_link_visual->material = std::make_shared<Material>(material_name);
    knob1_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1);
    knob1_link.visual.push_back(knob1_link_visual);

    Collision::Ptr knob1_link_collision = std::make_shared<Collision>();
    knob1_link_collision->origin = knob1_link_visual->origin;
    knob1_link_collision->geometry = knob1_link_visual->geometry;
    // knob1_link.collision.push_back(knob1_link_collision);

    Link knob2_link(object_name_ + "_knob2_link");
    Inertial::Ptr knob2_link_inertial = std::make_shared<Inertial>();
    knob2_link_inertial->mass = 1.0;
    knob2_link_inertial->origin = Eigen::Isometry3d::Identity();
    knob2_link.inertial = knob2_link_inertial;

    Visual::Ptr knob2_link_visual = std::make_shared<Visual>();
    knob2_link_visual->origin = Eigen::Isometry3d::Identity();
    knob2_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(0.03, 0.015, 0.01);
    material_name = knob2_link.getName() + "_color";
    knob2_link_visual->material = std::make_shared<Material>(material_name);
    knob2_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1);
    knob2_link.visual.push_back(knob2_link_visual);

    Collision::Ptr knob2_link_collision = std::make_shared<Collision>();
    knob2_link_collision->origin = knob2_link_visual->origin;
    knob2_link_collision->geometry = knob2_link_visual->geometry;
    // knob2_link.collision.push_back(knob2_link_collision);

    Link handle_link(object_name_ + "_handle_link");
    Inertial::Ptr handle_link_inertial = std::make_shared<Inertial>();
    handle_link_inertial->mass = 1.0;
    handle_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle_link.inertial = handle_link_inertial;

    Visual::Ptr handle_link_visual = std::make_shared<Visual>();
    handle_link_visual->origin = Eigen::Isometry3d::Identity();
    handle_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(0.01, 0.015, 0.14);
    material_name = handle_link.getName() + "_color";
    handle_link_visual->material = std::make_shared<Material>(material_name);
    handle_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1);
    handle_link.visual.push_back(handle_link_visual);

    Collision::Ptr handle_link_collision = std::make_shared<Collision>();
    handle_link_collision->origin = handle_link_visual->origin;
    handle_link_collision->geometry = handle_link_visual->geometry;
    // handle_link.collision.push_back(handle_link_collision);

    Joint fridge_bottom_joint(object_name_ + "_fridge_bottom_joint");
    fridge_bottom_joint.parent_link_name = base_link.getName();
    fridge_bottom_joint.child_link_name = fridge_bottom_link.getName();
    fridge_bottom_joint.type = JointType::FIXED;
    fridge_bottom_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_bottom_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, bottom_thickness / 2);

    Joint fridge_level1_joint(object_name_ + "_fridge_level1_joint");
    fridge_level1_joint.parent_link_name = base_link.getName();
    fridge_level1_joint.child_link_name = level1_link.getName();
    fridge_level1_joint.type = JointType::FIXED;
    fridge_level1_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_level1_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 2.0 * 1 + wall_thickness / 2);

    Joint fridge_top_joint(object_name_ + "_fridge_top_joint");
    fridge_top_joint.parent_link_name = base_link.getName();
    fridge_top_joint.child_link_name = top_link.getName();
    fridge_top_joint.type = JointType::FIXED;
    fridge_top_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_top_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 2.0 * 2 + wall_thickness / 2);

    Joint fridge_door_joint(object_name_ + "_fridge_door_joint");
    fridge_door_joint.parent_link_name = fridge_bottom_link.getName();
    fridge_door_joint.child_link_name = fridge_door_link.getName();
    fridge_door_joint.type = JointType::REVOLUTE;
    fridge_door_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_door_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-w_ / 2.0, -d_ / 2.0, h_ / 2.0);
    fridge_door_joint.axis = Eigen::Vector3d(0, 0, 1);
    fridge_door_joint.limits = std::make_shared<JointLimits>();
    fridge_door_joint.limits->lower = 0;
    fridge_door_joint.limits->upper = pi_;
    fridge_door_joint.limits->effort = 20;
    fridge_door_joint.limits->velocity = 5;

    Joint fridge_back_joint(object_name_ + "_fridge_back_joint");
    fridge_back_joint.parent_link_name = fridge_bottom_link.getName();
    fridge_back_joint.child_link_name = fridge_back_link.getName();
    fridge_back_joint.type = JointType::FIXED;
    fridge_back_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_back_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(w_ / 2.0 - wall_thickness / 2, d_ / 2.0, h_ / 2.0);

    Joint fridge_left_joint(object_name_ + "_fridge_left_joint");
    fridge_left_joint.parent_link_name = fridge_bottom_link.getName();
    fridge_left_joint.child_link_name = fridge_left_link.getName();
    fridge_left_joint.type = JointType::FIXED;
    fridge_left_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_left_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0, d_ / 2.0 - wall_thickness / 2, h_ / 2.0);

    Joint fridge_right_joint(object_name_ + "_fridge_right_joint");
    fridge_right_joint.parent_link_name = fridge_bottom_link.getName();
    fridge_right_joint.child_link_name = fridge_right_link.getName();
    fridge_right_joint.type = JointType::FIXED;
    fridge_right_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    fridge_right_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0, -d_ / 2.0 + wall_thickness / 2, h_ / 2.0);

    Joint knob1_joint(object_name_ + "_knob1_joint");
    knob1_joint.parent_link_name = fridge_door_link.getName();
    knob1_joint.child_link_name = knob1_link.getName();
    knob1_joint.type = JointType::FIXED;
    knob1_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob1_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.015, d_ - 0.1, h_ / 2 - 0.1);

    Joint knob2_joint(object_name_ + "_knob2_joint");
    knob2_joint.parent_link_name = fridge_door_link.getName();
    knob2_joint.child_link_name = knob2_link.getName();
    knob2_joint.type = JointType::FIXED;
    knob2_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob2_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.015, d_ - 0.1, h_ / 2 - 0.23);

    Joint knob3_joint(object_name_ + "_knob3_joint");
    knob3_joint.parent_link_name = knob1_link.getName();
    knob3_joint.child_link_name = handle_link.getName();
    knob3_joint.type = JointType::FIXED;
    knob3_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob3_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.02, 0, -0.065);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link));
    link_map_[fridge_bottom_link.getName()] =
        std::make_shared<Link>(std::move(fridge_bottom_link));
    link_map_[level1_link.getName()] =
        std::make_shared<Link>(std::move(level1_link));
    link_map_[top_link.getName()] = std::make_shared<Link>(std::move(top_link));
    link_map_[fridge_back_link.getName()] =
        std::make_shared<Link>(std::move(fridge_back_link));
    link_map_[fridge_left_link.getName()] =
        std::make_shared<Link>(std::move(fridge_left_link));
    link_map_[fridge_right_link.getName()] =
        std::make_shared<Link>(std::move(fridge_right_link));
    link_map_[fridge_door_link.getName()] =
        std::make_shared<Link>(std::move(fridge_door_link));
    link_map_[knob1_link.getName()] =
        std::make_shared<Link>(std::move(knob1_link));
    link_map_[knob2_link.getName()] =
        std::make_shared<Link>(std::move(knob2_link));
    link_map_[handle_link.getName()] =
        std::make_shared<Link>(std::move(handle_link));

    joint_map_[fridge_bottom_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_bottom_joint));
    joint_map_[fridge_door_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_door_joint));
    joint_map_[fridge_level1_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_level1_joint));
    joint_map_[fridge_top_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_top_joint));
    joint_map_[fridge_back_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_back_joint));
    joint_map_[fridge_left_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_left_joint));
    joint_map_[fridge_right_joint.getName()] =
        std::make_shared<Joint>(std::move(fridge_right_joint));
    joint_map_[knob1_joint.getName()] =
        std::make_shared<Joint>(std::move(knob1_joint));
    joint_map_[knob2_joint.getName()] =
        std::make_shared<Joint>(std::move(knob2_joint));
    joint_map_[knob3_joint.getName()] =
        std::make_shared<Joint>(std::move(knob3_joint));

    AttachLocation attach_location("attach_" + handle_link.getName(),
                                   handle_link.getName());
    attach_location.local_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.1, 0, 0.0);
    attach_location.local_joint_origin_transform.linear() =
        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();
    attach_location.fixed_base = true;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    attach_location.connection.child_link_name = handle_link.getName();
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
  double fridge_x = 1.5;
  double fridge_y = 0.0;
  double fridge_z = 0.0;
  double pi_ = 3.1415926535897931;
  double bottom_thickness = 0.08;
  double wall_thickness = 0.02;
  double h_, w_, d_;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_FRIDGE_H