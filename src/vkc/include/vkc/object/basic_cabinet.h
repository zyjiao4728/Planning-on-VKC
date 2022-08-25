#ifndef VKC_BASIC_CABINET_H
#define VKC_BASIC_CABINET_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseCabinet : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseCabinet>;
  using ConstPtr = std::shared_ptr<const BaseCabinet>;

  BaseCabinet(std::string object_name, double h = 1.90, double w = 0.8,
              double d = 1.6)
      : BaseObject(object_name), h_(h), w_(w), d_(d) {
    color_ = Eigen::Vector4d(1.0, 153.0 / 255.0, 51.0 / 255.0, 1);
  }

  ~BaseCabinet() = default;

  bool createObject() override {
    std::string material_name;

    Link base_link(object_name_ + "_base_link");

    Link cabinet_bottom_link(object_name_ + "_cabinet_bottom_link");
    Inertial::Ptr cabinet_bottom_link_inertial = std::make_shared<Inertial>();
    cabinet_bottom_link_inertial->mass = 5.0;
    cabinet_bottom_link_inertial->origin = Eigen::Isometry3d::Identity();
    cabinet_bottom_link.inertial = cabinet_bottom_link_inertial;

    Visual::Ptr cabinet_bottom_link_visual = std::make_shared<Visual>();
    cabinet_bottom_link_visual->origin = Eigen::Isometry3d::Identity();
    cabinet_bottom_link_visual->origin.translation() +=
        Eigen::Vector3d(0, 0, 0);
    cabinet_bottom_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, bottom_thickness);
    material_name = cabinet_bottom_link.getName() + "_color";
    cabinet_bottom_link_visual->material =
        std::make_shared<Material>(material_name);
    cabinet_bottom_link_visual->material->color = color_;
    cabinet_bottom_link.visual.push_back(cabinet_bottom_link_visual);

    Collision::Ptr cabinet_bottom_link_collision =
        std::make_shared<Collision>();
    cabinet_bottom_link_collision->origin = cabinet_bottom_link_visual->origin;
    cabinet_bottom_link_collision->geometry =
        cabinet_bottom_link_visual->geometry;
    cabinet_bottom_link.collision.push_back(cabinet_bottom_link_collision);

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
    level1_link.collision.push_back(level1_link_collision);

    Link level2_link(object_name_ + "_level2_link");
    Inertial::Ptr level2_link_inertial = std::make_shared<Inertial>();
    level2_link_inertial->mass = 1.0;
    level2_link_inertial->origin = Eigen::Isometry3d::Identity();
    level2_link.inertial = level2_link_inertial;

    Visual::Ptr level2_link_visual = std::make_shared<Visual>();
    level2_link_visual->origin = Eigen::Isometry3d::Identity();
    level2_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    level2_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, wall_thickness);
    material_name = level2_link.getName() + "_color";
    level2_link_visual->material = std::make_shared<Material>(material_name);
    level2_link_visual->material->color = color_;
    level2_link.visual.push_back(level2_link_visual);

    Collision::Ptr level2_link_collision = std::make_shared<Collision>();
    level2_link_collision->origin = level2_link_visual->origin;
    level2_link_collision->geometry = level2_link_visual->geometry;
    level2_link.collision.push_back(level2_link_collision);

    Link level3_link(object_name_ + "_level3_link");
    Inertial::Ptr level3_link_inertial = std::make_shared<Inertial>();
    level3_link_inertial->mass = 1.0;
    level3_link_inertial->origin = Eigen::Isometry3d::Identity();
    level3_link.inertial = level3_link_inertial;

    Visual::Ptr level3_link_visual = std::make_shared<Visual>();
    level3_link_visual->origin = Eigen::Isometry3d::Identity();
    level3_link_visual->origin.translation() += Eigen::Vector3d(0, 0, 0);
    level3_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, d_, wall_thickness);
    material_name = level3_link.getName() + "_color";
    level3_link_visual->material = std::make_shared<Material>(material_name);
    level3_link_visual->material->color = color_;
    level3_link.visual.push_back(level3_link_visual);

    Collision::Ptr level3_link_collision = std::make_shared<Collision>();
    level3_link_collision->origin = level3_link_visual->origin;
    level3_link_collision->geometry = level3_link_visual->geometry;
    level3_link.collision.push_back(level3_link_collision);

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
    top_link.collision.push_back(top_link_collision);

    Link cabinet_door_link(object_name_ + "_cabinet_door_link");
    Inertial::Ptr cabinet_door_link_inertial = std::make_shared<Inertial>();
    cabinet_door_link_inertial->mass = 1.0;
    cabinet_door_link_inertial->origin = Eigen::Isometry3d::Identity();
    cabinet_door_link.inertial = cabinet_door_link_inertial;

    Visual::Ptr cabinet_door_link_visual = std::make_shared<Visual>();
    cabinet_door_link_visual->origin = Eigen::Isometry3d::Identity();
    cabinet_door_link_visual->origin.translation() =
        Eigen::Vector3d(0, -d_ / 2.0, 0);
    cabinet_door_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(wall_thickness, d_,
                                                  h_ - bottom_thickness);
    material_name = cabinet_door_link.getName() + "_color";
    cabinet_door_link_visual->material =
        std::make_shared<Material>(material_name);
    cabinet_door_link_visual->material->color = color_;
    cabinet_door_link.visual.push_back(cabinet_door_link_visual);

    Collision::Ptr cabinet_door_link_collision = std::make_shared<Collision>();
    cabinet_door_link_collision->origin = cabinet_door_link_visual->origin;
    cabinet_door_link_collision->geometry = cabinet_door_link_visual->geometry;
    cabinet_door_link.collision.push_back(cabinet_door_link_collision);

    Link cabinet_back_link(object_name_ + "_cabinet_back_link");
    Inertial::Ptr cabinet_back_link_inertial = std::make_shared<Inertial>();
    cabinet_back_link_inertial->mass = 1.0;
    cabinet_back_link_inertial->origin = Eigen::Isometry3d::Identity();
    cabinet_back_link.inertial = cabinet_back_link_inertial;

    Visual::Ptr cabinet_back_link_visual = std::make_shared<Visual>();
    cabinet_back_link_visual->origin = Eigen::Isometry3d::Identity();
    cabinet_back_link_visual->origin.translation() =
        Eigen::Vector3d(0, -d_ / 2.0, 0);
    cabinet_back_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(wall_thickness, d_,
                                                  h_ - bottom_thickness);
    material_name = cabinet_back_link.getName() + "_color";
    cabinet_back_link_visual->material =
        std::make_shared<Material>(material_name);
    cabinet_back_link_visual->material->color = color_;
    cabinet_back_link.visual.push_back(cabinet_back_link_visual);

    Collision::Ptr cabinet_back_link_collision = std::make_shared<Collision>();
    cabinet_back_link_collision->origin = cabinet_back_link_visual->origin;
    cabinet_back_link_collision->geometry = cabinet_back_link_visual->geometry;
    cabinet_back_link.collision.push_back(cabinet_back_link_collision);

    Link cabinet_left_link(object_name_ + "_cabinet_left_link");
    Inertial::Ptr cabinet_left_link_inertial = std::make_shared<Inertial>();
    cabinet_left_link_inertial->mass = 1.0;
    cabinet_left_link_inertial->origin = Eigen::Isometry3d::Identity();
    cabinet_left_link.inertial = cabinet_left_link_inertial;

    Visual::Ptr cabinet_left_link_visual = std::make_shared<Visual>();
    cabinet_left_link_visual->origin = Eigen::Isometry3d::Identity();
    cabinet_left_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, wall_thickness,
                                                  h_ - bottom_thickness);
    material_name = cabinet_left_link.getName() + "_color";
    cabinet_left_link_visual->material =
        std::make_shared<Material>(material_name);
    cabinet_left_link_visual->material->color = color_;
    cabinet_left_link.visual.push_back(cabinet_left_link_visual);

    Collision::Ptr cabinet_left_link_collision = std::make_shared<Collision>();
    cabinet_left_link_collision->origin = cabinet_left_link_visual->origin;
    cabinet_left_link_collision->geometry = cabinet_left_link_visual->geometry;
    cabinet_left_link.collision.push_back(cabinet_left_link_collision);

    Link cabinet_right_link(object_name_ + "_cabinet_right_link");
    Inertial::Ptr cabinet_right_link_inertial = std::make_shared<Inertial>();
    cabinet_right_link_inertial->mass = 1.0;
    cabinet_right_link_inertial->origin = Eigen::Isometry3d::Identity();
    cabinet_right_link.inertial = cabinet_right_link_inertial;

    Visual::Ptr cabinet_right_link_visual = std::make_shared<Visual>();
    cabinet_right_link_visual->origin = Eigen::Isometry3d::Identity();
    cabinet_right_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(w_, wall_thickness,
                                                  h_ - bottom_thickness);
    material_name = cabinet_right_link.getName() + "_color";
    cabinet_right_link_visual->material =
        std::make_shared<Material>(material_name);
    cabinet_right_link_visual->material->color = color_;
    cabinet_right_link.visual.push_back(cabinet_right_link_visual);

    Collision::Ptr cabinet_right_link_collision = std::make_shared<Collision>();
    cabinet_right_link_collision->origin = cabinet_right_link_visual->origin;
    cabinet_right_link_collision->geometry =
        cabinet_right_link_visual->geometry;
    cabinet_right_link.collision.push_back(cabinet_right_link_collision);

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
    knob1_link.collision.push_back(knob1_link_collision);

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
    knob2_link.collision.push_back(knob2_link_collision);

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
    handle_link.collision.push_back(handle_link_collision);

    Joint cabinet_bottom_joint(object_name_ + "_cabinet_bottom_joint");
    cabinet_bottom_joint.parent_link_name = base_link.getName();
    cabinet_bottom_joint.child_link_name = cabinet_bottom_link.getName();
    cabinet_bottom_joint.type = JointType::FIXED;
    cabinet_bottom_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_bottom_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, bottom_thickness / 2);

    Joint cabinet_level1_joint(object_name_ + "_cabinet_level1_joint");
    cabinet_level1_joint.parent_link_name = base_link.getName();
    cabinet_level1_joint.child_link_name = level1_link.getName();
    cabinet_level1_joint.type = JointType::FIXED;
    cabinet_level1_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_level1_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 4.0 * 1 + wall_thickness / 2);

    Joint cabinet_level2_joint(object_name_ + "_cabinet_level2_joint");
    cabinet_level2_joint.parent_link_name = base_link.getName();
    cabinet_level2_joint.child_link_name = level2_link.getName();
    cabinet_level2_joint.type = JointType::FIXED;
    cabinet_level2_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_level2_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 4.0 * 2 + wall_thickness / 2);

    Joint cabinet_level3_joint(object_name_ + "_cabinet_level3_joint");
    cabinet_level3_joint.parent_link_name = base_link.getName();
    cabinet_level3_joint.child_link_name = level3_link.getName();
    cabinet_level3_joint.type = JointType::FIXED;
    cabinet_level3_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_level3_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 4.0 * 3 + wall_thickness / 2);

    Joint cabinet_top_joint(object_name_ + "_cabinet_top_joint");
    cabinet_top_joint.parent_link_name = base_link.getName();
    cabinet_top_joint.child_link_name = top_link.getName();
    cabinet_top_joint.type = JointType::FIXED;
    cabinet_top_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_top_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0, (h_) / 4.0 * 4 + wall_thickness / 2);

    Joint cabinet_door_joint(object_name_ + "_cabinet_door_joint");
    cabinet_door_joint.parent_link_name = cabinet_bottom_link.getName();
    cabinet_door_joint.child_link_name = cabinet_door_link.getName();
    cabinet_door_joint.type = JointType::REVOLUTE;
    cabinet_door_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_door_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-w_ / 2.0, d_ / 2.0, h_ / 2.0);
    cabinet_door_joint.axis = Eigen::Vector3d(0, 0, 1);
    cabinet_door_joint.limits = std::make_shared<JointLimits>();
    cabinet_door_joint.limits->lower = -pi_ / 2.0;
    cabinet_door_joint.limits->upper = 0;
    cabinet_door_joint.limits->effort = 20;
    cabinet_door_joint.limits->velocity = 5;

    Joint cabinet_back_joint(object_name_ + "_cabinet_back_joint");
    cabinet_back_joint.parent_link_name = cabinet_bottom_link.getName();
    cabinet_back_joint.child_link_name = cabinet_back_link.getName();
    cabinet_back_joint.type = JointType::FIXED;
    cabinet_back_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_back_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(w_ / 2.0 - wall_thickness / 2, d_ / 2.0, h_ / 2.0);

    Joint cabinet_left_joint(object_name_ + "_cabinet_left_joint");
    cabinet_left_joint.parent_link_name = cabinet_bottom_link.getName();
    cabinet_left_joint.child_link_name = cabinet_left_link.getName();
    cabinet_left_joint.type = JointType::FIXED;
    cabinet_left_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_left_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0, d_ / 2.0 - wall_thickness / 2, h_ / 2.0);

    Joint cabinet_right_joint(object_name_ + "_cabinet_right_joint");
    cabinet_right_joint.parent_link_name = cabinet_bottom_link.getName();
    cabinet_right_joint.child_link_name = cabinet_right_link.getName();
    cabinet_right_joint.type = JointType::FIXED;
    cabinet_right_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    cabinet_right_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0, -d_ / 2.0 + wall_thickness / 2, h_ / 2.0);

    Joint knob1_joint(object_name_ + "_knob1_joint");
    knob1_joint.parent_link_name = cabinet_door_link.getName();
    knob1_joint.child_link_name = knob1_link.getName();
    knob1_joint.type = JointType::FIXED;
    knob1_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob1_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.015, -d_ + 0.1, 0.0);

    Joint knob2_joint(object_name_ + "_knob2_joint");
    knob2_joint.parent_link_name = cabinet_door_link.getName();
    knob2_joint.child_link_name = knob2_link.getName();
    knob2_joint.type = JointType::FIXED;
    knob2_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob2_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.015, -d_ + 0.1, -0.13);

    Joint knob3_joint(object_name_ + "_knob3_joint");
    knob3_joint.parent_link_name = knob1_link.getName();
    knob3_joint.child_link_name = handle_link.getName();
    knob3_joint.type = JointType::FIXED;
    knob3_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    knob3_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.02, 0, -0.065);

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link.clone()));
    link_map_[cabinet_bottom_link.getName()] =
        std::make_shared<Link>(std::move(cabinet_bottom_link.clone()));
    link_map_[level1_link.getName()] =
        std::make_shared<Link>(std::move(level1_link.clone()));
    link_map_[level2_link.getName()] =
        std::make_shared<Link>(std::move(level2_link.clone()));
    link_map_[level3_link.getName()] =
        std::make_shared<Link>(std::move(level3_link.clone()));
    link_map_[top_link.getName()] = std::make_shared<Link>(std::move(top_link.clone()));
    link_map_[cabinet_back_link.getName()] =
        std::make_shared<Link>(std::move(cabinet_back_link.clone()));
    link_map_[cabinet_left_link.getName()] =
        std::make_shared<Link>(std::move(cabinet_left_link.clone()));
    link_map_[cabinet_right_link.getName()] =
        std::make_shared<Link>(std::move(cabinet_right_link.clone()));
    link_map_[cabinet_door_link.getName()] =
        std::make_shared<Link>(std::move(cabinet_door_link.clone()));
    link_map_[knob1_link.getName()] =
        std::make_shared<Link>(std::move(knob1_link.clone()));
    link_map_[knob2_link.getName()] =
        std::make_shared<Link>(std::move(knob2_link.clone()));
    link_map_[handle_link.getName()] =
        std::make_shared<Link>(std::move(handle_link.clone()));

    joint_map_[cabinet_bottom_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_bottom_joint.clone()));
    joint_map_[cabinet_door_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_door_joint.clone()));
    joint_map_[cabinet_level1_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_level1_joint.clone()));
    joint_map_[cabinet_level2_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_level2_joint.clone()));
    joint_map_[cabinet_level3_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_level3_joint.clone()));
    joint_map_[cabinet_top_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_top_joint.clone()));
    joint_map_[cabinet_back_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_back_joint.clone()));
    joint_map_[cabinet_left_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_left_joint.clone()));
    joint_map_[cabinet_right_joint.getName()] =
        std::make_shared<Joint>(std::move(cabinet_right_joint.clone()));
    joint_map_[knob1_joint.getName()] =
        std::make_shared<Joint>(std::move(knob1_joint.clone()));
    joint_map_[knob2_joint.getName()] =
        std::make_shared<Joint>(std::move(knob2_joint.clone()));
    joint_map_[knob3_joint.getName()] =
        std::make_shared<Joint>(std::move(knob3_joint.clone()));

    AttachLocation attach_location("attach_" + handle_link.getName(),
                                   handle_link.getName());
    attach_location.local_joint_origin_transform.translation() +=
        // Eigen::Vector3d(-0.2, 0, 0.0);
        Eigen::Vector3d(-0.1, 0, 0.0);
    attach_location.local_joint_origin_transform.linear() =
        // Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).matrix();
        Eigen::Quaterniond(0.0, 0.0, 0.70710678, 0.70710678).matrix();
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
  double cabinet_x = 1.5;
  double cabinet_y = 0.0;
  double cabinet_z = 0.0;
  double pi_ = 3.1415926535897931;
  double bottom_thickness = 0.08;
  double wall_thickness = 0.02;
  double h_, w_, d_;
  Eigen::Vector4d color_;
};
}  // namespace vkc

#endif  // VKC_BASIC_CABINET_H