#ifndef VKC_BASIC_DRAWER_H
#define VKC_BASIC_DRAWER_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseDrawer : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseDrawer>;
  using ConstPtr = std::shared_ptr<const BaseDrawer>;

  BaseDrawer(std::string object_name) : BaseObject(object_name) {
    side_wall_height_ = drawer_height_ + 2 * gap_;
    side_wall_length_ = drawer_length_ + gap_;

    base_wall_width_ = drawer_width_ + 2 * gap_ + 2 * wall_thickness_;
    base_wall_length_ = drawer_length_ + gap_;

    back_wall_height_ = side_wall_height_ + 2 * wall_thickness_;
    back_wall_length_ = base_wall_width_;

    handle_gap_ = wall_thickness_ * 3;
    handle_thickness_ = wall_thickness_;
    handle_span_ = drawer_width_ / 4;
  }

  ~BaseDrawer() = default;

  bool createObject() override {
    std::string material_name;
    Link base_link(object_name_ + "_base_link");
    // Visual::Ptr base_link_visual = std::make_shared<Visual>();
    // base_link_visual->origin = Eigen::Isometry3d::Identity();
    // base_link_visual->geometry =
    // std::make_shared<tesseract_geometry::Box>(0.1, 0.1, 0.1);
    // base_link.visual.push_back(base_link_visual);

    Link drawer(object_name_ + "_drawer");
    Visual::Ptr drawer_visual = std::make_shared<Visual>();
    drawer_visual->origin = Eigen::Isometry3d::Identity();
    drawer_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        drawer_length_, drawer_width_, drawer_height_);
    material_name = drawer.getName() + "_color";
    drawer_visual->material = std::make_shared<Material>(material_name);
    drawer_visual->material->color = Eigen::Vector4d(0.6, 0.6, 0.6, 1.0);
    drawer.visual.push_back(drawer_visual);

    Collision::Ptr drawer_collision = std::make_shared<Collision>();
    drawer_collision->origin = drawer_visual->origin;
    drawer_collision->geometry = drawer_visual->geometry;
    drawer.collision.push_back(drawer_collision);

    Inertial::Ptr drawer_inertial = std::make_shared<Inertial>();
    drawer_inertial->mass = 0.1;
    drawer_inertial->origin = Eigen::Isometry3d::Identity();
    drawer_inertial->ixx =
        1.0 / 2.0 * drawer_mass_ *
        (std::pow(drawer_width_, 2) + std::pow(drawer_height_, 2));
    drawer_inertial->iyy =
        1.0 / 2.0 * drawer_mass_ *
        (std::pow(drawer_length_, 2) + std::pow(drawer_height_, 2));
    drawer_inertial->izz =
        1.0 / 2.0 * drawer_mass_ *
        (std::pow(drawer_length_, 2) + std::pow(drawer_width_, 2));
    drawer.inertial = drawer_inertial;

    Link left_wall(object_name_ + "_left_wall");

    Inertial::Ptr left_wall_inertial = std::make_shared<Inertial>();
    left_wall_inertial->mass = wall_mass_;
    left_wall_inertial->origin = Eigen::Isometry3d::Identity();
    left_wall_inertial->ixx =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(wall_thickness_, 2) + std::pow(side_wall_height_, 2));
    left_wall_inertial->iyy =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(side_wall_length_, 2) + std::pow(side_wall_height_, 2));
    left_wall_inertial->izz =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(side_wall_length_, 2) + std::pow(wall_thickness_, 2));
    left_wall.inertial = left_wall_inertial;

    Visual::Ptr left_wall_visual = std::make_shared<Visual>();
    left_wall_visual->origin = Eigen::Isometry3d::Identity();
    left_wall_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        side_wall_length_, wall_thickness_, side_wall_height_);
    material_name = left_wall.getName() + "_color";
    left_wall_visual->material = std::make_shared<Material>(material_name);
    left_wall_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    left_wall.visual.push_back(left_wall_visual);

    // Collision::Ptr left_wall_collision = std::make_shared<Collision>();
    // left_wall_collision->origin = left_wall_visual->origin;
    // left_wall_collision->geometry = left_wall_visual->geometry;
    // left_wall.collision.push_back(left_wall_collision);

    Link right_wall(object_name_ + "_right_wall");

    Inertial::Ptr right_wall_inertial = std::make_shared<Inertial>();
    right_wall_inertial->mass = wall_mass_;
    right_wall_inertial->origin = Eigen::Isometry3d::Identity();
    right_wall_inertial->ixx =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(wall_thickness_, 2) + std::pow(side_wall_height_, 2));
    right_wall_inertial->iyy =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(side_wall_length_, 2) + std::pow(side_wall_height_, 2));
    right_wall_inertial->izz =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(side_wall_length_, 2) + std::pow(wall_thickness_, 2));
    right_wall.inertial = right_wall_inertial;

    Visual::Ptr right_wall_visual = std::make_shared<Visual>();
    right_wall_visual->origin = Eigen::Isometry3d::Identity();
    right_wall_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        side_wall_length_, wall_thickness_, side_wall_height_);
    material_name = right_wall.getName() + "_color";
    right_wall_visual->material = std::make_shared<Material>(material_name);
    right_wall_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    right_wall.visual.push_back(right_wall_visual);

    // Collision::Ptr right_wall_collision = std::make_shared<Collision>();
    // right_wall_collision->origin = right_wall_visual->origin;
    // right_wall_collision->geometry = right_wall_visual->geometry;
    // right_wall.collision.push_back(right_wall_collision);

    Link bottom_wall(object_name_ + "_bottom_wall");

    Inertial::Ptr bottom_wall_inertial = std::make_shared<Inertial>();
    bottom_wall_inertial->mass = wall_mass_;
    bottom_wall_inertial->origin = Eigen::Isometry3d::Identity();
    bottom_wall_inertial->ixx =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(wall_thickness_, 2) + std::pow(base_wall_width_, 2));
    bottom_wall_inertial->iyy =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(base_wall_length_, 2) + std::pow(wall_thickness_, 2));
    bottom_wall_inertial->izz =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(base_wall_length_, 2) + std::pow(base_wall_width_, 2));
    bottom_wall.inertial = bottom_wall_inertial;

    Visual::Ptr bottom_wall_visual = std::make_shared<Visual>();
    bottom_wall_visual->origin = Eigen::Isometry3d::Identity();
    bottom_wall_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        base_wall_length_, base_wall_width_, wall_thickness_);
    material_name = bottom_wall.getName() + "_color";
    bottom_wall_visual->material = std::make_shared<Material>(material_name);
    bottom_wall_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    bottom_wall.visual.push_back(bottom_wall_visual);

    // Collision::Ptr bottom_wall_collision = std::make_shared<Collision>();
    // bottom_wall_collision->origin = bottom_wall_visual->origin;
    // bottom_wall_collision->geometry = bottom_wall_visual->geometry;
    // bottom_wall.collision.push_back(bottom_wall_collision);

    Link top_wall(object_name_ + "_top_wall");

    Inertial::Ptr top_wall_inertial = std::make_shared<Inertial>();
    top_wall_inertial->mass = wall_mass_;
    top_wall_inertial->origin = Eigen::Isometry3d::Identity();
    top_wall_inertial->ixx =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(wall_thickness_, 2) + std::pow(base_wall_width_, 2));
    top_wall_inertial->iyy =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(base_wall_length_, 2) + std::pow(wall_thickness_, 2));
    top_wall_inertial->izz =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(base_wall_length_, 2) + std::pow(base_wall_width_, 2));
    top_wall.inertial = top_wall_inertial;

    Visual::Ptr top_wall_visual = std::make_shared<Visual>();
    top_wall_visual->origin = Eigen::Isometry3d::Identity();
    top_wall_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        base_wall_length_, base_wall_width_, wall_thickness_);
    material_name = top_wall.getName() + "_color";
    top_wall_visual->material = std::make_shared<Material>(material_name);
    top_wall_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    top_wall.visual.push_back(top_wall_visual);

    // Collision::Ptr top_wall_collision = std::make_shared<Collision>();
    // top_wall_collision->origin = top_wall_visual->origin;
    // top_wall_collision->geometry = top_wall_visual->geometry;
    // top_wall.collision.push_back(top_wall_collision);

    Link back_wall(object_name_ + "_back_wall");

    Inertial::Ptr back_wall_inertial = std::make_shared<Inertial>();
    back_wall_inertial->mass = wall_mass_;
    back_wall_inertial->origin = Eigen::Isometry3d::Identity();
    back_wall_inertial->ixx =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(back_wall_length_, 2) + std::pow(back_wall_height_, 2));
    back_wall_inertial->iyy =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(back_wall_height_, 2) + std::pow(wall_thickness_, 2));
    back_wall_inertial->izz =
        1.0 / 2.0 * wall_mass_ *
        (std::pow(back_wall_length_, 2) + std::pow(back_wall_length_, 2));
    back_wall.inertial = back_wall_inertial;

    Visual::Ptr back_wall_visual = std::make_shared<Visual>();
    back_wall_visual->origin = Eigen::Isometry3d::Identity();
    back_wall_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        wall_thickness_, back_wall_length_, back_wall_height_);
    material_name = back_wall.getName() + "_color";
    back_wall_visual->material = std::make_shared<Material>(material_name);
    back_wall_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    back_wall.visual.push_back(back_wall_visual);

    // Collision::Ptr back_wall_collision = std::make_shared<Collision>();
    // back_wall_collision->origin = back_wall_visual->origin;
    // back_wall_collision->geometry = back_wall_visual->geometry;
    // back_wall.collision.push_back(back_wall_collision);

    Link handle_left(object_name_ + "_handle_left");

    Inertial::Ptr handle_left_inertial = std::make_shared<Inertial>();
    handle_left_inertial->mass = wall_mass_;
    handle_left_inertial->origin = Eigen::Isometry3d::Identity();
    handle_left_inertial->ixx =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_thickness_, 2));
    handle_left_inertial->iyy =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_gap_, 2));
    handle_left_inertial->izz =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_gap_, 2));
    handle_left.inertial = handle_left_inertial;

    Visual::Ptr handle_left_visual = std::make_shared<Visual>();
    handle_left_visual->origin = Eigen::Isometry3d::Identity();
    handle_left_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        handle_gap_, handle_thickness_, handle_thickness_);
    material_name = handle_left.getName() + "_color";
    handle_left_visual->material = std::make_shared<Material>(material_name);
    handle_left_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    handle_left.visual.push_back(handle_left_visual);

    // Collision::Ptr handle_left_collision = std::make_shared<Collision>();
    // handle_left_collision->origin = handle_left_visual->origin;
    // handle_left_collision->geometry = handle_left_visual->geometry;
    // handle_left.collision.push_back(handle_left_collision);

    Link handle_right(object_name_ + "_handle_right");

    Inertial::Ptr handle_right_inertial = std::make_shared<Inertial>();
    handle_right_inertial->mass = wall_mass_;
    handle_right_inertial->origin = Eigen::Isometry3d::Identity();
    handle_right_inertial->ixx =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_thickness_, 2));
    handle_right_inertial->iyy =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_gap_, 2));
    handle_right_inertial->izz =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_gap_, 2));
    handle_right.inertial = handle_right_inertial;

    Visual::Ptr handle_right_visual = std::make_shared<Visual>();
    handle_right_visual->origin = Eigen::Isometry3d::Identity();
    handle_right_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        handle_gap_, handle_thickness_, handle_thickness_);
    material_name = handle_right.getName() + "_color";
    handle_right_visual->material = std::make_shared<Material>(material_name);
    handle_right_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    handle_right.visual.push_back(handle_right_visual);

    // Collision::Ptr handle_right_collision = std::make_shared<Collision>();
    // handle_right_collision->origin = handle_right_visual->origin;
    // handle_right_collision->geometry = handle_right_visual->geometry;
    // handle_right.collision.push_back(handle_right_collision);

    Link handle_link(object_name_ + "_handle_link");

    Inertial::Ptr handle_link_inertial = std::make_shared<Inertial>();
    handle_link_inertial->mass = wall_mass_;
    handle_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle_link_inertial->ixx =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) +
         std::pow(handle_span_ + 2 * handle_thickness_, 2));
    handle_link_inertial->iyy =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) + std::pow(handle_thickness_, 2));
    handle_link_inertial->izz =
        1 / 2 * wall_mass_ *
        (std::pow(handle_thickness_, 2) +
         std::pow(handle_span_ + 2 * handle_thickness_, 2));
    handle_link.inertial = handle_link_inertial;

    Visual::Ptr handle_link_visual = std::make_shared<Visual>();
    handle_link_visual->origin = Eigen::Isometry3d::Identity();
    handle_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        handle_thickness_, handle_span_ + 2 * handle_thickness_,
        handle_thickness_);
    material_name = handle_link.getName() + "_color";
    handle_link_visual->material = std::make_shared<Material>(material_name);
    handle_link_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    handle_link.visual.push_back(handle_link_visual);

    // Collision::Ptr handle_link_collision = std::make_shared<Collision>();
    // handle_link_collision->origin = handle_link_visual->origin;
    // handle_link_collision->geometry = handle_link_visual->geometry;
    // handle_link.collision.push_back(handle_link_collision);

    /*
    Link handle_top_cover(object_name_ + "_handle_top_cover");

    Inertial::Ptr handle_top_cover_inertial = std::make_shared<Inertial>();
    handle_top_cover_inertial->mass = wall_mass_;
    handle_top_cover_inertial->origin = Eigen::Isometry3d::Identity();
    handle_top_cover_inertial->ixx =
        1 / 2 * wall_mass_ * (std::pow(handle_thickness_, 2) +
    std::pow(handle_span_ + 2 * handle_thickness_, 2));
    handle_top_cover_inertial->iyy = 1 / 2 * wall_mass_ * (std::pow(handle_gap_,
    2) + std::pow(handle_thickness_, 2)); handle_top_cover_inertial->izz = 1 / 2
    * wall_mass_ * (std::pow(handle_gap_, 2) + std::pow(handle_span_ + 2 *
    handle_thickness_, 2)); handle_top_cover.inertial =
    handle_top_cover_inertial;

    Visual::Ptr handle_top_cover_visual = std::make_shared<Visual>();
    handle_top_cover_visual->origin = Eigen::Isometry3d::Identity();
    handle_top_cover_visual->geometry =
    std::make_shared<tesseract_geometry::Box>( handle_gap_ + handle_thickness_,
    handle_span_ + 2 * handle_thickness_, handle_thickness_ * 0.3);
    material_name = handle_top_cover.getName() + "_color";
    handle_top_cover_visual->material =
    std::make_shared<Material>(material_name);
    handle_top_cover_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    handle_top_cover.visual.push_back(handle_top_cover_visual);

    Link handle_bottom_cover(object_name_ + "_handle_bottom_cover");

    Inertial::Ptr handle_bottom_cover_inertial = std::make_shared<Inertial>();
    handle_bottom_cover_inertial->mass = wall_mass_;
    handle_bottom_cover_inertial->origin = Eigen::Isometry3d::Identity();
    handle_bottom_cover_inertial->ixx =
        1 / 2 * wall_mass_ * (std::pow(handle_thickness_, 2) +
    std::pow(handle_span_ + 2 * handle_thickness_, 2));
    handle_bottom_cover_inertial->iyy =
        1 / 2 * wall_mass_ * (std::pow(handle_gap_, 2) +
    std::pow(handle_thickness_, 2)); handle_bottom_cover_inertial->izz = 1 / 2 *
    wall_mass_ * (std::pow(handle_gap_, 2) + std::pow(handle_span_ + 2 *
    handle_thickness_, 2)); handle_bottom_cover.inertial =
    handle_bottom_cover_inertial;

    Visual::Ptr handle_bottom_cover_visual = std::make_shared<Visual>();
    handle_bottom_cover_visual->origin = Eigen::Isometry3d::Identity();
    handle_bottom_cover_visual->geometry =
    std::make_shared<tesseract_geometry::Box>( handle_thickness_*0.3,
    handle_span_ + 2 * handle_thickness_, (handle_gap_ +
    handle_thickness_)*1.4142); material_name = handle_bottom_cover.getName() +
    "_color"; handle_bottom_cover_visual->material =
    std::make_shared<Material>(material_name);
    handle_bottom_cover_visual->material->color = Eigen::Vector4d(1, 1, 1, 1.0);
    handle_bottom_cover.visual.push_back(handle_bottom_cover_visual);

    Collision::Ptr handle_bottom_cover_collision =
    std::make_shared<Collision>(); handle_bottom_cover_collision->origin =
    handle_bottom_cover_visual->origin; handle_bottom_cover_collision->geometry
    = handle_bottom_cover_visual->geometry;
    handle_bottom_cover.collision.push_back(handle_bottom_cover_collision);
    */
    Joint base_drawer_joint(object_name_ + "_base_drawer_joint");
    base_drawer_joint.parent_link_name = base_link.getName();
    base_drawer_joint.child_link_name = drawer.getName();
    base_drawer_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_drawer_joint.type = JointType::PRISMATIC;
    base_drawer_joint.axis = Eigen::Vector3d(1, 0, 0);
    base_drawer_joint.limits = std::make_shared<JointLimits>();
    base_drawer_joint.limits->lower = -gap_;
    base_drawer_joint.limits->upper = 0.8;
    base_drawer_joint.limits->effort = 1000;
    base_drawer_joint.limits->velocity = 10.0;
    base_drawer_joint.dynamics = std::make_shared<JointDynamics>();
    base_drawer_joint.dynamics->damping = 100.0;
    base_drawer_joint.dynamics->friction = 0.0;

    Joint base_left_wall_joint(object_name_ + "_base_left_wall_joint");
    base_left_wall_joint.parent_link_name = base_link.getName();
    base_left_wall_joint.child_link_name = left_wall.getName();
    base_left_wall_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_left_wall_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-gap_,
                        -(drawer_width_ / 2 + wall_thickness_ / 2) - gap_, 0);
    base_left_wall_joint.type = JointType::FIXED;

    Joint base_right_wall_joint(object_name_ + "_base_right_wall_joint");
    base_right_wall_joint.parent_link_name = base_link.getName();
    base_right_wall_joint.child_link_name = right_wall.getName();
    base_right_wall_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_right_wall_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-gap_, (drawer_width_ / 2 + wall_thickness_ / 2) + gap_,
                        0);
    base_right_wall_joint.type = JointType::FIXED;

    Joint base_bottom_wall_joint(object_name_ + "_base_bottom_wall_joint");
    base_bottom_wall_joint.parent_link_name = base_link.getName();
    base_bottom_wall_joint.child_link_name = bottom_wall.getName();
    base_bottom_wall_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_bottom_wall_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-gap_, 0,
                        -(drawer_height_ / 2 + wall_thickness_ / 2) - gap_);
    base_bottom_wall_joint.type = JointType::FIXED;

    Joint base_top_wall_joint(object_name_ + "_base_top_wall_joint");
    base_top_wall_joint.parent_link_name = base_link.getName();
    base_top_wall_joint.child_link_name = top_wall.getName();
    base_top_wall_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_top_wall_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-gap_, 0,
                        (drawer_height_ / 2 + wall_thickness_ / 2) + gap_);
    base_top_wall_joint.type = JointType::FIXED;

    Joint base_back_wall_joint(object_name_ + "_base_back_wall_joint");
    base_back_wall_joint.parent_link_name = base_link.getName();
    base_back_wall_joint.child_link_name = back_wall.getName();
    base_back_wall_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    base_back_wall_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-(drawer_length_ / 2 + gap_ + wall_thickness_ / 2), 0,
                        0);
    base_back_wall_joint.type = JointType::FIXED;

    Joint drawer_handle_left_joint(object_name_ + "_drawer_handle_left_joint");
    drawer_handle_left_joint.parent_link_name = drawer.getName();
    drawer_handle_left_joint.child_link_name = handle_left.getName();
    drawer_handle_left_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    drawer_handle_left_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(drawer_length_ / 2 + handle_gap_ / 2,
                        -(handle_span_ / 2 + handle_thickness_ / 2), 0);
    drawer_handle_left_joint.type = JointType::FIXED;

    Joint drawer_handle_right_joint(object_name_ +
                                    "_drawer_handle_right_joint");
    drawer_handle_right_joint.parent_link_name = drawer.getName();
    drawer_handle_right_joint.child_link_name = handle_right.getName();
    drawer_handle_right_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    drawer_handle_right_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(drawer_length_ / 2 + handle_gap_ / 2,
                        (handle_span_ / 2 + handle_thickness_ / 2), 0);
    drawer_handle_right_joint.type = JointType::FIXED;

    Joint drawer_handle_link_joint(object_name_ + "_drawer_handle_link_joint");
    drawer_handle_link_joint.parent_link_name = drawer.getName();
    drawer_handle_link_joint.child_link_name = handle_link.getName();
    drawer_handle_link_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    drawer_handle_link_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(
            drawer_length_ / 2 + handle_gap_ + handle_thickness_ / 2, 0, 0);
    drawer_handle_link_joint.type = JointType::FIXED;

    /*
    Joint drawer_handle_top_cover_joint(object_name_ +
    "_drawer_handle_top_cover_joint");
    drawer_handle_top_cover_joint.parent_link_name = drawer.getName();
    drawer_handle_top_cover_joint.child_link_name = handle_top_cover.getName();
    drawer_handle_top_cover_joint.parent_to_joint_origin_transform =
    Eigen::Isometry3d::Identity();
    drawer_handle_top_cover_joint.parent_to_joint_origin_transform.translation()
    += Eigen::Vector3d(drawer_length_ / 2 + handle_gap_ /2 + handle_thickness_ /
    2, 0, -handle_thickness_ * 0.3 / 2.0 - 2.0 * handle_thickness_);
    drawer_handle_top_cover_joint.type = JointType::FIXED;

    Joint drawer_handle_bottom_cover_joint(object_name_ +
    "_drawer_handle_bottom_cover_joint");
    drawer_handle_bottom_cover_joint.parent_link_name =
    handle_top_cover.getName(); drawer_handle_bottom_cover_joint.child_link_name
    = handle_bottom_cover.getName();
    drawer_handle_bottom_cover_joint.parent_to_joint_origin_transform =
    Eigen::Isometry3d::Identity();
    drawer_handle_bottom_cover_joint.parent_to_joint_origin_transform.translation()
    += Eigen::Vector3d(-handle_thickness_ * 0.1 / 2, 0, -(handle_gap_ +
    handle_thickness_) / 2);
    drawer_handle_bottom_cover_joint.parent_to_joint_origin_transform.linear() =
        Eigen::AngleAxisd(0.78539816339,
    Eigen::Vector3d::UnitY()).toRotationMatrix();
    drawer_handle_bottom_cover_joint.type = JointType::FIXED;
    */

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link.clone()));
    link_map_[drawer.getName()] = std::make_shared<Link>(std::move(drawer.clone()));
    link_map_[left_wall.getName()] =
        std::make_shared<Link>(std::move(left_wall.clone()));
    link_map_[right_wall.getName()] =
        std::make_shared<Link>(std::move(right_wall.clone()));
    link_map_[bottom_wall.getName()] =
        std::make_shared<Link>(std::move(bottom_wall.clone()));
    link_map_[top_wall.getName()] = std::make_shared<Link>(std::move(top_wall.clone()));
    link_map_[back_wall.getName()] =
        std::make_shared<Link>(std::move(back_wall.clone()));
    link_map_[handle_link.getName()] =
        std::make_shared<Link>(std::move(handle_link.clone()));
    link_map_[handle_left.getName()] =
        std::make_shared<Link>(std::move(handle_left.clone()));
    link_map_[handle_right.getName()] =
        std::make_shared<Link>(std::move(handle_right.clone()));
    // link_map_[handle_top_cover.getName()] =
    // std::make_shared<Link>(std::move(handle_top_cover.clone()));
    // link_map_[handle_bottom_cover.getName()] =
    // std::make_shared<Link>(std::move(handle_bottom_cover.clone()));

    joint_map_[base_drawer_joint.getName()] =
        std::make_shared<Joint>(std::move(base_drawer_joint.clone()));
    joint_map_[base_left_wall_joint.getName()] =
        std::make_shared<Joint>(std::move(base_left_wall_joint.clone()));
    joint_map_[base_right_wall_joint.getName()] =
        std::make_shared<Joint>(std::move(base_right_wall_joint.clone()));
    joint_map_[base_back_wall_joint.getName()] =
        std::make_shared<Joint>(std::move(base_back_wall_joint.clone()));
    joint_map_[base_bottom_wall_joint.getName()] =
        std::make_shared<Joint>(std::move(base_bottom_wall_joint.clone()));
    joint_map_[base_top_wall_joint.getName()] =
        std::make_shared<Joint>(std::move(base_top_wall_joint.clone()));
    // joint_map_[drawer_handle_bottom_cover_joint.getName()] =
    //     std::make_shared<Joint>(std::move(drawer_handle_bottom_cover_joint.clone()));
    joint_map_[drawer_handle_link_joint.getName()] =
        std::make_shared<Joint>(std::move(drawer_handle_link_joint.clone()));
    joint_map_[drawer_handle_left_joint.getName()] =
        std::make_shared<Joint>(std::move(drawer_handle_left_joint.clone()));
    joint_map_[drawer_handle_right_joint.getName()] =
        std::make_shared<Joint>(std::move(drawer_handle_right_joint.clone()));
    // joint_map_[drawer_handle_top_cover_joint.getName()] =
    //     std::make_shared<Joint>(std::move(drawer_handle_top_cover_joint.clone()));

    // Add an attach location
    AttachLocation attach_location(
        fmt::format("attach_{}", handle_link.getName()), handle_link.getName());

    attach_location.local_joint_origin_transform.setIdentity();
    attach_location.local_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.15, 0.0, 0.0);
    attach_location.local_joint_origin_transform.linear() =
        // Eigen::Quaterniond(0.0, 0.0, 0.70710678, 0.70710678).matrix();
        Eigen::Quaterniond(0.5,0.5,-0.5,-0.5).matrix();
    attach_location.fixed_base = true;

    // Define connection joint
    attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
    // attach_location.connection.type = JointType::REVOLUTE;
    // attach_location.connection.axis = Eigen::Vector3d(0, 0, 1);
    // attach_location.connection.limits = std::make_shared<JointLimits>();
    // attach_location.connection.limits->lower = -1.2;
    // attach_location.connection.limits->upper = 1.2;
    attach_location.connection.child_link_name = handle_link.getName();
    attach_location.connection.parent_link_name = "NULL";

    // Create scene graph
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

 private:
  double drawer_height_ = 0.3;
  double drawer_width_ = 0.6;
  double drawer_length_ = 1.0;
  double gap_ = 0.01;
  double wall_thickness_ = 0.02;
  double drawer_mass_ = 0.1;
  double wall_mass_ = 0.05;
  double side_wall_height_, side_wall_length_, base_wall_width_,
      base_wall_length_, back_wall_height_, back_wall_length_, handle_gap_,
      handle_thickness_, handle_span_;
};
}  // namespace vkc
#endif  // VKC_BASIC_DRAWER_H