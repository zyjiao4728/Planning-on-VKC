#ifndef VKC_BASIC_DOOR_H
#define VKC_BASIC_DOOR_H

#include <vkc/object/basic_object.h>

namespace vkc {
class BaseDoor : public BaseObject {
 public:
  using Ptr = std::shared_ptr<BaseDoor>;
  using ConstPtr = std::shared_ptr<const BaseDoor>;

  BaseDoor(std::string object_name, double door_width = 1.3,
           std::string dir = "right")
      : BaseObject(object_name), door_width_(door_width), dir_(dir) {
    mir_ = 0;
    if (dir == "left") {
      mir_ = -1;
    } else if (dir == "right") {
      mir_ = 1;
    } else {
      ROS_WARN("Unknown door direction...");
    }
  }

  ~BaseDoor() = default;

  bool createObject() override {
    std::string material_name;

    assert(mir_ != 0);

    Link base_link(object_name_ + "_base_link");

    Visual::Ptr base_link_visual = std::make_shared<Visual>();
    base_link_visual->origin = Eigen::Isometry3d::Identity();
    base_link_visual->origin.translation() +=
        Eigen::Vector3d(0, 0, door_height_ / 2);
    base_link_visual->geometry =
        std::make_shared<tesseract_geometry::Box>(0.1, 0.15, door_height_);
    material_name = base_link.getName() + "_color";
    base_link_visual->material = std::make_shared<Material>(material_name);
    base_link_visual->material->color = Eigen::Vector4d(0.4, 0.2, 0.0, 0.0);
    base_link.visual.push_back(base_link_visual);

    Link door_link(object_name_ + "_door_link");

    Inertial::Ptr door_link_inertial = std::make_shared<Inertial>();
    door_link_inertial->mass = 30.0;
    door_link_inertial->origin = Eigen::Isometry3d::Identity();
    door_link_inertial->origin.translation() +=
        Eigen::Vector3d(0, -mir_ * door_width_ / 2.0, door_height_ / 2);
    door_link_inertial->ixx = 1.0;
    door_link_inertial->iyy = 1.0;
    door_link_inertial->izz = 3.0;
    door_link.inertial = door_link_inertial;

    Visual::Ptr door_link_visual = std::make_shared<Visual>();
    door_link_visual->origin = Eigen::Isometry3d::Identity();
    door_link_visual->origin.translation() +=
        Eigen::Vector3d(0, -mir_ * door_width_ / 2.0, door_height_ / 2);
    door_link_visual->origin.linear() =
        Eigen::AngleAxisd(pi_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    door_link_visual->geometry = std::make_shared<tesseract_geometry::Box>(
        0.05, door_width_, door_height_);
    material_name = door_link.getName() + "_color";
    door_link_visual->material = std::make_shared<Material>(material_name);
    door_link_visual->material->color = Eigen::Vector4d(0.4, 0.2, 0.0, 1.0);
    door_link.visual.push_back(door_link_visual);

    Collision::Ptr door_link_collision = std::make_shared<Collision>();
    door_link_collision->origin = door_link_visual->origin;
    door_link_collision->geometry = door_link_visual->geometry;
    door_link.collision.push_back(door_link_collision);

    std::string handle_link_name = fmt::format("{}_handle_link", object_name_);

    Link handle_link(handle_link_name);

    Inertial::Ptr handle_link_inertial = std::make_shared<Inertial>();
    handle_link_inertial->mass = 0.3;
    handle_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle_link_inertial->origin.translation() +=
        Eigen::Vector3d(0, mir_ * (handle_length_ / 2.0), 0.0);
    handle_link_inertial->ixx = 0.01;
    handle_link_inertial->iyy = 0.01;
    handle_link_inertial->izz = 0.01;
    handle_link.inertial = handle_link_inertial;

    Visual::Ptr handle_link_visual = std::make_shared<Visual>();
    handle_link_visual->origin = Eigen::Isometry3d::Identity();
    handle_link_visual->origin.translation() +=
        Eigen::Vector3d(0, mir_ * (handle_length_ / 2.0), 0.0);
    handle_link_visual->origin.linear() =
        Eigen::AngleAxisd(pi_ / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    handle_link_visual->geometry =
        std::make_shared<tesseract_geometry::Cylinder>(0.013, handle_length_);
    material_name = fmt::format("{}_color", handle_link.getName());
    handle_link_visual->material = std::make_shared<Material>(material_name);
    handle_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    handle_link.visual.push_back(handle_link_visual);

    Collision::Ptr handle_link_collision = std::make_shared<Collision>();
    handle_link_collision->origin = handle_link_visual->origin;
    handle_link_collision->geometry = handle_link_visual->geometry;
    handle_link.collision.push_back(handle_link_collision);

    Link handle1_link(object_name_ + "_handle1_link");

    Inertial::Ptr handle1_link_inertial = std::make_shared<Inertial>();
    handle1_link_inertial->mass = 0.1;
    handle1_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle1_link_inertial->ixx = 0.01;
    handle1_link_inertial->iyy = 0.01;
    handle1_link_inertial->izz = 0.001;
    handle1_link.inertial = handle1_link_inertial;

    Visual::Ptr handle1_link_visual = std::make_shared<Visual>();
    handle1_link_visual->origin = Eigen::Isometry3d::Identity();
    handle1_link_visual->origin.translation() +=
        Eigen::Vector3d(0.025, 0.0, 0.0);
    handle1_link_visual->origin.linear() =
        Eigen::AngleAxisd(pi_ / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    handle1_link_visual->geometry =
        std::make_shared<tesseract_geometry::Cylinder>(0.01, 0.05);
    material_name = handle1_link.getName() + "_color";
    handle1_link_visual->material = std::make_shared<Material>(material_name);
    handle1_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    handle1_link.visual.push_back(handle1_link_visual);

    Collision::Ptr handle1_link_collision = std::make_shared<Collision>();
    handle1_link_collision->origin = handle1_link_visual->origin;
    handle1_link_collision->geometry = handle1_link_visual->geometry;
    handle1_link.collision.push_back(handle1_link_collision);

    Link handle2_link(object_name_ + "_handle2_link");

    Inertial::Ptr handle2_link_inertial = std::make_shared<Inertial>();
    handle2_link_inertial->mass = 0.1;
    handle2_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle2_link_inertial->ixx = 0.01;
    handle2_link_inertial->iyy = 0.01;
    handle2_link_inertial->izz = 0.001;
    handle2_link.inertial = handle2_link_inertial;

    Visual::Ptr handle2_link_visual = std::make_shared<Visual>();
    handle2_link_visual->origin = Eigen::Isometry3d::Identity();
    handle2_link_visual->origin.translation() +=
        Eigen::Vector3d(0.025, 0.0, 0.0);
    handle2_link_visual->origin.linear() =
        Eigen::AngleAxisd(pi_ / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    handle2_link_visual->geometry =
        std::make_shared<tesseract_geometry::Cylinder>(0.01, 0.05);
    material_name = handle2_link.getName() + "_color";
    handle2_link_visual->material = std::make_shared<Material>(material_name);
    handle2_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    handle2_link.visual.push_back(handle2_link_visual);

    Collision::Ptr handle2_link_collision = std::make_shared<Collision>();
    handle2_link_collision->origin = handle2_link_visual->origin;
    handle2_link_collision->geometry = handle2_link_visual->geometry;
    handle2_link.collision.push_back(handle2_link_collision);

    Link handle3_link(object_name_ + "_handle3_link");

    Inertial::Ptr handle3_link_inertial = std::make_shared<Inertial>();
    handle3_link_inertial->mass = 0.1;
    handle3_link_inertial->origin = Eigen::Isometry3d::Identity();
    handle3_link_inertial->ixx = 0.01;
    handle3_link_inertial->iyy = 0.01;
    handle3_link_inertial->izz = 0.001;
    handle3_link.inertial = handle3_link_inertial;

    Visual::Ptr handle3_link_visual = std::make_shared<Visual>();
    handle3_link_visual->origin = Eigen::Isometry3d::Identity();
    handle3_link_visual->origin.translation() +=
        Eigen::Vector3d(0.06, 0.0, 0.0);
    handle3_link_visual->origin.linear() =
        Eigen::AngleAxisd(pi_ / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    handle3_link_visual->geometry =
        std::make_shared<tesseract_geometry::Cylinder>(0.03, 0.02);
    material_name = handle3_link.getName() + "_color";
    handle3_link_visual->material = std::make_shared<Material>(material_name);
    handle3_link_visual->material->color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.0);
    handle3_link.visual.push_back(handle3_link_visual);

    Collision::Ptr handle3_link_collision = std::make_shared<Collision>();
    handle3_link_collision->origin = handle3_link_visual->origin;
    handle3_link_collision->geometry = handle3_link_visual->geometry;
    handle3_link.collision.push_back(handle3_link_collision);

    Joint door_joint(object_name_ + "_door_joint");
    door_joint.parent_link_name = base_link.getName();
    door_joint.child_link_name = door_link.getName();
    door_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    door_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0, 0.00, 0.0);
    door_joint.type = JointType::REVOLUTE;
    door_joint.axis = Eigen::Vector3d(0, 0, 1);
    door_joint.limits = std::make_shared<JointLimits>();
    door_joint.limits->lower = -pi_ / 2.0;
    door_joint.limits->upper = pi_ / 2.0;
    door_joint.limits->effort = 1000;
    door_joint.limits->velocity = 10;
    door_joint.dynamics = std::make_shared<JointDynamics>();
    door_joint.dynamics->damping = 100;
    door_joint.dynamics->friction = 0.0;

    Joint handle_joint(fmt::format("{}_handle_joint", object_name_));
    handle_joint.parent_link_name = door_link.getName();
    handle_joint.child_link_name = handle_link.getName();
    handle_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    handle_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(-0.125, mir_ * (-door_width_ + 0.08), handle_height_);
    handle_joint.type = JointType::FIXED;
    // handle_joint.axis = Eigen::Vector3d(1, 0, 0);
    // handle_joint.limits = std::make_shared<JointLimits>();
    // handle_joint.limits->lower = -pi_/2;
    // handle_joint.limits->upper = pi_/2;
    // handle_joint.limits->effort = 1000;
    // handle_joint.limits->velocity = 10;
    // handle_joint.dynamics = std::make_shared<JointDynamics>();
    // handle_joint.dynamics->damping = 100;
    // handle_joint.dynamics->friction = 0.0;

    Joint handle1_joint(object_name_ + "_handle1_joint");
    handle1_joint.parent_link_name = handle_link.getName();
    handle1_joint.child_link_name = handle1_link.getName();
    handle1_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    handle1_joint.type = JointType::FIXED;

    Joint handle2_joint(object_name_ + "_handle2_joint");
    handle2_joint.parent_link_name = handle_link.getName();
    handle2_joint.child_link_name = handle2_link.getName();
    handle2_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    handle2_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(0.0, mir_ * 0.12, 0.0);
    handle2_joint.type = JointType::FIXED;

    Joint handle3_joint(object_name_ + "_handle3_joint");
    handle3_joint.parent_link_name = handle_link.getName();
    handle3_joint.child_link_name = handle3_link.getName();
    handle3_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    handle3_joint.type = JointType::FIXED;

    link_map_[base_link.getName()] =
        std::make_shared<Link>(std::move(base_link.clone()));
    link_map_[door_link.getName()] =
        std::make_shared<Link>(std::move(door_link.clone()));
    link_map_[handle_link.getName()] =
        std::make_shared<Link>(std::move(handle_link.clone()));

    // link_map_[handle1_link.getName()] =
    // std::make_shared<Link>(std::move(handle1_link));
    // link_map_[handle2_link.getName()] =
    // std::make_shared<Link>(std::move(handle2_link));
    // link_map_[handle3_link.getName()] =
    // std::make_shared<Link>(std::move(handle3_link));

    joint_map_[door_joint.getName()] =
        std::make_shared<Joint>(std::move(door_joint.clone()));
    joint_map_[handle_joint.getName()] =
        std::make_shared<Joint>(std::move(handle_joint.clone()));
    // joint_map_[handle1_joint.getName()] =
    // std::make_shared<Joint>(std::move(handle1_joint));
    // joint_map_[handle2_joint.getName()] =
    // std::make_shared<Joint>(std::move(handle2_joint));
    // joint_map_[handle3_joint.getName()] =
    // std::make_shared<Joint>(std::move(handle3_joint));

    // Add an attach location
    AttachLocation attach_location(
        fmt::format("attach_{}", handle_link.getName()), handle_link.getName());

    attach_location.local_joint_origin_transform.setIdentity();
    
    // attach_location.local_joint_origin_transform.translation() +=
    // Eigen::Vector3d(-0.15, mir_ * 0.08, 0.0);
    // attach_location.local_joint_origin_transform.linear() =
    // Eigen::Quaterniond(0.7071, 0.7071, 0.0, 0.0).matrix();
    attach_location.local_joint_origin_transform.translation() +=
            Eigen::Vector3d(-0.15, mir_ * 0.08, 0.0);
        // Eigen::Vector3d(-0.15, mir_ * 0.08, 0.0);
    // attach_location.local_joint_origin_transform.linear() =
        // Eigen::Quaterniond(0.7071, 0.0, 0.7071, 0.0).matrix();
    attach_location.local_joint_origin_transform.linear() =
        // Eigen::Quaterniond(0.70710678, 0, 0.70710678, 0).matrix();
        Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).matrix();
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

  void setHandleHeight(double handle_height) {
    handle_height_ = handle_height;
    return;
  }

  void setHandleLength(double handle_length) {
    handle_length_ = handle_length;
    return;
  }

 private:
  double pi_ = 3.1415926535897931;
  double door_height_ = 2.06;
  double handle_height_ = 0.95;
  double handle_length_ = 0.10;
  double door_width_;
  std::string dir_;
  int mir_;
};
}  // namespace vkc

#endif  // VKC_BASIC_DOOR_H