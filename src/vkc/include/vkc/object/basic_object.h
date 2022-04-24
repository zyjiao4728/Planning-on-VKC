#ifndef VKC_BASIC_OBJECT_H
#define VKC_BASIC_OBJECT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/utils.h>

// #include <vkc/construct_vkc.h>

#include <fmt/core.h>

#include <boost/range/adaptor/reversed.hpp>
#include <cmath>
#include <iostream>

using namespace tesseract_scene_graph;
using namespace tesseract_environment;
using namespace tesseract_monitoring;

namespace vkc {
/**
 * @brief The basic object base class.
 * The class provides a basic structure of articulated objects or tree-structure
 * objects to be deployed in the environment.
 */
class BaseObject {
 public:
  BaseObject(std::string object_name) : object_name_(object_name) {
    clear();
    object_scene_graph_ = std::make_shared<SceneGraph>();
  }
  virtual ~BaseObject() = default;

  /**< @brief Create object. Assuming pose = (x,y,z,theta), from world frame to
   * base link */
  virtual bool createObject() = 0;

  std::string getName() { return object_name_; }

  /**
   * @brief An attach location consists of the name of parent/child link,
   * transformation from its parent link to its location.
   * Also the attachment (joint) type after the connection is established
   */
  struct AttachLocation {
    using Ptr = std::shared_ptr<AttachLocation>;
    using ConstPtr = std::shared_ptr<const AttachLocation>;

    std::string name_;      /**< @brief The name of the attachment. */
    std::string link_name_; /**< @brief Link to attach (reference). */
    std::string base_link_; /**< @brief Base link name of the object. */
    Eigen::Isometry3d
        local_joint_origin_transform; /**< @brief Transform from local link
                                         frame to Joint frame */
    Eigen::Isometry3d
        world_joint_origin_transform; /**< @brief Transform from world frame to
                                         Joint frame */
    Joint connection; /**< @brief Joint to be established between end effector
                         and link */
    bool fixed_base =
        false; /**< @brief If base_link is physicallly fixed to groud */

    // Constructor
    AttachLocation(std::string name, std::string link_name)
        : name_(name),
          link_name_(std::move(link_name)),
          connection(fmt::format("{}_joint", name)) {
      local_joint_origin_transform = Eigen::Isometry3d::Identity();
      world_joint_origin_transform = Eigen::Isometry3d::Identity();
    }
  };

  void addAttachLocation(AttachLocation::Ptr al) {
    // AttachLocation::Ptr al =
    // std::make_shared<AttachLocation>(std::move(attach_location));
    al->base_link_ = object_scene_graph_->getRoot();
    attach_locations_[al->name_] = al;
    return;
  }

  AttachLocation::Ptr getAttachLocation(std::string name) {
    auto attach_location = attach_locations_.find(name);

    if (attach_location == attach_locations_.end()) return nullptr;

    return attach_location->second;
  }

  std::unordered_map<std::string, AttachLocation::Ptr> getAttachLocation() {
    return attach_locations_;
  }

  bool setEndEffectorToAttachLocation(std::string name,
                                      std::string parent_link) {
    auto attach_location = attach_locations_.find(name);

    if (attach_location == attach_locations_.end()) {
      ROS_ERROR("Undefined attach location. Unable to set parent link.");
      return false;
    }

    attach_location->second->connection.parent_link_name = parent_link;
    return true;
  }

  bool setAttachLocationWorldTransform(Environment::Ptr env_) {
    if (attach_locations_.size() == 0) {
      ROS_WARN("Empty attach locations!");
      return false;
    }

    for (auto &it : attach_locations_) {
      it.second->world_joint_origin_transform =
          env_->getLinkTransform(it.second->link_name_) *
          it.second->local_joint_origin_transform;
      // std::cout << "translation of attachment " << it.second->name_ << " in
      // world frame: " << std::endl; std::cout <<
      // it.second->world_joint_origin_transform.translation() << std::endl;
      // std::cout << "rotation matrix of attachment " << it.second->name_ << "
      // in world frame: " << std::endl; std::cout <<
      // it.second->world_joint_origin_transform.rotation() << std::endl;
    }

    return true;
  }

  bool setAttachLocationWorldTransform(Eigen::Isometry3d transform,
                                       std::string name) {
    auto attach_location = attach_locations_.find(name);

    if (attach_location == attach_locations_.end()) {
      ROS_ERROR(
          "[%s]Undefined attach location %s. Unable to set world transform.",
          __func__, name.c_str());
      return false;
    }

    attach_location->second->world_joint_origin_transform =
        transform * attach_location->second->local_joint_origin_transform;
    // std::cout << "translation of attachment " <<
    // attach_location->second->name_ << " in world frame: " << std::endl;
    // std::cout <<
    // attach_location->second->world_joint_origin_transform.translation() <<
    // std::endl; std::cout << "rotation matrix of attachment " <<
    // attach_location->second->name_ << " in world frame: " << std::endl;
    // std::cout <<
    // attach_location->second->world_joint_origin_transform.rotation() <<
    // std::endl;

    return true;
  }

  void createWorldJoint(Eigen::Vector4d pose) {
    Joint world_joint(object_name_ + "_world");
    world_joint.parent_link_name = "world";
    world_joint.child_link_name = object_name_ + "_base_link";
    world_joint.type = JointType::FIXED;
    world_joint.parent_to_joint_origin_transform =
        Eigen::Isometry3d::Identity();
    world_joint.parent_to_joint_origin_transform.translation() +=
        Eigen::Vector3d(pose(0), pose(1), pose(2));
    world_joint.parent_to_joint_origin_transform.linear() =
        Eigen::AngleAxisd(pose(3), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Joint::Ptr joint = std::make_shared<Joint>(std::move(world_joint));
    setWorldJoint(joint);
    return;
  }

  bool setWorldJoint(Joint::Ptr joint) {
    // Joint::Ptr joint = std::make_shared<Joint>(std::move(world_joint));
    if (joint == nullptr) {
      ROS_WARN("Cannot set null joint as world joint.");
      return false;
    }
    if (world_joint_ != nullptr) {
      ROS_DEBUG("Modifying existing world joint.");
      old_world_joint_ = world_joint_;
      world_joint_ = joint;
      return true;
    }
    if (old_world_joint_ == nullptr) {
      old_world_joint_ = joint;
    }
    world_joint_ = joint;
    return true;
  }

  Joint::Ptr getWorldJoint() { return world_joint_; }

  SceneGraph::ConstPtr getSceneGraph() { return object_scene_graph_; }

 private:
  // Helper functions
  void addToEnvironmentHelper(Environment::Ptr env_,
                              const std::string &link_name,
                              long unsigned int child_joint_num) {
    if (child_joint_num > 0) {
      for (const auto &child_joint :
           object_scene_graph_->getOutboundJoints(link_name)) {
        std::string child_link_name =
            object_scene_graph_->getTargetLink(child_joint->getName())
                ->getName();
        ROS_DEBUG("Adding Link: %s to the environment.",
                  child_link_name.c_str());
        Command::Ptr command = std::make_shared<AddLinkCommand>(
            *object_scene_graph_->getLink(child_link_name), *child_joint);
        if (!env_->applyCommand(command)) {
          ROS_WARN("add to environment helper: add to environment failed");
        };
        addToEnvironmentHelper(
            env_, child_link_name,
            object_scene_graph_->getOutboundJoints(child_link_name).size());
      }
      return;
    } else {
      ROS_DEBUG("Reach terminal Link: %s...", link_name.c_str());
      return;
    }
  }

  void inverseRootTipHelper(SceneGraph::Ptr new_object_scene_graph,
                            const std::string link_name,
                            long unsigned int child_joint_num) {
    ROS_DEBUG("Copying Link:      %s to scene graph.", link_name.c_str());
    if (!new_object_scene_graph->addLink(
            *(object_scene_graph_->getLink(link_name)))) {
      ROS_ERROR("Error when copying Link %s to scene graph.",
                link_name.c_str());
    }

    if (child_joint_num > 0) {
      for (const auto child_joint :
           object_scene_graph_->getOutboundJoints(link_name)) {
        std::string child_link_name =
            object_scene_graph_->getTargetLink(child_joint->getName())
                ->getName();
        ROS_DEBUG(
            "Child link %s has %li child joints", child_link_name.c_str(),
            object_scene_graph_->getOutboundJoints(child_link_name).size());
        inverseRootTipHelper(
            new_object_scene_graph, child_link_name,
            object_scene_graph_->getOutboundJoints(child_link_name).size());
        ROS_DEBUG("Copying Joint:     %s to scene graph.",
                  child_joint->getName().c_str());
        new_object_scene_graph->addJoint(*child_joint);
      }
      return;
    } else {
      ROS_DEBUG("Reach terminal Link: %s...", link_name.c_str());
      return;
    }
  }

 public:
  bool addToEnvironment(Environment::Ptr env_) {
    if (world_joint_ == nullptr) {
      ROS_ERROR("The world joint is not defined for object: %s.",
                object_name_.c_str());
    }

    if (object_scene_graph_ == nullptr) {
      ROS_ERROR("Null object %s!", object_name_.c_str());
    }
    Command::Ptr command = std::make_shared<AddLinkCommand>(
        *(object_scene_graph_->getLink(object_scene_graph_->getRoot())),
        *world_joint_);
    if (!env_->applyCommand(command)) {
      ROS_WARN("add to environment failed");
      return false;
    }
    addToEnvironmentHelper(
        env_, object_scene_graph_->getRoot(),
        object_scene_graph_->getOutboundJoints(object_scene_graph_->getRoot())
            .size());

    return true;
  }

  /**
   * @brief Given object scene graph, set new_root as root node, new_tip as tip
   * node. Inverse the parent-child relationship of joint along the path. Other
   * parent-child relationships remain unchanged.
   * @param new_root: new root to be set in scene graph.
   * @param tip_root: new tip node to be set in scene graph.
   * @return return true if the inverse is successful.
   */
  bool inverseRootTip(const std::string &new_tip, const std::string &new_root) {
    ROS_DEBUG("Inverting %s and %s of object %s", new_tip.c_str(),
              new_root.c_str(), object_name_.c_str());

    SceneGraph::Ptr new_object_scene_graph =
        std::move(object_scene_graph_->clone());

    // Add a dummy world link
    Link world("world");
    object_scene_graph_->addLink(world);
    object_scene_graph_->addJoint(*world_joint_);
    object_scene_graph_->setRoot("world");

    // Get shortest path between root and tip
    ShortestPath stem = object_scene_graph_->getShortestPath(new_tip, new_root);

    Eigen::Isometry3d tip_root_transform;
    tip_root_transform.setIdentity();

    // record previous joint old transform
    Eigen::Isometry3d prev_old_joint_tf;
    prev_old_joint_tf.setIdentity();
    Eigen::Isometry3d prev_old_joint_tf_inv;
    prev_old_joint_tf_inv.setIdentity();

    // stem.second: joint
    // joint contains:
    //    Parent
    //    Child
    //    Parent-child joint transform
    //    Joint type (prismatics, revolute) and axis
    for (auto &it : boost::adaptors::reverse(stem.joints)) {
      ROS_DEBUG("joint name: %s", it.c_str());
      if (object_scene_graph_->getJoint(it)->parent_link_name == "world") {
        // new_tip location in the global world frame
        tip_root_transform =
            world_joint_->parent_to_joint_origin_transform * tip_root_transform;
        // reverse branch on base link;
        if (object_scene_graph_
                ->getOutboundJoints(
                    object_scene_graph_->getJoint(it)->child_link_name)
                .size() > 0) {
          for (auto &it2 : object_scene_graph_->getOutboundJoints(
                   object_scene_graph_->getJoint(it)->child_link_name)) {
            if (std::find(stem.joints.begin(), stem.joints.end(),
                          it2->getName()) == stem.joints.end()) {
              Joint::Ptr outbound_joint = std::make_shared<Joint>(std::move(
                  object_scene_graph_->getJoint(it2->getName())->clone()));
              outbound_joint->parent_to_joint_origin_transform =
                  prev_old_joint_tf.inverse() *
                  outbound_joint->parent_to_joint_origin_transform;
              new_object_scene_graph->removeJoint(it2->getName());
              new_object_scene_graph->addJoint(*outbound_joint);
            }
          }
        }
        continue;
      }
      ROS_DEBUG("Inverting Joint: %s", it.c_str());

      Joint::Ptr current_joint = std::make_shared<Joint>(std::move(
          object_scene_graph_->getJoint(it)
              ->clone()));  // TODO! find ways to modify joint(delete&add?)
      std::string old_parent_link = current_joint->parent_link_name;
      std::string old_child_link = current_joint->child_link_name;
      current_joint->parent_link_name = old_child_link;
      current_joint->child_link_name = old_parent_link;

      tip_root_transform =
          current_joint->parent_to_joint_origin_transform * tip_root_transform;

      if (old_child_link == new_root) {
        current_joint->parent_to_joint_origin_transform.setIdentity();
      } else {
        current_joint->parent_to_joint_origin_transform = prev_old_joint_tf_inv;
      }

      // reverse branch
      if (object_scene_graph_->getOutboundJoints(old_child_link).size() > 0) {
        for (auto &it :
             object_scene_graph_->getOutboundJoints(old_child_link)) {
          if (std::find(stem.joints.begin(), stem.joints.end(),
                        it->getName()) == stem.joints.end()) {
            // std::cout << it->getName()->c_str() << std::endl;
            Joint::Ptr outbound_joint = std::make_shared<Joint>(std::move(
                object_scene_graph_->getJoint(it->getName())->clone()));
            outbound_joint->parent_to_joint_origin_transform =
                prev_old_joint_tf.inverse() *
                outbound_joint->parent_to_joint_origin_transform;
            new_object_scene_graph->removeJoint(it->getName());
            new_object_scene_graph->addJoint(*outbound_joint);
          }
        }
      }

      prev_old_joint_tf =
          object_scene_graph_->getJoint(it)->parent_to_joint_origin_transform;
      prev_old_joint_tf_inv = prev_old_joint_tf.inverse();

      // Get the link that to be modified
      Link::Ptr modified_link =
          new_object_scene_graph->getEditableLink(old_parent_link);
      // std::cout << modified_link->getName() << std::endl;
      // std::cout << prev_old_joint_tf_inv.translation() << std::endl;
      // reverse inertial transform [ToDo]
      if (modified_link->inertial == nullptr) {
        ROS_DEBUG("No inertial info");
      } else {
        modified_link->inertial->origin =
            prev_old_joint_tf_inv * modified_link->inertial->origin;
      }

      // reverse visual transform [ToDo]
      for (auto &it : modified_link->visual) {
        if (it == nullptr) {
          ROS_DEBUG("No visual info");
          continue;
        }
        // std::cout << prev_old_joint_tf_inv.translation() << std::endl;
        // std::cout << prev_old_joint_tf_inv.linear() << std::endl;
        it->origin = prev_old_joint_tf_inv * it->origin;
        ROS_WARN("after inv");
        std::cout << it->origin.translation() << std::endl;
        std::cout << it->origin.linear() << std::endl;
        // it->origin.translation() += Eigen::Vector3d(0, 0, 2);
      }

      // reverse collision transform [ToDo]
      for (auto &it : modified_link->collision) {
        if (it == nullptr) {
          ROS_DEBUG("No collision info");
          continue;
        }
        it->origin = prev_old_joint_tf_inv * it->origin;
      }

      if (current_joint->type == JointType::REVOLUTE ||
          current_joint->type == JointType::PRISMATIC) {
        ROS_DEBUG("Inverting joint limit...");
        if (current_joint->limits != nullptr) {
          double old_upper_limit = current_joint->limits->upper;
          current_joint->limits->upper = -current_joint->limits->lower;
          current_joint->limits->lower = -old_upper_limit;
        }
      }

      // update new scene graph
      new_object_scene_graph->removeJoint(it);
      new_object_scene_graph->addJoint(*current_joint);
    }

    setAttachLocationWorldTransform(tip_root_transform, "attach_" + new_root);

    new_object_scene_graph->setRoot(new_root);
    object_scene_graph_ = new_object_scene_graph;

    Joint new_world_joint(object_name_ + "_world");
    new_world_joint.parent_link_name = "world";
    new_world_joint.child_link_name = new_root;
    new_world_joint.type = JointType::FIXED;
    new_world_joint.parent_to_joint_origin_transform = tip_root_transform;
    Joint::Ptr joint = std::make_shared<Joint>(std::move(new_world_joint));
    setWorldJoint(joint);

    // for (auto jnt : object_scene_graph_->getJoints())
    // {
    //   std::cout << jnt->getName() << ": " << std::endl;
    //   std::cout << jnt->parent_to_joint_origin_transform.translation() <<
    //   std::endl;
    // }
    // for (auto lnk : object_scene_graph_->getLinks())
    // {
    //   std::cout << lnk->getName() << ": " << std::endl;
    //   std::cout << lnk->visual[0]->origin.translation() << std::endl;
    // }
    // std::cout << object_scene_graph_->getRoot().c_str() << std::endl;
    // std::cout << new_object_scene_graph->getRoot().c_str() << std::endl;
    return false;
  }

 protected:
  std::string object_name_; /**< @brief The name of the object. */
  Joint::Ptr
      world_joint_; /**< @brief Define joint between base link and world. */
  Joint::Ptr
      old_world_joint_; /**< @brief Define joint between base link and world. */
  SceneGraph::Ptr
      object_scene_graph_; /**< @brief The scene graph of the object. */
  tesseract_environment::Environment::Ptr
      environment_; /**< @brief The environment of the object. */
  std::unordered_map<std::string, Link::Ptr>
      link_map_; /**< @brief The link map of the object.*/
  std::unordered_map<std::string, Joint::Ptr>
      joint_map_; /**< @brief The joint map of the object. */
  std::unordered_map<std::string, AttachLocation::Ptr>
      attach_locations_;  /**< @brief The attach locations of the
                             object. */
  std::string base_link_; /**< @brief The base link name of the object. */

 private:
  void clear() {
    world_joint_ = nullptr;
    old_world_joint_ = nullptr;
    object_scene_graph_ = nullptr;
    attach_locations_.clear();
    link_map_.clear();
    joint_map_.clear();
    base_link_.clear();
  }
};

}  // namespace vkc
#endif  // VKC_BASIC_OBJECT_H
