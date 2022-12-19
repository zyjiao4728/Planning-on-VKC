#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
// #include <tesseract_environment/kdl/kdl_env.h>
#include <fmt/format.h>
#include <ros/ros.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_factories.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_srdf/utils.h>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <vkc/construct_vkc.h>

namespace vkc {
ConstructVKC::ConstructVKC() {
  ROS_INFO("Virtual Kinematic Chain constructor initialized...");
  clear();
}
ConstructVKC::ConstructVKC(
    tesseract_srdf::SRDFModel::Ptr srdf_model,
    tesseract_environment::Environment::Ptr env,
    tesseract_environment::Environment::Ptr env_non_inverse) {
  ROS_INFO("Virtual Kinematic Chain constructor initialized...");
  clear();
  srdf_model_ = srdf_model;
  env_ = env;
  env_non_inverse_ = env_non_inverse;
  scene_graph_ = std::move(env->getSceneGraph()->clone());
}

ConstructVKC::UPtr ConstructVKC::clone() {
  ROS_WARN("srdf model is reused, may cause problem...");
  auto cloned_vkc =
      std::make_unique<ConstructVKC>(srdf_model_, std::move(env_->clone()),
                                     std::move(env_non_inverse_->clone()));
  return cloned_vkc;
}

bool ConstructVKC::loadURDFtoSceneGraph(
    const std::string &urdf_xml_file, const std::string &srdf_xml_file,
    const tesseract_common::ResourceLocator::ConstPtr &locator) {
  clear();

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph =
      tesseract_urdf::parseURDFString(urdf_xml_file, *locator);
  if (scene_graph == nullptr) {
    ROS_ERROR("Failed to parse URDF file: %s.", urdf_xml_file.c_str());
    return false;
  }
  ROS_INFO("The root of scene graph is: %s", scene_graph->getRoot().c_str());

  if (srdf_xml_file == "NULL") {
    ROS_WARN("Empty SRDF file, ignore.");
    scene_graph_ = scene_graph;
    srdf_model_ = std::make_shared<tesseract_srdf::SRDFModel>();
    return true;
  }

  // Parse srdf string into SRDF Model
  tesseract_srdf::SRDFModel::Ptr srdf =
      std::make_shared<tesseract_srdf::SRDFModel>();
  // throws if error occurs, no need for the if.
  srdf->initString(*scene_graph, srdf_xml_file, *locator);
  // if (!srdf->initString(*scene_graph, srdf_xml_file))
  // {
  //   ROS_ERROR("Failed to parse SRDF file: %s.", srdf_xml_file.c_str());
  //   return false;
  // }

  // Add allowed collision matrix to scene graph
  tesseract_srdf::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // addVirtualMobileBase(scene_graph, srdf);

  scene_graph_ = scene_graph;
  srdf_model_ = srdf;
  return true;
}

void ConstructVKC::updateSceneGraph(tesseract_scene_graph::SceneGraph::Ptr sg) {
  scene_graph_ = sg;
}

bool ConstructVKC::initTesseract(std::string monitor_namespace) {
  if (monitor_ != nullptr) {
    return true;
  }
  if (scene_graph_ == nullptr || srdf_model_ == nullptr) {
    ROS_ERROR("Null scene graph or srdf model!");
  }
  env_ = std::make_shared<tesseract_environment::Environment>();
  if (!env_->init(*scene_graph_, srdf_model_)) {
    ROS_INFO("Failed to initialize tesseract environment");
    throw std::runtime_error("tesseract init failed");
  }
  env_non_inverse_ = std::make_shared<tesseract_environment::Environment>();
  env_non_inverse_->init(*scene_graph_, srdf_model_);
  monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(
      env_, monitor_namespace);
  monitor_->startPublishingEnvironment();
  // monitor_->startStateMonitor();  // DO NOT ENABLE THIS, WILL CAUSE SERIOUS
  // PROBLEM(RESET ENVIRONMENT STATE EVERY TIME)
  return true;
}

tesseract_environment::Environment::Ptr ConstructVKC::getTesseract() {
  if (env_ == nullptr) {
    CONSOLE_BRIDGE_logError("Null tesseract!");
  }
  return env_;
}

tesseract_environment::Environment::Ptr ConstructVKC::getTesseractNonInverse() {
  if (env_non_inverse_ == nullptr) {
    CONSOLE_BRIDGE_logError("Null non inverse tesseract!");
  }
  return env_non_inverse_;
}

tesseract_scene_graph::SceneGraph::Ptr ConstructVKC::getSceneGraph() {
  return scene_graph_;
}

tesseract_monitoring::ROSEnvironmentMonitor::Ptr ConstructVKC::getMonitor() {
  return monitor_;
}

tesseract_srdf::SRDFModel::ConstPtr ConstructVKC::getSRDFModel() const {
  return srdf_model_;
}

// tesseract_environment::Environment::Ptr ConstructVKC::getEnvironment()
// {
//   return env_;
// }

// tesseract_environment::Environment::Ptr ConstructVKC::getTesseract()
// {
//   return monitor_->getEnvironment();
// }

void ConstructVKC::clear() {
  srdf_model_ = nullptr;
  scene_graph_ = nullptr;
  env_ = nullptr;
  env_non_inverse_ = nullptr;
  monitor_ = nullptr;
}

void ConstructVKC::clearTesseract() {
  monitor_ = nullptr;
  env_ = nullptr;
  env_non_inverse_ = nullptr;
}

}  // namespace vkc