#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
// #include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_factories.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/utils.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <vkc/construct_vkc.h>


namespace vkc
{
  ConstructVKC::ConstructVKC()
  {
    ROS_INFO("Virtual Kinematic Chain constructor initialized...");
    clear();
  }

  bool ConstructVKC::loadURDFtoSceneGraph(const std::string &urdf_xml_file, const std::string &srdf_xml_file,
                                          const tesseract_common::ResourceLocator::ConstPtr &locator)
  {
    clear();

    // Parse urdf string into Scene Graph
    tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFString(urdf_xml_file, *locator);
    if (scene_graph == nullptr)
    {
      ROS_ERROR("Failed to parse URDF file: %s.", urdf_xml_file.c_str());
      return false;
    }
    ROS_INFO("The root of scene graph is: %s", scene_graph->getRoot().c_str());

    if (srdf_xml_file == "NULL")
    {
      ROS_WARN("Empty SRDF file, ignore.");
      scene_graph_ = scene_graph;
      srdf_model_ = std::make_shared<tesseract_srdf::SRDFModel>();
      return true;
    }

    // Parse srdf string into SRDF Model
    tesseract_srdf::SRDFModel::Ptr srdf = std::make_shared<tesseract_srdf::SRDFModel>();
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

  void ConstructVKC::updateSceneGraph(tesseract_scene_graph::SceneGraph::Ptr sg)
  {
    scene_graph_ = sg;
  }

  bool ConstructVKC::initTesseract(std::string monitor_namespace)
  {
    if (monitor_ != nullptr)
    {
      return true;
    }
    if (scene_graph_ == nullptr || srdf_model_ == nullptr)
    {
      ROS_ERROR("Null scene graph or srdf model!");
    }
    env_ = std::make_shared<tesseract_environment::Environment>();
    if (!env_->init(*scene_graph_, srdf_model_))
    {
      ROS_INFO("Failed to initialize tesseract environment");
      return false;
    }
    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(env_, monitor_namespace);
    monitor_->startPublishingEnvironment();
    return true;
  }

  tesseract_environment::Environment::Ptr ConstructVKC::getTesseract()
  {
    if (env_ == nullptr)
    {
      ROS_ERROR("Null tesseract!");
    }
    return env_;
  }

  tesseract_scene_graph::SceneGraph::Ptr ConstructVKC::getSceneGraph()
  {
    return scene_graph_;
  }

  tesseract_srdf::SRDFModel::Ptr ConstructVKC::getSRDFModel()
  {
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

  void ConstructVKC::clear()
  {
    srdf_model_ = nullptr;
    scene_graph_ = nullptr;
    monitor_ = nullptr;
  }

  void ConstructVKC::clearTesseract()
  {
    monitor_ = nullptr;
    env_ = nullptr;
  }

} // namespace vkc