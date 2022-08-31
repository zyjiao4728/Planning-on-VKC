#ifndef VKC_CONSTRUCTVKC_H
#define VKC_CONSTRUCTVKC_H

#include <tesseract_common/macros.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/srdf_model.h>

namespace vkc {
class ConstructVKC {
 public:
  using Ptr = std::shared_ptr<ConstructVKC>;
  using ConstPtr = std::shared_ptr<const ConstructVKC>;
  using UPtr = std::unique_ptr<ConstructVKC>;

  // Constructor and destructor
  ConstructVKC();
  ConstructVKC(tesseract_srdf::SRDFModel::Ptr srdf_model,
               tesseract_environment::Environment::Ptr env);
  virtual ~ConstructVKC() = default;

  ConstructVKC::UPtr clone();

  // Load URDF and SRDF file to scene graph
  bool loadURDFtoSceneGraph(
      const std::string &urdf_xml_file, const std::string &srdf_xml_file,
      const tesseract_common::ResourceLocator::ConstPtr &locator);

  // Initialize a tesseract environment that capatible to optimization
  // framework;
  bool initTesseract(std::string monitor_namespace);

  void updateSceneGraph(tesseract_scene_graph::SceneGraph::Ptr sg);

  // Return private members
  tesseract_environment::Environment::Ptr getTesseract();
  tesseract_scene_graph::SceneGraph::Ptr getSceneGraph();
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr getMonitor();
  tesseract_srdf::SRDFModel::ConstPtr getSRDFModel() const;

  // tesseract_environment::Environment::Ptr getEnvironment();
  // tesseract_environment::Environment::Ptr getTesseract();

  // // Break scene graph and reconnect
  // bool construct(const std::string end_effector_link_name, const
  // AttachLocation attach_location);

 private:
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_;
  tesseract_srdf::SRDFModel::Ptr srdf_model_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_environment::Environment::Ptr env_;

  // Copy a link and all its child links/joints to a new scene graph
  // tesseract_scene_graph::SceneGraph::Ptr breakSceneGraphatJoint(const
  // std::string& scene_graph_name,
  //                                                               const
  //                                                               std::string&
  //                                                               link_name);

  // void breakSceneGraphatJointHelper(tesseract_scene_graph::SceneGraph::Ptr
  // scene_graph_branch,
  //                                   const std::string link_name, long
  //                                   unsigned int child_joint_num);

  // const std::string constructHelper_FindLinkToWorld(const std::string
  // leaf_link_name);

  // void constructHelper_InvConnect(tesseract_scene_graph::SceneGraph::Ptr
  // scene_graph_branch,
  //                                 const std::string end_effector_link);

  void clear();

  void clearTesseract();
};

}  // namespace vkc

#endif  // VKC_CONSTRUCTVKC_H