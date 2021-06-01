#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_urdf/urdf_parser.h>
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

bool ConstructVKC::loadURDFtoSceneGraph(const std::string& urdf_xml_file, const std::string& srdf_xml_file,
                                        tesseract_scene_graph::ResourceLocator::Ptr& locator)
{
  clear();

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = tesseract_urdf::parseURDFString(urdf_xml_file, locator);
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
    srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>();
    return true;
  }

  // Parse srdf string into SRDF Model
  tesseract_scene_graph::SRDFModel::Ptr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  if (!srdf->initString(*scene_graph, srdf_xml_file))
  {
    ROS_ERROR("Failed to parse SRDF file: %s.", srdf_xml_file.c_str());
    return false;
  }

  // Add allowed collision matrix to scene graph
  tesseract_scene_graph::processSRDFAllowedCollisions(*scene_graph, *srdf);

  // addVirtualMobileBase(scene_graph, srdf);

  scene_graph_ = scene_graph;
  srdf_model_ = srdf;
  return true;
}

void ConstructVKC::updateSceneGraph(tesseract_scene_graph::SceneGraph::Ptr sg)
{
  scene_graph_ = sg;
}

bool ConstructVKC::initTesseract()
{
  if (tesseract_ == nullptr)
  {
    ROS_WARN("Null pointer to tesseract!");
    tesseract_ = std::make_shared<tesseract::Tesseract>();
  }
  if (scene_graph_ == nullptr || srdf_model_ == nullptr)
  {
    ROS_ERROR("Null scene graph or srdf model!");
  }
  if (!tesseract_->init(scene_graph_, srdf_model_))
  {
    ROS_INFO("Failed to initialize tesseract environment");
    return false;
  }
  return true;
}

tesseract::Tesseract::Ptr ConstructVKC::getTesseract()
{
  if (tesseract_ == nullptr)
  {
    ROS_ERROR("Null tesseract!");
  }
  return tesseract_;
}

tesseract_scene_graph::SceneGraph::Ptr ConstructVKC::getSceneGraph()
{
  return scene_graph_;
}

tesseract_scene_graph::SRDFModel::Ptr ConstructVKC::getSRDFModel()
{
  return srdf_model_;
}

const tesseract_environment::Environment::Ptr& ConstructVKC::getTesseractEnvironment()
{
  return tesseract_->getEnvironment();
}

const tesseract_scene_graph::SceneGraph::ConstPtr& ConstructVKC::getTesseractSceneGraph()
{
  return tesseract_->getEnvironment()->getSceneGraph();
}

/*
tesseract_scene_graph::SceneGraph::Ptr ConstructVKC::breakSceneGraphatJoint(const std::string& scene_graph_name,
                                                                            const std::string& link_name)
{
  ROS_INFO("Breaking scene graph of link %s", link_name.c_str());
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_branch(new tesseract_scene_graph::SceneGraph());
  breakSceneGraphatJointHelper(scene_graph_branch, link_name, scene_graph_->getInboundJoints(link_name).size());
  scene_graph_branch->setName(scene_graph_name);
  scene_graph_branch->setRoot(link_name);
  if (!scene_graph_branch->isTree())
  {
    ROS_ERROR("Scene graph branch is not a tree!");
  }
  return scene_graph_branch;
}

void ConstructVKC::breakSceneGraphatJointHelper(tesseract_scene_graph::SceneGraph::Ptr scene_graph_branch,
                                                const std::string link_name, long unsigned int child_joint_num)
{
  ROS_INFO("Adding Link:      %s to scene graph.", link_name.c_str());
  if (!scene_graph_branch->addLink(*(scene_graph_->getLink(link_name))))
  {
    ROS_ERROR("Error when adding Link %s to scene graph.", link_name.c_str());
  }

  if (child_joint_num > 0)
  {
    for (const auto child_joint : scene_graph_->getOutboundJoints(link_name))
    {
      std::string child_link_name = scene_graph_->getTargetLink(child_joint->getName())->getName();
      ROS_WARN("Child link %s has %i child joints", child_link_name.c_str(),
               scene_graph_->getOutboundJoints(child_link_name).size());
      breakSceneGraphatJointHelper(scene_graph_branch, child_link_name,
                                   scene_graph_->getOutboundJoints(child_link_name).size());

      ROS_INFO("Adding Joint:     %s to scene graph.", child_joint->getName().c_str());
      scene_graph_branch->addJoint(*child_joint);
    }
    return;
  }
  else
  {
    ROS_WARN("Reach terminal Link: %s...", link_name.c_str());
    return;
  }
}

bool ConstructVKC::construct(const std::string end_effector_link_name,
                             const ConstructVKC::AttachLocation attach_location)
{
  clearTesseract();

  if (!scene_graph_->isTree())
  {
    ROS_ERROR("Scene graph is not a tree!");
  }

  tesseract_scene_graph::SceneGraph::Ptr scene_graph_branch =
      breakSceneGraphatJoint("branch", constructHelper_FindLinkToWorld(attach_location.link_name_));

  constructHelper_InvConnect(scene_graph_branch, end_effector_link_name);

  return true;
}

const std::string ConstructVKC::constructHelper_FindLinkToWorld(const std::string leaf_link_name)
{
  std::string world_object_link = "";
  if (scene_graph_->getRoot() == leaf_link_name)
  {
    ROS_ERROR("Cannot find world link, or the leaf link is not connected to the world!");
    return "";
  }
  if (scene_graph_->getSourceLink((scene_graph_->getInboundJoints(leaf_link_name)[0]->getName()))->getName() == "worl"
                                                                                                                "d")
  {
    return leaf_link_name;
  }
  else
  {
    world_object_link = constructHelper_FindLinkToWorld(
        scene_graph_->getSourceLink((scene_graph_->getInboundJoints(leaf_link_name)[0]->getName()))->getName());
  }
  return world_object_link;
}

void ConstructVKC::constructHelper_InvConnect(tesseract_scene_graph::SceneGraph::Ptr scene_graph_branch,
                                              const std::string end_effector_link)
{
  // Remove branch from main scene graph
  if (scene_graph_->getLink(scene_graph_branch->getRoot()) == nullptr)
  {
    ROS_ERROR("Tried to remove link (%s) that does not exist", scene_graph_branch->getRoot().c_str());
    return;
  }

  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
      scene_graph_->getInboundJoints(scene_graph_branch->getRoot());
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<std::string> child_link_names = scene_graph_->getLinkChildrenNames(scene_graph_branch->getRoot());

  scene_graph_->removeLink(scene_graph_branch->getRoot());

  for (const auto& link_name : child_link_names)
  {
    scene_graph_->removeLink(link_name);
  }

  if (!environment_->init(scene_graph_))
  {
    ROS_ERROR("Failed to initialize environment.");
    return;
  }

  environment_const_ = environment_;
  registerDefaultContactManagers();
  registerDefaultFwdKinSolvers();
  registerDefaultInvKinSolvers();
  initialized_ = true;

  initTesseract();
  return;
}
*/

void ConstructVKC::clear()
{
  srdf_model_ = nullptr;
  scene_graph_ = nullptr;
  tesseract_ = nullptr;
}

void ConstructVKC::clearTesseract()
{
  tesseract_ = nullptr;
}

}  // namespace vkc