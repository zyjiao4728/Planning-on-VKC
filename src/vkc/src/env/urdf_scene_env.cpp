#include <vkc/env/urdf_scene_env.h>

using namespace std;
using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

#define DOUT(x) (std::cout << "[DEBUG]: " << x << std::endl)

// URDF and SRDF file describes environment and robot
const std::string INTERACTIVE_SCENE_DESC_PARAM = "interactive_description";
const std::string ENV_DESCRIPTION_PARAM = "env_description";
const std::string ENV_SEMANTIC_PARAM = "env_description_semantic";

// Link name defined as end effector
const std::string END_EFFECTOR_LINK = "end_effector_link";

// RVIZ service
const std::string GET_ENVIRONMENT_CHANGES_SERVICE =
    "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

namespace vkc {
UrdfSceneEnv::UrdfSceneEnv(ros::NodeHandle nh, bool plotting, bool rviz,
                           int steps)
    : VKCEnvBasic(nh, plotting, rviz), steps_(steps) {
  // Set Log Level
  util::gLogLevel = util::LevelInfo;

  loadRobotModel(ENV_DESCRIPTION_PARAM, ENV_SEMANTIC_PARAM, END_EFFECTOR_LINK);

  initTesseractConfig(MODIFY_ENVIRONMENT_SERVICE,
                      GET_ENVIRONMENT_CHANGES_SERVICE);

  // set robot initial pose in scene graph
  setHomePose();

  ROS_INFO("Sucessfully load the robot model, now creating environment...");

  createEnvironment();

  ROS_INFO(
      "Sucessfully create the environment, now creating optimization "
      "problem...");
}

UrdfSceneEnv::UrdfSceneEnv(ros::NodeHandle nh, bool plotting, bool rviz,
                           int steps, const AttachObjectInfos &attaches,
                           const InverseChainsInfos &inverse_chains)
    : VKCEnvBasic(nh, plotting, rviz),
      steps_(steps),
      attaches_(attaches),
      inverse_chains_(inverse_chains) {
  // Set Log Level
  util::gLogLevel = util::LevelInfo;
  loadRobotModel(ENV_DESCRIPTION_PARAM, ENV_SEMANTIC_PARAM, END_EFFECTOR_LINK);

  initTesseractConfig(MODIFY_ENVIRONMENT_SERVICE,
                      GET_ENVIRONMENT_CHANGES_SERVICE);

  // set robot initial pose in scene graph
  setHomePose();
  ROS_INFO("Sucessfully load the robot model, now creating environment...");

  createEnvironment();
  ROS_INFO(
      "sucessfully create the environment, now creating optimization "
      "problem...");
}

bool UrdfSceneEnv::createEnvironment() {
  DOUT("Before Create Environment SceneGraph:");
  printSceneGraph(tesseract_->getTesseract()->getSceneGraph());

  configAttachLocations_(attaches_);
  configInverseChains_(inverse_chains_);

  DOUT("After Create Environment SceneGraph:");
  printSceneGraph(tesseract_->getTesseract()->getSceneGraph());

  DOUT("Before RVIZ Changed");

  for (auto &it : attach_locations_) {
    it.second->world_joint_origin_transform =
        tesseract_->getTesseract()->getEnvironment()->getLinkTransform(
            it.second->link_name_) *
        it.second->local_joint_origin_transform;
    std::cout << "translation of attachment " << it.second->name_
              << " in world frame: " << std::endl;
    std::cout << it.second->world_joint_origin_transform.translation()
              << std::endl;
    std::cout << "rotation matrix of attachment " << it.second->name_
              << " in world frame: " << std::endl;
    std::cout << it.second->world_joint_origin_transform.rotation()
              << std::endl;
  }

  if (rviz_) {
    // Now update rviz environment
    if (!sendRvizChanges(n_past_revisions_, tesseract_)) return false;
  }

  DOUT("RVIZ Changed");

  return true;
}

/**
 * Load SceneGraph from ROS-published URDF description
 *
 * @param scene_description (std::string) the description param ROS published
 * @return (tesseract_scene_graph::SceneGraph::Ptr): A SceneGraph pointer that
 * decodes the parsed scene
 */
tesseract_scene_graph::SceneGraph::Ptr UrdfSceneEnv::loadSceneGraphFromURDF_(
    const std::string &scene_description) {
  // Initial setup, load xml directory defined in launch
  std::string scene_urdf_xml_string;

  nh_.getParam(scene_description, scene_urdf_xml_string);

  DOUT("Load URDF from: " << scene_description);
  ResourceLocator::Ptr locator =
      std::make_shared<tesseract_rosutils::ROSResourceLocator>();

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr sg =
      tesseract_urdf::parseURDFString(scene_urdf_xml_string, locator);
  if (sg == nullptr) {
    DOUT("Fail to parse SceneGraph: " << scene_description);
    exit(1);
  }

  return sg;
}

bool UrdfSceneEnv::reInit() {
  // TODO
  attached_links_.clear();
  configAttachLocations_(attaches_);

  VKCEnvBasic::reInit();
}
/*****************************************************************8
 * Configurate AttachLocations in the URDF scene
 *
 * Add AttachLocations in to the UrdfSceneEnv
 */
void UrdfSceneEnv::configAttachLocations_(
    const std::vector<AttachObjectInfo> &attaches) {
  for (const auto &attach : attaches) {
    ROS_INFO(
        "[%s]add attach object, attach name: %s, attach_link: %s, fixed base: "
        "%s",
        __func__, attach.attach_name.c_str(), attach.attach_link.c_str(),
        attach.base_link.c_str(), (attach.fixed_base ? "yes" : "no"));

    newAttachLocation_(attach.attach_name, attach.attach_link, attach.base_link,
                       attach.local_joint_tf_trans, attach.local_joint_tf_quat,
                       attach.fixed_base);
  }
}

void UrdfSceneEnv::newAttachLocation_(std::string attach_name,
                                      std::string attach_object_link,
                                      std::string object_baselink,
                                      std::vector<double> local_joint_tf_trans,
                                      std::vector<double> local_joint_tf_quat,
                                      bool fixed_base) {
  // define AttachLocation object
  vkc::BaseObject::AttachLocation attach_location(attach_name,
                                                  attach_object_link);
  attach_location.base_link_ = object_baselink;
  attach_location.local_joint_origin_transform.translation() +=
      Eigen::Vector3d(local_joint_tf_trans[0], local_joint_tf_trans[1],
                      local_joint_tf_trans[2]);
  attach_location.local_joint_origin_transform.linear() =
      Eigen::Quaterniond(local_joint_tf_quat[0], local_joint_tf_quat[1],
                         local_joint_tf_quat[2], local_joint_tf_quat[3])
          .matrix();
  attach_location.fixed_base = fixed_base;

  // Define connection joint
  attach_location.connection.type = tesseract_scene_graph::JointType::FIXED;
  attach_location.connection.child_link_name = attach_object_link;
  attach_location.connection.parent_link_name = "NULL";

  // add AttachLocation object into the Env
  // The name ID of the AttachLocation will be AttachLocation.name_
  AttachLocation::Ptr attach_location_ptr =
      std::make_shared<AttachLocation>(std::move(attach_location));

  addAttachLocation(attach_location_ptr);
}

// /*******************************************************************
//  * Configurate Inserve Kinematics Chains
//  *
//  * Inserve the chain of the manipulated object
//  ******************************************************************/
TesseractSceneGraphPtr UrdfSceneEnv::configInverseChains_(
    const std::vector<InverseChainsInfo> &inverse_chains) {
  SceneGraph::Ptr iscene_sg =
      loadSceneGraphFromURDF_(INTERACTIVE_SCENE_DESC_PARAM);

  for (const auto &chain : inverse_chains) {
    ROS_INFO(
        "[%s][Debug]inversing chain, origin root: %s, destination root: %s",
        __func__, chain.ori_root_link.c_str(), chain.dst_root_link.c_str());

    iscene_sg =
        inverseEnvChain_(iscene_sg, chain.ori_root_link, chain.dst_root_link);
    ROS_INFO("root has changed: %s",
             iscene_sg->getLinkChildrenNames(chain.dst_root_link).size() > 0
                 ? "yes"
                 : "no");
  }

  updateInvertedEnv_(iscene_sg);

  return iscene_sg;
}
/*****************************************************************
 * Inserve a kinematic chain inside the Environment
 *
 * After inserving the kinematic chain, the original source node
 * will be the new destination node, whereas the original
 * destination node will be the new source node.
 *
 * @param sg (tesseract_scene_graph::SceneGraph::Ptr) scene graph pointer
 * @param src (std::string): the source node of the original chain
 * @param dst (std::string): the destination node of the original chain
 * @return (SceneGraph::Ptr): inverted SceneGraph pointer
 */
SceneGraph::Ptr UrdfSceneEnv::inverseEnvChain_(SceneGraph::ConstPtr sg,
                                               std::string src,
                                               std::string dst) {
  // #todo
  DOUT("inverse chain: `" << src << "` -----> `" << dst << "`");

  SceneGraph::Ptr inv_sg = inverseEnvChainHelper_(sg, src, dst);

  return inv_sg;
}

/*****************************************************************8
 * Inserve a kinematic chain inside the Environment
 *
 * After inserving the kinematic chain, the original source node
 * will be the new destination node, whereas the original
 * destination node will be the new source node.
 *
 * @param src (std::string): the source node of the original chain
 * @param dst (std::string): the destination node of the original chain
 */
tesseract_scene_graph::SceneGraph::Ptr UrdfSceneEnv::inverseEnvChainHelper_(
    SceneGraph::ConstPtr sg, std::string src, std::string dst) {
  DOUT("[" << __func__ << "]inverting link `" << src << "` and `" << dst
           << "`");

  SceneGraph::Ptr tmp_sg = deepcopySceneGraph(sg);
  std::string src_parent_name = getLinkParentName_(sg, src);
  Joint::Ptr src_parent_joint = getLinkParentJoint_(sg, src);

  // find a path from `src` link to `dst` link
  // path.second: return JSON-type joint_name list
  vector<string> path = findPath(sg, src_parent_name, dst);

  // new_tip location in the global world frame
  Eigen::Isometry3d tip_root_transform;
  tip_root_transform.setIdentity();

  // record previous joint old transform
  Eigen::Isometry3d prev_old_joint_tf;
  prev_old_joint_tf.setIdentity();
  Eigen::Isometry3d prev_old_joint_tf_inv;
  prev_old_joint_tf_inv.setIdentity();

  // each iteractor `it` is a string that describes a joint name
  reverse(path.begin(), path.end());

  for (auto &it : path) {
    Joint::Ptr sg_joint = sg->getJoint(it);
    if (sg_joint->parent_link_name == src_parent_name) {
      // new_tip location in the global world frame
      tip_root_transform = src_parent_joint->parent_to_joint_origin_transform *
                           tip_root_transform;
      ROS_INFO("[%s]link_name: %s, outbound joints:", __func__,
               sg_joint->child_link_name.c_str());
      for (auto &child_it : sg->getOutboundJoints(sg_joint->child_link_name)) {
        ROS_INFO("\t\t%s", child_it->getName().c_str());
        if (std::find(path.begin(), path.end(), child_it->getName()) ==
            path.end()) {
          Joint::Ptr outbound_joint = sg->getJoint(child_it->getName());
          outbound_joint->parent_to_joint_origin_transform =
              prev_old_joint_tf.inverse() *
              outbound_joint->parent_to_joint_origin_transform;
          tmp_sg->removeJoint(child_it->getName());
          tmp_sg->addJoint(*outbound_joint);
        }
      }
      // jump to next iteration in the path
      continue;
    }

    Joint::Ptr new_joint = sg_joint;
    new_joint->parent_link_name = sg_joint->child_link_name;
    new_joint->child_link_name = sg_joint->parent_link_name;

    tip_root_transform =
        new_joint->parent_to_joint_origin_transform * tip_root_transform;

    if (sg_joint->child_link_name == dst) {
      new_joint->parent_to_joint_origin_transform.setIdentity();
    } else {
      new_joint->parent_to_joint_origin_transform = prev_old_joint_tf_inv;
    }

    // update transform of child branch
    for (auto &child_it : sg->getOutboundJoints(sg_joint->child_link_name)) {
      if (std::find(path.begin(), path.end(), child_it->getName()) ==
          path.end()) {
        Joint::Ptr outbound_joint =
            std::make_shared<Joint>(*sg->getJoint(child_it->getName()));

        outbound_joint->parent_to_joint_origin_transform =
            prev_old_joint_tf.inverse() *
            outbound_joint->parent_to_joint_origin_transform;

        tmp_sg->removeJoint(child_it->getName());
        tmp_sg->addJoint(*outbound_joint);
      }
    }

    prev_old_joint_tf = sg->getJoint(it)->parent_to_joint_origin_transform;
    prev_old_joint_tf_inv = prev_old_joint_tf.inverse();

    // Get the link that to be modified
    Link::Ptr modified_link =
        tmp_sg->getEditableLink(sg_joint->parent_link_name);

    // reverse inertial transform
    if (nullptr == modified_link || modified_link->inertial == nullptr) {
      ROS_DEBUG("No inertial info");
    } else {
      modified_link->inertial->origin =
          prev_old_joint_tf_inv * modified_link->inertial->origin;
    }

    // reverse visual transform
    for (auto &element : modified_link->visual) {
      if (element == nullptr) {
        ROS_DEBUG("No visual info");
        continue;
      }
      element->origin = prev_old_joint_tf_inv * element->origin;
    }

    // reverse collision transform
    for (auto &element : modified_link->collision) {
      if (element == nullptr) {
        ROS_DEBUG("No collision info");
        continue;
      }
      element->origin = prev_old_joint_tf_inv * element->origin;
    }

    if (new_joint->type == JointType::REVOLUTE ||
        new_joint->type == JointType::PRISMATIC) {
      ROS_DEBUG("Inverting joint limit...");
      if (new_joint->limits != nullptr) {
        double old_upper_limit = new_joint->limits->upper;
        new_joint->limits->upper = -new_joint->limits->lower;
        new_joint->limits->lower = -old_upper_limit;
      }
    }

    // update new scene graph
    tmp_sg->removeJoint(it);
    tmp_sg->addJoint(*new_joint);
  }

  Joint new_root_parent_joint(src_parent_joint->getName());
  new_root_parent_joint.parent_link_name = src_parent_name;
  new_root_parent_joint.child_link_name = dst;
  new_root_parent_joint.type = JointType::FIXED;
  new_root_parent_joint.parent_to_joint_origin_transform = tip_root_transform;

  tmp_sg->removeJoint(src_parent_joint->getName());
  tmp_sg->addJoint(new_root_parent_joint);

  return tmp_sg;
}

/*****************************************************************8
 * Update inverted SceneGraph to environment
 */
void UrdfSceneEnv::updateInvertedEnv_(SceneGraph::ConstPtr sg) {
  string root_name = sg->getRoot();

  for (auto it : sg->getOutboundJoints(root_name)) {
    addToEnv_(sg, it->child_link_name);
  }
}

void UrdfSceneEnv::addToEnv_(SceneGraph::ConstPtr sg, string root_name) {
  Joint::ConstPtr root_parent_joint = getLinkParentJoint_(sg, root_name);
  Link::ConstPtr link = sg->getLink(root_name);

  DOUT("add link: " << link->getName()
                    << ", Joint: " << root_parent_joint->getName());
  tesseract_->getTesseractEnvironment()->addLink(*link, *root_parent_joint);
  plot_tesseract_->getTesseractEnvironment()->addLink(*link,
                                                      *root_parent_joint);

  std::vector<Joint::ConstPtr> out_bound_joints =
      sg->getOutboundJoints(root_name);
  for (auto it : out_bound_joints) {
    addToEnv_(sg, it->child_link_name);
  }
}

string UrdfSceneEnv::getLinkParentName_(SceneGraph::ConstPtr sg,
                                        string link_name) {
  auto inbound_joints = sg->getInboundJoints(link_name);
  if (0 != inbound_joints.size()) {
    return inbound_joints[0]->parent_link_name;
  } else {
    std::cout << "[" << __func__ << "]no link parent is found!" << std::endl;
    return string();
  }
}

Joint::Ptr UrdfSceneEnv::getLinkParentJoint_(SceneGraph::ConstPtr sg,
                                             string link_name) {
  auto inbound_joints = sg->getInboundJoints(link_name);

  return std::make_shared<Joint>(*inbound_joints[0]);
}

/*******************************************************************
 * Configurate Inserve Kinematics Chains
 *
 * Inserve the chain of the manipulated object
 ******************************************************************/
void printSceneGraph(const SceneGraph::ConstPtr &sg) {
  cout << "- " << sg->getRoot() << endl;
  printSceneGraphHelper(sg, sg->getRoot(), 1);
}

void printSceneGraphHelper(const SceneGraph::ConstPtr &sg,
                           const string &root_name, int level) {
  for (auto &it : sg->getOutboundJoints(root_name)) {
    for (int i = 0; i < level; i++) cout << "  ";
    cout << "- " << it->child_link_name << " | (Joint: " << it->getName() << ")"
         << endl;
    printSceneGraphHelper(sg, it->child_link_name, level + 1);
  }
}

SceneGraph::Ptr deepcopySceneGraph(SceneGraph::ConstPtr sg) {
  SceneGraph::Ptr sg_copied = std::make_shared<SceneGraph>();

  deepcopySceneGraphHelper(sg_copied, sg, sg->getRoot());
  sg_copied->setRoot(sg->getRoot());
  sg_copied->setName(sg->getName());

  return sg_copied;
}

void deepcopySceneGraphHelper(SceneGraph::Ptr dst_sg,
                              SceneGraph::ConstPtr src_sg, string root_name) {
  if (!dst_sg->addLink(*(src_sg->getLink(root_name)))) {
    DOUT("Error when copying Link " << root_name << " to scene graph.");
  }

  for (const auto &child_joint : src_sg->getOutboundJoints(root_name)) {
    deepcopySceneGraphHelper(dst_sg, src_sg, child_joint->child_link_name);
    dst_sg->addJoint(*child_joint);
  }
}

vector<string> findPath(SceneGraph::ConstPtr sg, const string &src,
                        const string &dst) {
  vector<string> path;

  findPathHelper(sg, src, dst, path);

  reverse(path.begin(), path.end());
  return path;
}

bool findPathHelper(SceneGraph::ConstPtr sg, const string &src,
                    const string &dst, vector<string> &path) {
  if (src == dst) return true;

  for (auto &it : sg->getOutboundJoints(src)) {
    if (findPathHelper(sg, it->child_link_name, dst, path)) {
      path.push_back(it->getName());
      return true;
    }
  }

  return false;
}

}  // namespace vkc

// const tesseract_environment::Environment::Ptr&
