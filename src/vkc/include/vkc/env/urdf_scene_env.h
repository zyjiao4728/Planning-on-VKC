#ifndef VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H
#define VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H


#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_urdf/urdf_parser.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <vkc/object/objects.h>
#include <vkc/env/vkc_env_basic.h>
#include <string>
#include <vector>
#include <algorithm>


namespace vkc
{
using TesseractSceneGraphPtr = tesseract_scene_graph::SceneGraph::Ptr;
using TesseractSceneGraphConstPtr = tesseract_scene_graph::SceneGraph::ConstPtr;

std::vector<std::string> findPath(tesseract_scene_graph::SceneGraph::ConstPtr sg, const std::string &src, const std::string &dst);
bool findPathHelper(tesseract_scene_graph::SceneGraph::ConstPtr sg, const std::string &src, 
  const std::string &dst, std::vector<std::string> &path);

void printSceneGraph(const tesseract_scene_graph::SceneGraph::ConstPtr &sg);
void printSceneGraphHelper(const tesseract_scene_graph::SceneGraph::ConstPtr &sg, const std::string &root_name, int level);

tesseract_scene_graph::SceneGraph::Ptr deepcopySceneGraph(tesseract_scene_graph::SceneGraph::ConstPtr sg);
void deepcopySceneGraphHelper(tesseract_scene_graph::SceneGraph::Ptr dst_sg, tesseract_scene_graph::SceneGraph::ConstPtr src_sg, std::string root_name);


/**
 * @brief Load dual ur5e husky for door opening task.
 */
class UrdfSceneEnv : public VKCEnvBasic
{
public: 
  struct AttachObjectInfo
  {
    std::string attach_name;
    std::string attach_link;
    std::string base_link;
    std::vector<double> local_joint_tf_trans;
    std::vector<double> local_joint_tf_quat;
    bool fixed_base;
  };

  struct InverseChainsInfo
  {
    std::string ori_root_link;
    std::string dst_root_link;
  };

  using AttachObjectInfos = std::vector<AttachObjectInfo>;
  using InverseChainsInfos = std::vector<InverseChainsInfo>;

  UrdfSceneEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps);

  UrdfSceneEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps,
               const AttachObjectInfos &attaches,
               const InverseChainsInfos &inverse_chains);

  ~UrdfSceneEnv() = default;

  bool createEnvironment() override;

  bool reInit()override;


private:
  /**
   * Load SceneGraph from ROS-published URDF description
   * 
   * @param scene_description (std::string) the description param ROS published
   * @return (tesseract_scene_graph::SceneGraph::Ptr): A SceneGraph pointer that decodes the
   *    parsed scene
   */
  tesseract_scene_graph::SceneGraph::Ptr loadSceneGraphFromURDF_(const std::string &scene_description);
  /*****************************************************************8
   * Configurate AttachLocations in the URDF scene
   * 
   * Add AttachLocations in to the UrdfSceneEnv
   */
  void configAttachLocations_();

  /*****************************************************************8
   * Configurate AttachLocations in the URDF scene
   * 
   * Add AttachLocations in to the UrdfSceneEnv
   */
  void configAttachLocations_(const AttachObjectInfos &attaches);

  /*******************************************************************
   * Configurate Inserve Kinematics Chains
   * 
   * Inserve the chain of the manipulated object
   ******************************************************************/
  tesseract_scene_graph::SceneGraph::Ptr configInverseChains_();

  /*******************************************************************
   * Configurate Inserve Kinematics Chains
   * 
   * Inserve the chain of the manipulated object
   ******************************************************************/
  TesseractSceneGraphPtr configInverseChains_(const InverseChainsInfos &inverse_chains);

  /*****************************************************************8
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
  tesseract_scene_graph::SceneGraph::Ptr inverseEnvChain_(tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string src, std::string dst);

  /*****************************************************************8
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
  tesseract_scene_graph::SceneGraph::Ptr inverseEnvChainHelper_(tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string src, std::string dst);


  void updateInvertedEnv_(tesseract_scene_graph::SceneGraph::ConstPtr sg);

  void addToEnv_(tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string root_name);

  void newAttachLocation_(
    std::string attach_name,
    std::string attach_object_link,
    std::string object_baselink,
    std::vector<double> local_joint_tf_trans,
    std::vector<double> local_joint_tf_quat,
    bool fixed_base
  );

  std::string getLinkParentName_(tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string link_name);

  tesseract_scene_graph::Joint::Ptr getLinkParentJoint_(tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string link_name);

private:
  int steps_;
  AttachObjectInfos attaches_for_reset_;
};

}  // namespace vkc

#endif  // VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H
