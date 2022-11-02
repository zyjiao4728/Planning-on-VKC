#ifndef VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H
#define VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>

#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_urdf/urdf_parser.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/object/objects.h>

#include <algorithm>
#include <string>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <vector>

namespace vkc {
using TesseractSceneGraphPtr = tesseract_scene_graph::SceneGraph::Ptr;
using TesseractSceneGraphConstPtr = tesseract_scene_graph::SceneGraph::ConstPtr;

std::vector<std::string> findPath(
    tesseract_scene_graph::SceneGraph::ConstPtr sg, const std::string &src,
    const std::string &dst);
bool findPathHelper(tesseract_scene_graph::SceneGraph::ConstPtr sg,
                    const std::string &src, const std::string &dst,
                    std::vector<std::string> &path);

void printSceneGraph(const tesseract_scene_graph::SceneGraph::ConstPtr &sg);
void printSceneGraphHelper(
    const tesseract_scene_graph::SceneGraph::ConstPtr &sg,
    const std::string &root_name, int level);

tesseract_scene_graph::SceneGraph::Ptr deepcopySceneGraph(
    tesseract_scene_graph::SceneGraph::ConstPtr sg);
void deepcopySceneGraphHelper(
    tesseract_scene_graph::SceneGraph::Ptr dst_sg,
    tesseract_scene_graph::SceneGraph::ConstPtr src_sg, std::string root_name);

/**
 * @brief Load dual ur5e husky for door opening task.
 */
class UrdfSceneEnv : public VKCEnvBasic {
 public:
  struct AttachObjectInfo {
    std::string attach_name;
    std::string attach_link;
    std::string base_link;
    std::vector<double> local_joint_tf_trans;
    std::vector<double> local_joint_tf_quat;
    std::unordered_map<std::string, Eigen::VectorXd> cartesian_constraints;

    AttachObjectInfo(std::string attach_name, std::string attach_link,
                     std::string base_link,
                     std::vector<double> local_joint_tf_trans,
                     std::vector<double> local_joint_tf_quat)
        : attach_name(attach_name),
          attach_link(attach_link),
          base_link(base_link),
          local_joint_tf_trans(local_joint_tf_trans),
          local_joint_tf_quat(local_joint_tf_quat) {
      cartesian_constraints.clear();
    }

    AttachObjectInfo(std::string attach_name, std::string attach_link,
                     std::string base_link,
                     std::vector<double> local_joint_tf_trans,
                     std::vector<double> local_joint_tf_quat, bool fixed_base)
        : attach_name(attach_name),
          attach_link(attach_link),
          base_link(base_link),
          local_joint_tf_trans(local_joint_tf_trans),
          local_joint_tf_quat(local_joint_tf_quat) {
      cartesian_constraints.clear();
      if (fixed_base) {
        cartesian_constraints[base_link] = Eigen::VectorXd();
      }
    }

    AttachObjectInfo(std::string attach_name, std::string attach_link,
                     std::string base_link,
                     std::vector<double> local_joint_tf_trans,
                     std::vector<double> local_joint_tf_quat,
                     const Eigen::VectorXd &cartesian_constraint_coeff)
        : attach_name(attach_name),
          attach_link(attach_link),
          base_link(base_link),
          local_joint_tf_trans(local_joint_tf_trans),
          local_joint_tf_quat(local_joint_tf_quat) {
      cartesian_constraints.clear();
      cartesian_constraints[base_link] = cartesian_constraint_coeff;
    }
  };

  struct InverseChainsInfo {
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

  bool reInit() override;

 private:
  /**
   * Load SceneGraph from ROS-published URDF description
   *
   * @param scene_description (std::string) the description param ROS published
   * @return (tesseract_scene_graph::SceneGraph::Ptr): A SceneGraph pointer that
   * decodes the parsed scene
   */
  tesseract_scene_graph::SceneGraph::Ptr loadSceneGraphFromURDF_(
      const std::string &scene_description);
  // /*****************************************************************8
  //  * Configurate AttachLocations in the URDF scene
  //  *
  //  * Add AttachLocations in to the UrdfSceneEnv
  //  */
  // void configAttachLocations_();

  /*****************************************************************8
   * Configurate AttachLocations in the URDF scene
   *
   * Add AttachLocations in to the UrdfSceneEnv
   */
  void configAttachLocations_(const AttachObjectInfos &attaches);

  // /*******************************************************************
  //  * Configurate Inserve Kinematics Chains
  //  *
  //  * Inserve the chain of the manipulated object
  //  ******************************************************************/
  // tesseract_scene_graph::SceneGraph::Ptr configInverseChains_();

  /*******************************************************************
   * Configurate Inserve Kinematics Chains
   *
   * Inserve the chain of the manipulated object
   ******************************************************************/
  TesseractSceneGraphPtr configInverseChains_(
      const InverseChainsInfos &inverse_chains);

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
  tesseract_scene_graph::SceneGraph::Ptr inverseEnvChain_(
      tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string src,
      std::string dst);

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
  tesseract_scene_graph::SceneGraph::Ptr inverseEnvChainHelper_(
      tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string src,
      std::string dst);

  void updateSceneGraphToEnv_(tesseract_scene_graph::SceneGraph::ConstPtr sg,
                              tesseract_environment::Environment::Ptr env);

  void addToEnv_(tesseract_scene_graph::SceneGraph::ConstPtr sg,
                 std::string root_name,
                 tesseract_environment::Environment::Ptr env);

  void newAttachLocation_(std::string attach_name,
                          std::string attach_object_link,
                          std::string object_baselink,
                          std::vector<double> local_joint_tf_trans,
                          std::vector<double> local_joint_tf_quat,
                          const std::unordered_map<std::string, Eigen::VectorXd>
                              &cartesian_constraints);

  std::string getLinkParentName_(tesseract_scene_graph::SceneGraph::ConstPtr sg,
                                 std::string link_name);

  tesseract_scene_graph::Joint::ConstPtr getLinkParentJoint_(
      tesseract_scene_graph::SceneGraph::ConstPtr sg, std::string link_name);

 private:
  int steps_;
  AttachObjectInfos attaches_;
  InverseChainsInfos inverse_chains_;
};

}  // namespace vkc

#endif  // VIRTUAL_KINEMATIC_CHAIN_OPEN_DOOR_ENV_H
