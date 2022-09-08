#ifndef VIRTUAL_KINEMATIC_CHAIN_ENVIRONMENT_H
#define VIRTUAL_KINEMATIC_CHAIN_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_msgs/ModifyEnvironment.h>

#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <fmt/core.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/utils.h>
#include <vkc/action/actions.h>
#include <vkc/construct_vkc.h>
#include <vkc/object/objects.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace vkc {
/**
 * @brief The Virtual Kinematic Chain (VKC) environment base class
 *
 * It provides a generic interface for all VKC environment as a library which
 * then can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 * It follows the format of tesseract_ros_example package, with some customized
 * features.
 */
class VKCEnvBasic {
 public:
  using Ptr = std::shared_ptr<VKCEnvBasic>;
  using UPtr = std::unique_ptr<VKCEnvBasic>;

  VKCEnvBasic(ros::NodeHandle nh_, bool plotting, bool rviz, int steps,
              bool inverse_kinematic_chain = true);
  VKCEnvBasic(ros::NodeHandle nh, ConstructVKC::Ptr vkc, bool plotting,
              bool rviz, int steps, bool inverse_kinematic_chain = true);

  virtual ~VKCEnvBasic() = default;

  // Create a environment for VKCc planning
  virtual bool createEnvironment();

  VKCEnvBasic::UPtr clone();

  void setEndEffector(std::string name);

  virtual bool reInit();
  void setRobotEndEffector(std::string link_name);

  void addAttachLocation(
      vkc::BaseObject::AttachLocation::Ptr attach_location_ptr);

  void updateAttachLocations(
      std::unordered_map<std::string, vkc::BaseObject::AttachLocation::ConstPtr>
          attach_locations);

  std::unordered_map<std::string, double> getHomePose();

  vkc::BaseObject::AttachLocation::ConstPtr getAttachLocation(std::string name);

  std::unordered_map<std::string, vkc::BaseObject::AttachLocation::ConstPtr>
  getAttachLocations();

  vkc::ConstructVKC::Ptr getVKCEnv();
  vkc::ConstructVKC::Ptr getPlotVKCEnv();

  tesseract_visualization::Visualization::Ptr getPlotter();

  std::string updateEnv(const std::vector<std::string>& joint_names,
                        const Eigen::VectorXd& joint_states,
                        ActionBase::Ptr action);
  std::string updatePlotEnv(const std::vector<std::string>& joint_names,
                            const Eigen::VectorXd& joint_states,
                            ActionBase::Ptr action);

  std::string getEndEffectorLink();

  void disableInverseKinematicChain();
  void enableInverseKinematicChain();

 protected:
  ros::NodeHandle nh_;
  bool rviz_; /**< @brief Enable rviz updating */
  bool plotting_;

  bool inverse_kinematic_chain_ = true;

  int steps_;

  std::unordered_map<std::string, double>
      home_pose_;                    /**< @brief Home pose of robot model */
  vkc::ConstructVKC::Ptr tesseract_; /**< @brief Tesseract Manager Class */

  vkc::ConstructVKC::Ptr plot_tesseract_;
  tesseract_visualization::Visualization::Ptr plotter_;
  std::string end_effector_link_;
  // the robot end effctor
  std::string robot_end_effector_link_;
  std::unordered_map<std::string, vkc::BaseObject::AttachLocation::ConstPtr>
      attach_locations_;
  std::vector<std::string> attached_links_;

  /**
   * @brief Set initial pose to home pose for all groups as defined in SRDF file
   * @return False if no home pose is defined
   */
  bool setHomePose();

  bool setInitPose(std::unordered_map<std::string, double> init_pose);

  bool loadRobotModel(const std::string& ENV_DESCRIPTION_PARAM,
                      const std::string& ENV_SEMANTIC_PARAM,
                      const std::string& END_EFFECTOR_LINK);

  bool initTesseractConfig();

  bool isGroupExist(std::string group_id);

  void attachObject(std::string attach_location_name,
                    vkc::ConstructVKC::Ptr tesseract,
                    Eigen::Isometry3d* tf = nullptr);

  void detachObject(std::string attach_location_name,
                    vkc::ConstructVKC::Ptr tesseract,
                    const std::string& new_attach_link = std::string(""));

  void detachTopObject(vkc::ConstructVKC::Ptr tesseract,
                       const std::string& new_attach_link = std::string(""));

  void addAttachedLink(std::string link_name);

  void removeTopAttachedLink();

  std::string getTopAttachedLink();

  bool ifAttachedLink(std::string link_name);

  bool isRobotArmFree();

  void updateAttachLocParentLink(std::string attach_loc_name,
                                 std::string parent_link_name);

  std::string updateEnv_(const std::vector<std::string>& joint_names,
                         const Eigen::VectorXd& joint_states,
                         ActionBase::Ptr action,
                         vkc::ConstructVKC::Ptr tesseract);

  void updateKinematicInfo(vkc::ConstructVKC::Ptr env);
};

}  // namespace vkc
#endif  // VIRTUAL_KINEMATIC_CHAIN_ENVIRONMENT_H
