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

#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <vkc/action/actions.h>
#include <vkc/construct_vkc.h>
#include <vkc/object/objects.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace vkc
{
/**
 * @brief The Virtual Kinematic Chain (VKC) environment base class
 *
 * It provides a generic interface for all VKC environment as a library which then
 * can easily be integrated as unit tests so breaking changes are caught.
 * Also it provides a few utility functions for checking rviz environment and
 * updating the rviz environment.
 * It follows the format of tesseract_ros_example package, with some customized features.
 */
class VKCEnvBasic
{
public:
  VKCEnvBasic(ros::NodeHandle nh_, bool plotting, bool rviz);
  
  virtual ~VKCEnvBasic() = default;

  // Create a environment for VKCc planning
  virtual bool createEnvironment() = 0;

  void setEndEffector(std::string name);

  virtual bool reInit();
  void setRobotEndEffector(std::string link_name);

  void addAttachLocation(vkc::BaseObject::AttachLocation attach_location);

  void addAttachLocations(
    std::unordered_map<std::string, vkc::BaseObject::AttachLocation::Ptr> attach_locations
  );

  std::unordered_map<std::string, double> getHomePose();

  vkc::BaseObject::AttachLocation::Ptr getAttachLocation(std::string name);

  vkc::ConstructVKC::Ptr getVKCEnv();
  vkc::ConstructVKC::Ptr getPlotVKCEnv();

  std::string updateEnv(const std::vector<std::string>& joint_names, const Eigen::VectorXd& joint_states, ActionBase::Ptr action);
  std::string updatePlotEnv(const std::vector<std::string>& joint_names, const Eigen::VectorXd& joint_states, ActionBase::Ptr action);

  std::string getEndEffectorLink();

  

  

protected:
  ros::NodeHandle nh_;
  bool plotting_; /**< @brief Enable plotting so data is published for rviz if available */
  bool rviz_;     /**< @brief Enable rviz updating */
  
  std::unordered_map<std::string, double> home_pose_; /**< @brief Home pose of robot model */

  unsigned long n_past_revisions_;
  vkc::ConstructVKC::Ptr tesseract_;                  /**< @brief Tesseract Manager Class */

  unsigned long n_past_plot_revisions_;
  vkc::ConstructVKC::Ptr plot_tesseract_;                  /**< @brief Tesseract Manager Class */

  ros::ServiceClient modify_env_rviz_;                /**< @brief Service for modifying tesseract environment in rviz */
  ros::ServiceClient get_env_changes_rviz_;           /**< @brief Get the environment changes from rviz */
  // current end effector link of the whole body, which could be changed over time
  std::string end_effector_link_;
  // the robot end effctor
  std::string robot_end_effector_link_;
  std::unordered_map<std::string, vkc::BaseObject::AttachLocation::Ptr> attach_locations_;
  std::vector<std::string> attached_links_;
  /**
   * @brief Set initial pose to home pose for all groups as defined in SRDF file
   * @return False if no home pose is defined
   */
  bool setHomePose();

  bool setInitPose(std::unordered_map<std::string, double> init_pose);

  /**
   * @brief Check rviz and make sure the rviz environment revision number is zero.
   * @return True if revision number is zero, otherwise false.
   */
  bool checkRviz();

  bool sendRvizChanges(unsigned long& past_revision, vkc::ConstructVKC::Ptr tesseract);

  /**
   * @brief Send RViz the latest number of commands
   * @param n The past revision number
   * @return True if successful otherwise false
   */
  bool sendRvizChanges_(unsigned long past_revision, vkc::ConstructVKC::Ptr tesseract);

  bool loadRobotModel(
    const std::string &ENV_DESCRIPTION_PARAM,
    const std::string &ENV_SEMANTIC_PARAM,
    const std::string &END_EFFECTOR_LINK
  );

  bool initTesseractConfig(
    const std::string &modify_env_srv, 
    const std::string &get_env_changes_srv
  );

  bool isGroupExist(std::string group_id);


  void attachObject(std::string attach_location_name, vkc::ConstructVKC::Ptr tesseract, Eigen::Isometry3d* tf = nullptr);

  void detachObject(std::string attach_location_name, vkc::ConstructVKC::Ptr tesseract, const std::string& new_attach_link= std::string(""));

  void detachTopObject(vkc::ConstructVKC::Ptr tesseract, const std::string& new_attach_link = std::string(""));

  void addAttachedLink(std::string link_name);

  void removeTopAttachedLink();

  std::string getTopAttachedLink();

  bool ifAttachedLink(std::string link_name);

  bool isRobotArmFree();

  void updateAttachLocParentLink(std::string attach_loc_name, std::string parent_link_name);

  std::string updateEnv_(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_states,
                         ActionBase::Ptr action, vkc::ConstructVKC::Ptr tesseract, unsigned long &past_revision);
};

}  // namespace vkc_example
#endif  // VIRTUAL_KINEMATIC_CHAIN_ENVIRONMENT_H
