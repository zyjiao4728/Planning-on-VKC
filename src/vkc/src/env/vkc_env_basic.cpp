#include <fmt/ranges.h>
#include <tesseract_rosutils/plotting.h>
#include <vkc/env/vkc_env_basic.h>
#include <vkc/object/basic_object.h>

using namespace tesseract_rosutils;

const std::string DEFAULT_VKC_GROUP_ID = "vkc";

namespace vkc
{
  // namespace vkc starts

  VKCEnvBasic::VKCEnvBasic(ros::NodeHandle nh, bool plotting, bool rviz,
                           int steps)
      : nh_(nh),
        plotting_(plotting),
        rviz_(rviz),
        tesseract_(std::make_shared<vkc::ConstructVKC>()),
        steps_(steps),
        plot_tesseract_(nullptr) {}

  VKCEnvBasic::VKCEnvBasic(ros::NodeHandle nh, ConstructVKC::Ptr vkc,
                           bool plotting, bool rviz, int steps)
      : nh_(nh),
        plotting_(plotting),
        rviz_(rviz),
        tesseract_(vkc),
        steps_(steps),
        plot_tesseract_(nullptr) {}

  VKCEnvBasic::UPtr VKCEnvBasic::clone() const
  {
    auto cloned_vkc_env =
        std::make_unique<VKCEnvBasic>(nh_, tesseract_->clone(), plotting_, rviz_, steps_);
    cloned_vkc_env->setEndEffector(end_effector_link_);
    cloned_vkc_env->setRobotEndEffector(robot_end_effector_link_);
    return cloned_vkc_env;
  }

  void VKCEnvBasic::setEndEffector(std::string link_name)
  {
    end_effector_link_ = link_name;
  }

  void VKCEnvBasic::setRobotEndEffector(std::string link_name)
  {
    robot_end_effector_link_ = link_name;
  }

  ConstructVKC::Ptr VKCEnvBasic::getVKCEnv() { return tesseract_; }
  ConstructVKC::Ptr VKCEnvBasic::getPlotVKCEnv() { return nullptr; }

  bool VKCEnvBasic::loadRobotModel(const std::string &ENV_DESCRIPTION_PARAM,
                                   const std::string &ENV_SEMANTIC_PARAM,
                                   const std::string &END_EFFECTOR_LINK)
  {
    // Initial setup, load xml directory defined in launch
    std::string env_urdf_xml_string, env_srdf_xml_string, end_effector_link;

    nh_.getParam(ENV_DESCRIPTION_PARAM, env_urdf_xml_string);
    nh_.getParam(ENV_SEMANTIC_PARAM, env_srdf_xml_string);
    nh_.getParam(END_EFFECTOR_LINK, end_effector_link);

    // at the beginning stage, current end effector
    // is the robot end effector
    setEndEffector(end_effector_link);
    setRobotEndEffector(end_effector_link);

    ROS_INFO("Loading environment URDF to scene graph...");
    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();

    if (!tesseract_->loadURDFtoSceneGraph(env_urdf_xml_string,
                                          env_srdf_xml_string, locator))
    {
      ROS_INFO("Failed to generate environment scene graph");
      return false;
    }

    // if (!plot_tesseract_->loadURDFtoSceneGraph(env_urdf_xml_string,
    //                                            env_srdf_xml_string, locator)) {
    //   ROS_INFO("Failed to generate environment scene graph");
    //   return false;
    // }

    return true;
  }

  tesseract_visualization::Visualization::Ptr VKCEnvBasic::getPlotter()
  {
    return plotter_;
  }

  bool VKCEnvBasic::initTesseractConfig()
  {
    // Initialize scene graph to tesseract environment
    ROS_INFO("Initializing tesseract...");
    /** @brief RViz Example Namespace */
    const std::string VKC_MONITOR_NAMESPACE = "tesseract_vkc";

    tesseract_->initTesseract(VKC_MONITOR_NAMESPACE);
    // plot_tesseract_->initTesseract("tesseract_vkc_plot");
    if (plotting_)
    {
      plotter_ = std::make_shared<ROSPlotting>(
          tesseract_->getTesseract()->getSceneGraph()->getRoot());
      plotter_->waitForConnection();
      if (plotter_ != nullptr && plotter_->isConnected())
        plotter_->waitForInput(
            "tesseract plotter init success, press enter to continue");
    }

    ROS_INFO("Tesseract initialized...");

    // These are used to keep visualization updated
    // if (rviz_) {
    //   modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(
    //       fmt::format("/{}/{}", VKC_MONITOR_NAMESPACE, modify_env_srv), false);
    //   get_env_changes_rviz_ =
    //       nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(
    //           fmt::format("/{}/{}", VKC_MONITOR_NAMESPACE,
    //           get_env_changes_srv), false);

    //   // Check RViz to make sure nothing has changed
    //   if (!checkRviz()) throw std::runtime_error("checkRviz failed");
    //   // return false;
    // }

    return true;
  }

  std::unordered_map<std::string, double> VKCEnvBasic::getHomePose()
  {
    return home_pose_;
  }

  void VKCEnvBasic::addAttachedLink(std::string link_name)
  {
    attached_links_.push_back(link_name);
  }

  void VKCEnvBasic::removeTopAttachedLink() { attached_links_.pop_back(); }

  std::string VKCEnvBasic::getTopAttachedLink() { return attached_links_.back(); }

  bool VKCEnvBasic::isRobotArmFree() { return attached_links_.size() == 0; }

  bool VKCEnvBasic::ifAttachedLink(std::string link_name)
  {
    // check if link_name already in the attched_links
    return (find(attached_links_.begin(), attached_links_.end(), link_name) !=
            attached_links_.end());
  }

  void VKCEnvBasic::addAttachLocation(
      vkc::BaseObject::AttachLocation attach_location)
  {
    vkc::BaseObject::AttachLocation::Ptr al_ptr =
        std::make_shared<vkc::BaseObject::AttachLocation>(
            std::move(attach_location));
    al_ptr->connection.parent_link_name = end_effector_link_;
    attach_locations_[al_ptr->name_] = al_ptr;
  }

  void VKCEnvBasic::addAttachLocations(
      std::unordered_map<std::string, vkc::BaseObject::AttachLocation::Ptr>
          attach_locations)
  {
    for (auto &attach_location : attach_locations)
    {
      attach_locations_[attach_location.first] = attach_location.second;
      attach_locations_[attach_location.first]->connection.parent_link_name =
          end_effector_link_;
    }
  }

  std::string VKCEnvBasic::getEndEffectorLink() { return end_effector_link_; }

  vkc::BaseObject::AttachLocation::Ptr VKCEnvBasic::getAttachLocation(
      std::string link_name)
  {
    auto attach_location = attach_locations_.find(link_name);
    // cannot find the attached link
    if (attach_location == attach_locations_.end())
      return nullptr;

    return attach_location->second;
  }

  bool VKCEnvBasic::setHomePose()
  {
    for (const auto &group_state :
         tesseract_->getSRDFModel()->kinematics_information.group_states.at(
             "vkc"))
    {
      if (std::string::npos == group_state.first.find("home"))
      {
        continue;
      }
      home_pose_ = group_state.second;
      // for (auto const& val : group_state.second) {
      //   // TODO: support for joints with multi-dofs, this may be currently
      //   wrong!! ROS_INFO(fmt::format("value: {}", val.first).c_str());
      //   home_pose_ = val.second;
      // }
    }
    if (home_pose_.size() <= 0)
    {
      ROS_WARN("No home pose defined!");
      return false;
    }
    tesseract_->getTesseract()->setState(home_pose_);
    // plot_tesseract_->getTesseract()->setState(home_pose_);
    return true;
  }

  bool VKCEnvBasic::setInitPose(
      std::unordered_map<std::string, double> init_pose)
  {
    tesseract_->getTesseract()->setState(init_pose);
    // plot_tesseract_->getTesseract()->setState(init_pose);
    return true;
  }

  void VKCEnvBasic::updateAttachLocParentLink(std::string attach_loc_name,
                                              std::string parent_link_name)
  {
    if (ifAttachedLink(attach_loc_name))
    {
      ROS_ERROR(
          "Cannot set link %s as the parent of %s, since %s is already attached "
          "to another link",
          parent_link_name.c_str(), attach_loc_name.c_str(),
          attach_loc_name.c_str());
      return;
    }
    attach_locations_.at(attach_loc_name)->connection.parent_link_name =
        parent_link_name;
  }

  void VKCEnvBasic::attachObject(std::string attach_location_name,
                                 vkc::ConstructVKC::Ptr tesseract,
                                 Eigen::Isometry3d *tf)
  {
    // update current end effector to the parent link of the attached object
    updateAttachLocParentLink(attach_location_name, end_effector_link_);

    // specified transform between end effector and attached location
    if (tf != nullptr)
    {
      // attach_locations_.at(attach_location_name)->connection.parent_to_joint_origin_transform
      // = (*tf).inverse();
      attach_locations_.at(attach_location_name)
          ->connection.parent_to_joint_origin_transform =
          tesseract->getTesseract()
              ->getLinkTransform(getEndEffectorLink())
              .inverse() *
          tesseract->getTesseract()->getLinkTransform(
              attach_locations_.at(attach_location_name)
                  ->connection.child_link_name);
      attach_locations_.at(attach_location_name)->connection.parent_link_name =
          getEndEffectorLink();
    }
    // default transform
    else
    {
      attach_locations_.at(attach_location_name)
          ->connection.parent_to_joint_origin_transform =
          tesseract->getTesseract()
              ->getLinkTransform(getEndEffectorLink())
              .inverse() *
          tesseract->getTesseract()->getLinkTransform(
              attach_locations_.at(attach_location_name)
                  ->connection.child_link_name);
      // attach_locations_.at(attach_location_name)->local_joint_origin_transform.inverse();
      attach_locations_.at(attach_location_name)->connection.parent_link_name =
          getEndEffectorLink();
    }
    ROS_INFO("[%s]pre end-effector: %s", __func__, getEndEffectorLink().c_str());

    // std::cout <<
    // tesseract->getTesseract()->getLinkTransform(getEndEffectorLink()).translation()
    // << std::endl; std::cout <<
    // tesseract->getTesseract()->getLinkTransform(getEndEffectorLink()).linear()
    // << std::endl; std::cout <<
    // tesseract->getTesseract()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name).translation()
    // << std::endl; std::cout <<
    // tesseract->getTesseract()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name).linear()
    // << std::endl; std::cout <<
    // attach_locations_.at(attach_location_name)->connection.parent_link_name <<
    // std::endl; std::cout <<
    // attach_locations_.at(attach_location_name)->connection.child_link_name <<
    // std::endl;

    Command::Ptr move_link = std::make_shared<MoveLinkCommand>(
        attach_locations_.at(attach_location_name)->connection);
    // TODO! make things in parenthesis into a joint
    tesseract->getTesseract()->applyCommand(move_link);
    end_effector_link_ = attach_locations_.at(attach_location_name)->base_link_;

    ROS_INFO("[%s]current end-effector: %s", __func__,
             getEndEffectorLink().c_str());
    addAttachedLink(attach_location_name);
  }

  void VKCEnvBasic::detachTopObject(vkc::ConstructVKC::Ptr tesseract,
                                    const std::string &new_attach_link)
  {
    std::string target_location_name = getTopAttachedLink();
    removeTopAttachedLink();

    end_effector_link_ =
        attach_locations_.at(target_location_name)->connection.parent_link_name;

    ROS_INFO("end_effector_link_: %s", end_effector_link_.c_str());

    const std::string attach_link_name =
        attach_locations_.at(target_location_name)->link_name_;
    std::string object_name =
        attach_link_name.substr(0, attach_link_name.rfind("_"));
    ROS_INFO("[%s]object link name: %s, object_name: %s", __func__,
             attach_link_name.c_str(), object_name.c_str());

    std::string new_parent_link{new_attach_link.empty() ? "world"
                                                        : new_attach_link};
    Joint new_joint(
        object_name + "_" +
        new_parent_link); // wanglei@2021-11-15, to optionally support container
                          // fill operation, such as put a egg into a basket
    ROS_INFO("[%s]detach %s from %s to attach to %s, assigned attach link: %s",
             __func__, target_location_name.c_str(), end_effector_link_.c_str(),
             new_parent_link.c_str(), new_attach_link.c_str());

    new_joint.parent_link_name = new_parent_link;
    new_joint.child_link_name = attach_link_name;
    new_joint.type = JointType::FIXED;
    new_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    new_joint.parent_to_joint_origin_transform =
        tesseract->getTesseract()->getLinkTransform(new_parent_link).inverse() *
        tesseract->getTesseract()->getLinkTransform(attach_link_name);
    Command::Ptr move_new_joint = std::make_shared<MoveLinkCommand>(new_joint);
    // bool link_moved =
    // tesseract->getTesseract()->applyMoveLinkCommand(new_joint);
    bool link_moved = tesseract->getTesseract()->applyCommand(move_new_joint);
    ROS_INFO("[%s]detach action, move link %s: %s", __func__,
             attach_link_name.c_str(), link_moved ? "true" : "false");
    // std::cout <<
    // tesseract->getTesseract()->getLinkTransform(attach_link_name).translation()
    // << std::endl;

    // author: wanglei@bigai.ai
    // date: 2021-12-08
    // reason: update attach location transformation for the object having two
    // attach locations
    const std::string target_object_base_link =
        attach_locations_.at(target_location_name)->base_link_;
    for (auto &attach_location : attach_locations_)
    {
      if (target_object_base_link == attach_location.second->base_link_)
      {
        attach_location.second->world_joint_origin_transform =
            tesseract->getTesseract()->getLinkTransform(
                attach_location.second->link_name_) *
            attach_location.second->local_joint_origin_transform;
      }
      // attach_locations_.at(target_location_name)->world_joint_origin_transform
      // =
      //     tesseract->getTesseract()->getLinkTransform(attach_link_name) *
      //     attach_locations_.at(target_location_name)->local_joint_origin_transform;
    }
  }

  void VKCEnvBasic::detachObject(std::string detach_location_name,
                                 vkc::ConstructVKC::Ptr tesseract,
                                 const std::string &new_attach_link)
  {
    if (!ifAttachedLink(detach_location_name))
      return;

    // detach previously attached links
    while (getTopAttachedLink() != detach_location_name)
      detachTopObject(tesseract);

    // detach the real target detach_location_name
    detachTopObject(tesseract, new_attach_link);
  }

  std::string VKCEnvBasic::updateEnv(const std::vector<std::string> &joint_names,
                                     const Eigen::VectorXd &joint_states,
                                     ActionBase::Ptr action)
  {
    return updateEnv_(joint_names, joint_states, action, tesseract_);
  }
  std::string VKCEnvBasic::updatePlotEnv(
      const std::vector<std::string> &joint_names,
      const Eigen::VectorXd &joint_states, ActionBase::Ptr action)
  {
    return DEFAULT_VKC_GROUP_ID;
    // return updateEnv_(joint_names, joint_states, action, plot_tesseract_);
  }

  std::string VKCEnvBasic::updateEnv_(const std::vector<std::string> &joint_names,
                                      const Eigen::VectorXd &joint_states,
                                      ActionBase::Ptr action,
                                      vkc::ConstructVKC::Ptr tesseract)
  {
    std::cout << __func__ << ": action" << std::endl
              << action << std::endl;
    // Set the current state to the last state of the trajectory
    if (joint_names.size())
    {
      // for (auto joint_name : joint_names){
      //   std::cout << joint_name << "," << joint_states << std::endl;
      // }
      tesseract->getTesseract()->setState(joint_names, joint_states);
    }

    if (action == nullptr)
    {
      if (rviz_)
      {
        // Now update rviz environment
        // tesseract_->getTesseract()->
      }
      updateKinematicInfo(tesseract);
      return DEFAULT_VKC_GROUP_ID;
    }

    std::string location_name;
    if (action->getActionType() == ActionType::PickAction)
    {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      location_name = pick_act->getAttachedObject();
      if (attach_locations_.find(location_name) == attach_locations_.end())
      {
        ROS_ERROR("Cannot find attach location %s inside environment!",
                  location_name.c_str());
        return DEFAULT_VKC_GROUP_ID;
      }
      attachObject(location_name, tesseract);
    }
    else if (action->getActionType() == ActionType::PlaceAction)
    {
      PlaceAction::Ptr place_act = std::dynamic_pointer_cast<PlaceAction>(action);
      location_name = place_act->getDetachedObject();
      std::cout << "detach: " << location_name << std::endl;
      std::cout << "pre end-effector: " << getEndEffectorLink() << std::endl;
      detachObject(location_name, tesseract, place_act->getNewAttachObject());
      std::cout << "current end-effector: " << getEndEffectorLink() << std::endl;
      // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    else if (action->getActionType() == ActionType::UseAction)
    {
      UseAction::Ptr use_act = std::dynamic_pointer_cast<UseAction>(action);
      location_name = use_act->getAttachedObject();
      // std::cout << location_name << std::endl;
      Eigen::Isometry3d *tf = &(use_act->getTransform());
      attachObject(location_name, tesseract, tf);
    }

    if (rviz_)
    {
      // Now update rviz environment
      tesseract_->getMonitor()->updateEnvironmentWithCurrentState();
    }

    if (action->getActionType() != ActionType::GotoAction)
    {
      updateKinematicInfo(tesseract);
    }

    return DEFAULT_VKC_GROUP_ID;
  } // namespace vkc

  void VKCEnvBasic::updateKinematicInfo(vkc::ConstructVKC::Ptr tesseract)
  {
    CONSOLE_BRIDGE_logDebug("updating kinematic info");
    tesseract_srdf::KinematicsInformation kin_info;
    for (auto group_id : tesseract_->getSRDFModel()->kinematics_information.group_names)
    {
      if (group_id == DEFAULT_VKC_GROUP_ID)
      {
        tesseract_common::PluginInfo pi;
        pi.class_name = "KDLInvKinChainLMAFactory";
        pi.config["base_link"] = "world";
        pi.config["tip_link"] = end_effector_link_;
        kin_info.kinematics_plugin_info.inv_plugin_infos[DEFAULT_VKC_GROUP_ID]
            .plugins = {std::make_pair("KDLFInvKinLMA", pi)};
        tesseract_srdf::ChainGroup group;
        group.push_back(std::make_pair("world", end_effector_link_));
        kin_info.addChainGroup(DEFAULT_VKC_GROUP_ID, group);
      }
      else
      {
        tesseract_common::PluginInfo pi;
        pi.class_name = "KDLInvKinChainLMAFactory";
        pi.config["base_link"] = tesseract_->getSRDFModel()->kinematics_information.chain_groups.at(group_id)[0].first;
        pi.config["tip_link"] = tesseract_->getSRDFModel()->kinematics_information.chain_groups.at(group_id)[0].second;
        kin_info.kinematics_plugin_info.inv_plugin_infos[group_id]
            .plugins = {std::make_pair("KDLFInvKinLMA", pi)};
        tesseract_srdf::ChainGroup group;
        group.push_back(tesseract_->getSRDFModel()->kinematics_information.chain_groups.at(group_id)[0]);
        kin_info.addChainGroup(DEFAULT_VKC_GROUP_ID, group);
      }
    }

    auto cmd = std::make_shared<AddKinematicsInformationCommand>(kin_info);

    tesseract->getTesseract()->applyCommand(cmd);
  }

  bool VKCEnvBasic::isGroupExist(std::string group_id)
  {
    return tesseract_->getSRDFModel()->kinematics_information.hasChainGroup(
        group_id);
    // bool isfound_group = false;

    // for (const auto &groups_ :
    // tesseract_->getSRDFModel()->kinematics_information.chain_groups)
    // {
    //   if (groups_.first == group_id)
    //     isfound_group = true;
    // }

    // return isfound_group;
  }

  bool VKCEnvBasic::createEnvironment() { return false; }
  bool VKCEnvBasic::reInit()
  {
    // end_effector_link_ = robot_end_effector_link_;
    // ROS_INFO("[%s]revision: %u", __func__,
    //          plot_tesseract_->getTesseract()->getRevision());
    return true;
  }

} // namespace vkc