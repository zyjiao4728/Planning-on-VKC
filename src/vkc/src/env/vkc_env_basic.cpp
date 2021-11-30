#include <vkc/env/vkc_env_basic.h>

const std::string DEFAULT_VKC_GROUP_ID = "vkc";

namespace vkc
{
  // namespace vkc starts

  VKCEnvBasic::VKCEnvBasic(ros::NodeHandle nh, bool plotting, bool rviz)
      : nh_(nh), plotting_(plotting), rviz_(rviz),
        tesseract_(std::make_shared<vkc::ConstructVKC>()),
        plot_tesseract_(std::make_shared<vkc::ConstructVKC>())
  {
    // Initial number of past revisions
    n_past_revisions_ = 0;
    n_past_plot_revisions_ = 0;
  }

  bool VKCEnvBasic::reInit()
  {
    ROS_WARN("[%s]TODO: reset the rviz via call programming interface", __func__);
    // attached_links_.clear();
    end_effector_link_ = robot_end_effector_link_;
    ROS_INFO("[%s]revision: %lu", __func__, plot_tesseract_->getTesseractEnvironment()->getRevision());
    if (!sendRvizChanges(n_past_plot_revisions_, plot_tesseract_))
      return false;
  }

  void VKCEnvBasic::setEndEffector(std::string link_name)
  {
    end_effector_link_ = link_name;
  }

  void VKCEnvBasic::setRobotEndEffector(std::string link_name)
  {
    robot_end_effector_link_ = link_name;
  }

  ConstructVKC::Ptr VKCEnvBasic::getVKCEnv()
  {
    return tesseract_;
  }
  ConstructVKC::Ptr VKCEnvBasic::getPlotVKCEnv()
  {
    return plot_tesseract_;
  }

  bool VKCEnvBasic::loadRobotModel(const std::string &ENV_DESCRIPTION_PARAM, const std::string &ENV_SEMANTIC_PARAM,
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
    ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();

    if (!tesseract_->loadURDFtoSceneGraph(env_urdf_xml_string, env_srdf_xml_string, locator))
    {
      ROS_INFO("Failed to generate environment scene graph");
      return false;
    }

    if (!plot_tesseract_->loadURDFtoSceneGraph(env_urdf_xml_string, env_srdf_xml_string, locator))
    {
      ROS_INFO("Failed to generate environment scene graph");
      return false;
    }

    return true;
  }

  bool VKCEnvBasic::initTesseractConfig(const std::string &modify_env_srv, const std::string &get_env_changes_srv)
  {
    // Initialize scene graph to tesseract environment
    ROS_INFO("Initializing tesseract...");

    tesseract_->initTesseract();
    plot_tesseract_->initTesseract();

    ROS_INFO("Tesseract initialized...");

    // These are used to keep visualization updated
    if (rviz_)
    {
      modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(modify_env_srv, false);
      get_env_changes_rviz_ = nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(get_env_changes_srv, false);

      // Check RViz to make sure nothing has changed
      if (!checkRviz())
        return false;
    }

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

  void VKCEnvBasic::removeTopAttachedLink()
  {
    attached_links_.pop_back();
  }

  std::string VKCEnvBasic::getTopAttachedLink()
  {
    return attached_links_.back();
  }

  bool VKCEnvBasic::isRobotArmFree()
  {
    return attached_links_.size() == 0;
  }

  bool VKCEnvBasic::ifAttachedLink(std::string link_name)
  {
    // check if link_name already in the attched_links
    return (find(attached_links_.begin(), attached_links_.end(), link_name) != attached_links_.end());
  }

  void VKCEnvBasic::addAttachLocation(vkc::BaseObject::AttachLocation attach_location)
  {
    vkc::BaseObject::AttachLocation::Ptr al_ptr = std::make_shared<vkc::BaseObject::AttachLocation>(std::move(attach_location));
    al_ptr->connection.parent_link_name = end_effector_link_;
    attach_locations_[al_ptr->name_] = al_ptr;
  }

  void VKCEnvBasic::addAttachLocations(
      std::unordered_map<std::string, vkc::BaseObject::AttachLocation::Ptr> attach_locations)
  {
    for (auto &attach_location : attach_locations)
    {
      attach_locations_[attach_location.first] = attach_location.second;
      attach_locations_[attach_location.first]->connection.parent_link_name = end_effector_link_;
    }
  }

  std::string VKCEnvBasic::getEndEffectorLink()
  {
    return end_effector_link_;
  }

  vkc::BaseObject::AttachLocation::Ptr VKCEnvBasic::getAttachLocation(std::string link_name)
  {
    auto attach_location = attach_locations_.find(link_name);
    // cannot find the attached link
    if (attach_location == attach_locations_.end())
      return nullptr;

    return attach_location->second;
  }

  bool VKCEnvBasic::setHomePose()
  {
    for (const auto &group_state : tesseract_->getTesseract()->getSRDFModel()->getGroupStates())
    {
      if (std::string::npos == group_state.name_.find("home"))
      {
        continue;
      }
      for (auto const &val : group_state.joint_values_)
      {
        // TODO: support for joints with multi-dofs
        home_pose_[val.first] = val.second[0];
      }
    }
    if (home_pose_.size() <= 0)
    {
      ROS_WARN("No home pose defined!");
      return false;
    }
    tesseract_->getTesseractEnvironment()->setState(home_pose_);
    plot_tesseract_->getTesseractEnvironment()->setState(home_pose_);
    return true;
  }

  bool VKCEnvBasic::setInitPose(std::unordered_map<std::string, double> init_pose)
  {
    tesseract_->getTesseractEnvironment()->setState(init_pose);
    plot_tesseract_->getTesseractEnvironment()->setState(init_pose);
    return true;
  }

  bool VKCEnvBasic::checkRviz()
  {
    // Get the current state of the environment.
    // Usually you would not be getting environment state from rviz
    // this is just an example. You would be gettting it from the
    // environment_monitor node. Need to update examples to launch
    // environment_monitor node.
    get_env_changes_rviz_.waitForExistence();
    tesseract_msgs::GetEnvironmentChanges env_changes;
    env_changes.request.revision = 0;
    if (get_env_changes_rviz_.call(env_changes))
    {
      ROS_INFO("Retrieve current environment changes!");
    }
    else
    {
      ROS_ERROR("Failed to retrieve current environment changes!");
      return false;
    }

    // There should not be any changes but check
    if (env_changes.response.revision != 0)
    {
      ROS_ERROR("The environment has changed externally!");
      return false;
    }
    return true;
  }

  bool VKCEnvBasic::sendRvizChanges(unsigned long &past_revision, vkc::ConstructVKC::Ptr tesseract)
  {
    bool ret = sendRvizChanges_(past_revision, tesseract);
    past_revision = (unsigned long)tesseract->getTesseractEnvironment()->getRevision();

    return ret;
  }

  bool VKCEnvBasic::sendRvizChanges_(unsigned long past_revision, vkc::ConstructVKC::Ptr tesseract)
  {
    ROS_INFO("[%s]start to update rviz environment...", __func__);
    modify_env_rviz_.waitForExistence();
    tesseract_msgs::ModifyEnvironment update_env;
    update_env.request.id = tesseract->getTesseractEnvironment()->getName();
    update_env.request.revision = past_revision;

    if (!tesseract_rosutils::toMsg(update_env.request.commands,
                                   tesseract->getTesseractEnvironment()->getCommandHistory(),
                                   update_env.request.revision))
    {
      ROS_ERROR("Failed to generate commands to update rviz environment!");
      return false;
    }

    if (modify_env_rviz_.call(update_env))
    {
      ROS_INFO("[%s]rviz environment updated, result: %d, revision: %llu, past revision: %lu, current_revision: %lu",
               __func__, update_env.response.success, update_env.response.revision, past_revision,
               tesseract->getTesseractEnvironment()->getRevision());
    }
    else
    {
      ROS_INFO("[%s]failed to update rviz environment, result: %d, revision: %llu, past revision: %lu, current_revision: %lu",
               __func__, update_env.response.success, update_env.response.revision, past_revision,
               tesseract->getTesseractEnvironment()->getRevision());
      return false;
    }

    return true;
  }

  void VKCEnvBasic::updateAttachLocParentLink(std::string attach_loc_name, std::string parent_link_name)
  {
    if (ifAttachedLink(attach_loc_name))
    {
      ROS_ERROR("Cannot set link %s as the parent of %s, since %s is already attached to another link",
                parent_link_name.c_str(), attach_loc_name.c_str(), attach_loc_name.c_str());
      return;
    }
    attach_locations_.at(attach_loc_name)->connection.parent_link_name = parent_link_name;
  }

  void VKCEnvBasic::attachObject(std::string attach_location_name, vkc::ConstructVKC::Ptr tesseract, Eigen::Isometry3d *tf)
  {
    // update current end effector to the parent link of the attached object
    updateAttachLocParentLink(attach_location_name, end_effector_link_);

    // specified transform between end effector and attached location
    if (tf != nullptr)
    {
      // attach_locations_.at(attach_location_name)->connection.parent_to_joint_origin_transform = (*tf).inverse();
      attach_locations_.at(attach_location_name)->connection.parent_to_joint_origin_transform = tesseract->getTesseractEnvironment()->getLinkTransform(getEndEffectorLink()).inverse() *
                                                                                                tesseract->getTesseractEnvironment()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name);
      attach_locations_.at(attach_location_name)->connection.parent_link_name = getEndEffectorLink();
    }
    // default transform
    else
    {
      attach_locations_.at(attach_location_name)->connection.parent_to_joint_origin_transform = tesseract->getTesseractEnvironment()->getLinkTransform(getEndEffectorLink()).inverse() *
                                                                                                tesseract->getTesseractEnvironment()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name);
      // attach_locations_.at(attach_location_name)->local_joint_origin_transform.inverse();
      attach_locations_.at(attach_location_name)->connection.parent_link_name = getEndEffectorLink();
    }
    std::cout << "pre end-effector: " << getEndEffectorLink() << std::endl;

    // std::cout << tesseract->getTesseractEnvironment()->getLinkTransform(getEndEffectorLink()).translation() << std::endl;
    // std::cout << tesseract->getTesseractEnvironment()->getLinkTransform(getEndEffectorLink()).linear() << std::endl;
    // std::cout << tesseract->getTesseractEnvironment()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name).translation() << std::endl;
    // std::cout << tesseract->getTesseractEnvironment()->getLinkTransform(attach_locations_.at(attach_location_name)->connection.child_link_name).linear() << std::endl;
    // std::cout << attach_locations_.at(attach_location_name)->connection.parent_link_name << std::endl;
    // std::cout << attach_locations_.at(attach_location_name)->connection.child_link_name << std::endl;

    tesseract->getTesseractEnvironment()->moveLink((attach_locations_.at(attach_location_name)->connection));
    end_effector_link_ = attach_locations_.at(attach_location_name)->base_link_;

    std::cout << "current end-effector: " << getEndEffectorLink() << std::endl;
    addAttachedLink(attach_location_name);
  }

  void VKCEnvBasic::detachTopObject(vkc::ConstructVKC::Ptr tesseract, const std::string& new_attach_link)
  {
    std::string target_location_name = getTopAttachedLink();
    removeTopAttachedLink();

    end_effector_link_ = attach_locations_.at(target_location_name)->connection.parent_link_name;

    ROS_INFO("end_effector_link_: %s", end_effector_link_.c_str());

    std::string link_name = attach_locations_.at(target_location_name)->link_name_;
    std::string object_name = link_name.substr(0, link_name.rfind("_"));
    ROS_INFO("[%s]object link name: %s, object_name: %s", __func__, link_name.c_str(), object_name.c_str());

    std::string new_parent_link { new_attach_link.empty() ? "world" : new_attach_link};
    Joint new_joint(object_name + "_" + new_parent_link);   // wanglei@2021-11-15, to optionally support container fill operation, such as put a egg into a basket
    ROS_INFO("[%s]detach %s from %s to attach to %s, assigned attach link: %s",
             __func__, target_location_name.c_str(), end_effector_link_.c_str(), new_parent_link.c_str(), new_attach_link.c_str());

    new_joint.parent_link_name = new_parent_link;
    new_joint.child_link_name = link_name;
    new_joint.type = JointType::FIXED;
    new_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    new_joint.parent_to_joint_origin_transform = tesseract->getTesseractEnvironment()->getLinkTransform(new_parent_link).inverse() *
        tesseract->getTesseractEnvironment()->getLinkTransform(link_name);
    bool link_moved = tesseract->getTesseractEnvironment()->moveLink(new_joint);
    ROS_INFO("[%s]detach action, move link %s: %s", __func__, link_name.c_str(), link_moved ? "true" : "false");
    // std::cout << tesseract->getTesseractEnvironment()->getLinkTransform(link_name).translation() << std::endl;
    attach_locations_.at(target_location_name)->world_joint_origin_transform =
        tesseract->getTesseractEnvironment()->getLinkTransform(link_name) *
        attach_locations_.at(target_location_name)->local_joint_origin_transform;

    
  }

  void VKCEnvBasic::detachObject(std::string detach_location_name, vkc::ConstructVKC::Ptr tesseract, const std::string& new_attach_link)
  {
    if (!ifAttachedLink(detach_location_name))
      return;

    // detach previously attached links
    while (getTopAttachedLink() != detach_location_name)
      detachTopObject(tesseract);

    // detach the real target detach_location_name
    detachTopObject(tesseract, new_attach_link);
  }

  std::string VKCEnvBasic::updateEnv(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_states, ActionBase::Ptr action)
  {
    return updateEnv_(joint_names, joint_states, action, tesseract_, n_past_revisions_);
  }
  std::string VKCEnvBasic::updatePlotEnv(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_states, ActionBase::Ptr action)
  {
    return updateEnv_(joint_names, joint_states, action, plot_tesseract_, n_past_plot_revisions_);
  }

  std::string VKCEnvBasic::updateEnv_(const std::vector<std::string> &joint_names,
                                      const Eigen::VectorXd &joint_states,
                                      ActionBase::Ptr action,
                                      vkc::ConstructVKC::Ptr tesseract,
                                      unsigned long &past_revision)
  {
    std::cout << __func__ << ": actioin" << std::endl
              << action << std::endl;

    // Set the current state to the last state of the pick trajectory
    tesseract->getTesseractEnvironment()->setState(joint_names, joint_states);

    if (action == nullptr)
    {
      if (rviz_)
      {
        // Now update rviz environment
        if (!sendRvizChanges(past_revision, tesseract))
          exit(1);
      }
      return DEFAULT_VKC_GROUP_ID;
    }
    
    std::string location_name;
    if (action->getActionType() == ActionType::PickAction)
    {
      PickAction::Ptr pick_act = std::dynamic_pointer_cast<PickAction>(action);
      location_name = pick_act->getAttachedObject();
      if (attach_locations_.find(location_name) == attach_locations_.end())
      {
        ROS_ERROR("Cannot find attach location %s inside environment!", location_name.c_str());
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
      if (!sendRvizChanges(past_revision, tesseract))
        exit(1);
    }

    if (action->getActionType() != ActionType::GotoAction)
    {
      tesseract->getTesseract()->getSRDFModel()->removeGroup(DEFAULT_VKC_GROUP_ID);

      SRDFModel::Group group;
      group.name_ = DEFAULT_VKC_GROUP_ID;
      group.chains_.push_back(std::pair<std::string, std::string>("world", end_effector_link_));
      tesseract->getTesseract()->getSRDFModel()->addGroup(group);

      tesseract->getTesseract()->clearKinematics();
      tesseract->getTesseract()->registerDefaultFwdKinSolvers();
      tesseract->getTesseract()->registerDefaultInvKinSolvers();
    }

    return DEFAULT_VKC_GROUP_ID;
  }

  bool VKCEnvBasic::isGroupExist(std::string group_id)
  {
    bool isfound_group = false;

    for (const auto &groups_ : tesseract_->getTesseract()->getSRDFModel()->getGroups())
    {
      if (groups_.name_ == group_id)
        isfound_group = true;
    }

    return isfound_group;
  }

} // namespace vkc
