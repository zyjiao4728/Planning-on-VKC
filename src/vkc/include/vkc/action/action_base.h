#ifndef VKC_ACTION_BASE_H
#define VKC_ACTION_BASE_H

#include <string>
#include <Eigen/Eigen>

// added: wanglei@bigai.ai 
// time: 2021-08-17 
#include <memory>
#include <iostream>




namespace vkc
{

enum class ActionType
{
  GotoAction,
  PickAction,
  PlaceAction,
  UseAction
};

struct LinkDesiredPose{
  std::string link_name;
  Eigen::Isometry3d tf;

  LinkDesiredPose(std::string lname, Eigen::Isometry3d &tf_input)
    : link_name(lname), tf(tf_input)
  {}
};

// added: wanglei@bigai.ai 
// time: 2021-08-17
// reason: for output Eigen::Isometry to std::ostream 
std::ostream& operator << (std::ostream& oss, Eigen::Isometry3d& tf)
{
    for(int r = 0; r < tf.rows(); ++r)
    {
        oss << "\t\t";
        for(int c = 0; c < tf.cols(); ++c)
        {
            oss << tf(r,c) << " "; 
        }
        
        oss << std::endl;
    }
            
    
    return oss;
}

// added: wanglei@bigai.ai 
// time: 2021-08-17 
// reason: for output LinkDesiredPose to std::ostream easily
std::ostream& operator << (std::ostream& oss, std::vector<vkc::LinkDesiredPose>& poses)
{
    for(auto& pos : poses)
    {
        oss << "-----------------------" << std::endl
            << "\tlink: " << pos.link_name << std::endl
            << "\ttf: " << std::endl
            << pos.tf << std::endl;
            
    }
    
    return oss;
}

struct JointDesiredPose{
  std::string joint_name;
  double joint_angle;

  JointDesiredPose(std::string jname, double jval){
    joint_name = jname;
    joint_angle = jval;
  }
};

// added: wanglei@bigai.ai 
// time: 2021-08-17 
// reason: for output JointDesiredPose to std::ostream easily
std::ostream& operator << (std::ostream& oss, std::vector<vkc::JointDesiredPose>& poses)
{

    for(auto& pos : poses)
    {
        oss << "-----------------------" << std::endl
            << "\tjoint: " << pos.joint_name  << std::endl
            << "\tvalue: " << pos.joint_angle << std::endl;
    }

    return oss;
}

class ActionBase
{
public:
  using Ptr = std::shared_ptr<ActionBase>;

  ActionBase(ActionType action_type, std::string manipulator_id)
  {
    action_type_ = action_type;
    manipulator_id_ = manipulator_id;

    // added: wanglei@bigai.ai
    // time: 2021-08-27
    // reason: mark and specify these action need init trajectory
    init_traj_required_ = true;
  }

  virtual ~ActionBase() = default;

public:
  ActionType getActionType()
  {
    return action_type_;
  }

  std::string getManipulatorID()
  {
    return manipulator_id_;
  }

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  bool RequireInitTraj(bool is_required)
  {
    bool pre_state = init_traj_required_;
    init_traj_required_ = is_required;

    return pre_state;
  }

  bool RequireInitTraj()const
  {
    std::cout << __func__ << init_traj_required_ << std::endl;
    return init_traj_required_;
  }

protected:
  ActionType action_type_;
  std::string manipulator_id_;

  // added: wanglei@bigai.ai
  // time: 2021-08-27
  // reason: mark and specify these action need init trajectory
  bool init_traj_required_;
};

} // end of namespace vkc

#endif
