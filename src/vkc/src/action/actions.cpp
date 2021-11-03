#include "vkc/action/actions.h"

namespace vkc
{

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

// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, ActionBase::Ptr p_act)
{
    assert(nullptr != p_act);
    
    switch(p_act->getActionType())
    {
    case vkc::ActionType::PickAction:
        oss << *std::dynamic_pointer_cast<PickAction>(p_act);
        break;
    case vkc::ActionType::PlaceAction:
        oss << *std::dynamic_pointer_cast<PlaceAction>(p_act);
        break;
    case vkc::ActionType::UseAction:
        oss << *std::dynamic_pointer_cast<UseAction>(p_act);
        break;
    case vkc::ActionType::GotoAction:
        oss << *std::dynamic_pointer_cast<GotoAction>(p_act);
        break;
    default:
        std::cout << "unknown action" << std::endl;
        break;
    }
    
    return oss;
}

// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, ActionSeq& actions)
{
    oss << "actions sequence: " << std::endl;
    std::for_each(actions.begin(), actions.end(), [&](vkc::ActionBase::Ptr& p_act)
    {  
        oss << p_act << std::endl; 
    });
    
    return oss;
}
}