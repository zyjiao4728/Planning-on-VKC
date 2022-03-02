#ifndef VKC_INTERACTIVE_STICK_H
#define VKC_INTERACTIVE_STICK_H

#include "vkc/action/actions.h"
#include "vkc/interactive_object/interface_object.h"


#include <string>
#include <vector>
namespace vkc
{
// added: wanglei@bigai.ai 
// time: 2021-08-17
// reason: for defining the type of entity to save the interactive information 
class Stick : public IUsableObject
{
public:
    using Ptr = std::shared_ptr<Stick>;


    Stick(const std::string& name):
        name_(name)
    {
        link_name_ = name_ + "_stick_link";
        base_link_name_ = name_ + "_base_link";
        attach_link_name_ = std::string("attach_") + name_ + "_stick_link";
    }
    
    virtual const std::string& Name()const
    {
        return name_;
    }
    
    double* Pose()
    {
        return pos_;
    }
    
    const std::string& AttachLinkName()
    {
        return attach_link_name_;
    }
    
    std::string LinkName()
    {
        return link_name_;
    }
    
    std::string BaseLinkName()
    {
        return base_link_name_;
    }

    virtual PickAction::Ptr CreatePickAction(const std::string& manipulator)
    {
        return std::make_shared<PickAction>(manipulator, attach_link_name_);
    }

    virtual UseAction::Ptr CreateUseAction(const std::string &manipulator,
                                           IUsableObject &applied_object) // which the stick acts on
    {
        return applied_object.CreateUseAction(manipulator,
                                              link_name_);
    }

    virtual UseAction::Ptr CreateUseAction(const std::string &manipulator,
                                           const std::string &end_effect_link)
    {
        return nullptr;
    }

    virtual PlaceAction::Ptr CreatePlaceAction(const std::string &manipulator, const std::string& pose_name = "")
    {
        Eigen::Isometry3d tf;

        tf.translation() = Eigen::Vector3d(pos_[0], pos_[1], pos_[2]);
        tf.linear() = Eigen::Quaterniond(pos_[3], pos_[4], pos_[5], pos_[6]).matrix();

        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        link_objectives.emplace_back(base_link_name_, tf);

        return std::make_shared<PlaceAction>(manipulator,
                                             attach_link_name_,
                                             link_objectives,
                                             joint_objectives,
                                             false);
    }

private:
    std::string name_;
    std::string link_name_;
    std::string base_link_name_;
    std::string attach_link_name_;
    double pos_[7]{2.5, 1, 0.8, 0.7071, 0, 0, -0.7071};
};


using Sticks = std::vector<Stick>; 
}

#endif // VKC_INTERACTIVE_STICK_H