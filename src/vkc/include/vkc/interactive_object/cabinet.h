#ifndef VKC_INTERACTIVE_CABINET_H
#define VKC_INTERACTIVE_CABINET_H

#include "vkc/action/actions.h"
#include "vkc/interactive_object/interface_object.h"


#include <string>
#include <vector>
#include <map>
namespace vkc
{
class Cabinet : public IObject
{
public:
    using Ptr = std::shared_ptr<Cabinet>;

    Cabinet(const std::string& name):
        name_(name)
    {
        joint_name_ = name_ + "_cabinet_door_joint"; 
        attach_handle_name_ = std::string("attach_") + name_ + "_handle_link";

        joint_poses_.emplace("door_opened", 1.5);
        joint_poses_.emplace("door_closed", 0);
    }
    
    virtual const std::string& Name()const
    {
        return name_;
    }
    
    bool HasPoseName(const std::string& pose_name)
    {
        return (0 < joint_poses_.count(pose_name));
    }

    double DoorPose(const std::string& pose_name)
    {
        return joint_poses_[pose_name];
    }

    double OpenPose()
    {
        return joint_poses_["door_opened"];
    }
    
    double ClosePose()
    {
        return joint_poses_["door_closed"];
    }
    
    std::string AttachLinkName()
    {
        return attach_handle_name_;
    }
    
    std::string JointName()
    {
        return joint_name_;
    }


    virtual PickAction::Ptr CreatePickAction(const std::string& manipulator)
    {
        return std::make_shared<PickAction>(manipulator, attach_handle_name_);
    }

    virtual PlaceAction::Ptr CreatePlaceAction(const std::string &manipulator,
                                       const std::string &pose_name)   // name of the pose where to place the ball
    {
        if(0 == joint_poses_.count(pose_name))
        {
            return nullptr;
        }

        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        joint_objectives.emplace_back(joint_name_,
                                      joint_poses_[pose_name]);

        return std::make_shared<PlaceAction>(manipulator,
                                             attach_handle_name_,
                                             link_objectives,
                                             joint_objectives);
    }

    PlaceAction::Ptr CreateOpenDoorAction(const std::string &manipulator)
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        joint_objectives.emplace_back(joint_name_,
                                      joint_poses_["door_open"]);   // TODO: this may be danager if the poses mantained by user

        return std::make_shared<PlaceAction>(manipulator,
                                             attach_handle_name_,
                                             link_objectives,
                                             joint_objectives);
    }

    PlaceAction::Ptr CreateCloseDoorAction(const std::string &manipulator)
    {
        std::vector<LinkDesiredPose> link_objectives;
        std::vector<JointDesiredPose> joint_objectives;
        joint_objectives.emplace_back(joint_name_,
                                      joint_poses_["door_close"]);  // TODO: this may be danager if the poses mantained by user

        return std::make_shared<PlaceAction>(manipulator,
                                             attach_handle_name_,
                                             link_objectives,
                                             joint_objectives);
    }

    std::string name_;
    std::string joint_name_;
    std::string attach_handle_name_;

    std::map<std::string, double> joint_poses_;
};


using Cabinets = std::vector<Cabinet>;
}


#endif  // VKC_INTERACTIVE_CABINET_H
