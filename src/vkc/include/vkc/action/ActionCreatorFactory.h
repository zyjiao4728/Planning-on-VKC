#ifndef VKC_ACTION_CREATOR_FACTORY_H
#define VKC_ACTION_CREATOR_FACTORY_H


#include "vkc_example/scene_objects.h"
#include "vkc/action/actions.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>     // store scene interactive objects

namespace vkc
{
class ActionCreatorFactory
{
public:
    ActionCreatorFactory(SceneObjects &scene_objs) : scene_objs_(scene_objs)
    {
        action_mapper_.emplace("get_object", ActionType::PickAction);
        action_mapper_.emplace("use_object", ActionType::UseAction);
        action_mapper_.emplace("place", ActionType::PlaceAction);
        action_mapper_.emplace("fetch", ActionType::PlaceAction);
        action_mapper_.emplace("open_door", ActionType::PlaceAction);
        action_mapper_.emplace("close_door", ActionType::PlaceAction);
    }

    ActionBase::Ptr CreateAction(std::vector<std::string> &action_meta)
    {
        if (0 == action_mapper_.count(action_meta[0]))
        {
            return nullptr;
        }

        switch (action_mapper_[action_meta[0]])
        {
        case ActionType::PickAction:
            return CreatePickAction(action_meta);
            break;
        case ActionType::PlaceAction:
            return CreatePlaceAction(action_meta);
            break;
        case ActionType::UseAction:
            return CreateUseAction(action_meta);
            break;
        case ActionType::GotoAction:
        default:
            return nullptr;
            break;
        }
    }

private:
    ActionBase::Ptr CreatePickAction(std::vector<std::string> &action_meta)
    {
        IObject::Ptr picked_obj = scene_objs_.GetObject(action_meta[2]);
        if (!picked_obj)
        {
            ROS_ERROR("[%s]invalid picked object: %s",
                      __func__, action_meta[2].c_str());
            return nullptr;
        }

        return picked_obj->CreatePickAction(action_meta[1]);
    }

    ActionBase::Ptr CreatePlaceAction(std::vector<std::string> &action_meta)
    {
        IObject::Ptr placed_obj = scene_objs_.GetObject(action_meta[2]);
        if (!placed_obj)
        {
            ROS_ERROR("[%s]invalid placed object: %s",
                      __func__, action_meta[2].c_str());
            return nullptr;
        }

        return placed_obj->CreatePlaceAction(action_meta[1], action_meta[3]);
    }

    ActionBase::Ptr CreateUseAction(std::vector<std::string> &action_meta)
    {
        // TODO: how to define use syntax? use aaa for bbb
        IUsableObject::Ptr used_obj = std::dynamic_pointer_cast<IUsableObject>(
            scene_objs_.GetObject(action_meta[2]));
        IUsableObject::Ptr applied_obj = std::dynamic_pointer_cast<IUsableObject>(
            scene_objs_.GetObject(action_meta[3]));

        if (!used_obj || !applied_obj)
        {
            if (!used_obj)
            {
                ROS_ERROR("[%s]invalid use_action used object: %s",
                          __func__, action_meta[2].c_str());
            }
            if (!applied_obj)
            {
                ROS_ERROR("[%s]invalid use_action action object: %s",
                          __func__, action_meta[3].c_str());
            }

            return nullptr;
        }

        return used_obj->CreateUseAction(action_meta[1],
                                         *applied_obj);
    }

    SceneObjects& scene_objs_;
    std::map<std::string, ActionType> action_mapper_;
};
}

#endif // VKC_ACTION_CREATOR_FACTORY_H 