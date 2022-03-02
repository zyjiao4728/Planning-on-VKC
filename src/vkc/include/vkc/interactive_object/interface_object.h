#ifndef VKC_INTERACTIVE_INTERFACE_OBJECT_H
#define VKC_INTERACTIVE_INTERFACE_OBJECT_H

#include "vkc/action/actions.h"
#include <memory>

namespace vkc
{
class IObject
{
public:
    using Ptr = std::shared_ptr<IObject>;
    IObject() {}
    virtual ~IObject(){}
    virtual const std::string& Name()const = 0;
    virtual PickAction::Ptr CreatePickAction(const std::string& manipulator) = 0;
    virtual PlaceAction::Ptr CreatePlaceAction(const std::string &manipulator,
                                       const std::string &pose_name) = 0;                                
};


class IUsableObject : public IObject
{
public:
    using Ptr = std::shared_ptr<IUsableObject>;
    IUsableObject() {}
    virtual ~IUsableObject() {}
    virtual UseAction::Ptr CreateUseAction(const std::string &manipulator,
                                           IUsableObject &applied_object) = 0;

    virtual UseAction::Ptr CreateUseAction(const std::string &manipulator,
                                           const std::string &end_effect_link) = 0;
};
}

#endif // VKC_INTERACTIVE_INTERFACE_OBJECT_H