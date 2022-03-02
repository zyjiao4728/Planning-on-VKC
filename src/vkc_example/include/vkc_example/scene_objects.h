#ifndef VKC_SCENE_OBJECTS_H
#define VKC_SCENE_OBJECTS_H

#include "vkc/interactive_object/ball.h"
#include "vkc/interactive_object/stick.h"
#include "vkc/interactive_object/cabinet.h"
#include "vkc/interactive_object/interface_object.h"
#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>

namespace vkc
{
    class SceneObjects
    {
    public:
        SceneObjects()
        {
            Init();
        }
        ~SceneObjects() {}

        void Init()
        {
            // prepare mock data
            objects_.emplace_back(std::make_shared<Ball>("marker0"));
            objects_.emplace_back(std::make_shared<Stick>("stick0"));
            objects_.emplace_back(std::make_shared<Cabinet>("cabinet0"));
        }

        IObject::Ptr GetObject(const std::string &name)
        {
            auto obj_it = std::find_if(objects_.begin(), objects_.end(),
                                       [&](IObject::Ptr &obj_ptr)
                                       { return name == obj_ptr->Name(); });
            if (obj_it != objects_.end())
            {
                return *obj_it;
            }

            std::cout << "[" << __func__ << "]invalid action object: " << name << std::endl;
            return nullptr;
        }

    private:
        std::vector<IObject::Ptr> objects_;
    };
}

#endif // VKC_SCENE_OBJECTS_H