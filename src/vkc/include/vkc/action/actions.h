#ifndef VKC_ACTIONS_H
#define VKC_ACTIONS_H

#include <vkc/action/action_base.h>
#include <vkc/action/goto_action.h>
#include <vkc/action/pick_action.h>
#include <vkc/action/place_action.h>
#include <vkc/action/use_action.h>

// added: wanglei@bigai.ai 
// time: 2021-08-17 
#include <iostream>


namespace vkc{
  using ActionSeq = std::vector<vkc::ActionBase::Ptr>;


// added: wanglei@bigai.ai 
// time: 2021-08-17 
std::ostream& operator << (std::ostream& oss, ActionBase::Ptr p_act)
{
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

#endif
