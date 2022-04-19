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

namespace vkc {
using ActionSeq = std::vector<vkc::ActionBase::Ptr>;

// added: wanglei@bigai.ai
// time: 2021-08-17
std::ostream& operator<<(std::ostream& oss, ActionBase::Ptr p_act);

// added: wanglei@bigai.ai
// time: 2021-08-17
std::ostream& operator<<(std::ostream& oss, ActionSeq& actions);

}  // namespace vkc

#endif
