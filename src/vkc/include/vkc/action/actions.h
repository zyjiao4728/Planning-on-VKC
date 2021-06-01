#ifndef VKC_ACTIONS_H
#define VKC_ACTIONS_H

#include <vkc/action/action_base.h>
#include <vkc/action/goto_action.h>
#include <vkc/action/pick_action.h>
#include <vkc/action/place_action.h>
#include <vkc/action/use_action.h>

namespace vkc{
  using ActionSeq = std::vector<vkc::ActionBase::Ptr>;
}

#endif