#ifndef VKC_ACTION_SEQUENCE_H
#define VKC_ACTION_SEQUENCE_H

#include <vkc/action/actions.h>

namespace vkc
{
class ActionSequence
{
public:
  ActionSequence()
  {
  }

  ~ActionSequence() = default;

  virtual void createActionSequnce() = 0;

  std::vector<ActionBase::Ptr> getActionSequence()
  {
    return action_sequence_;
  }

protected:
  std::vector<ActionBase::Ptr> action_sequence_;

private:
  void clear()
  {
    action_sequence_.clear();
  }
};
}  // namespace vkc

#endif  // VKC_ACTION_SEQUENCE_H