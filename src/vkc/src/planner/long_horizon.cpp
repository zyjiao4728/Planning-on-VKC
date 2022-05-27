#include <vkc/planner/long_horizon.h>

namespace vkc {
void generateLongHorizonSeeds(std::vector<ActionBase::Ptr> actions,
                              size_t window_size) {
  auto sub_actions = {actions.begin(),
                      actions.begin() + std::min(window_size, actions.size())};
    for (auto &action: sub_actions){
        
    }
}
}  // namespace vkc