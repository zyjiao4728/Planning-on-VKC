#include <vkc/planner/long_horizon.h>

namespace vkc {

LongHorizonSeedGenerator::LongHorizonSeedGenerator() {}

void LongHorizonSeedGenerator::generate(VKCEnvBasic &env,
                                        std::vector<ActionBase::Ptr> actions,
                                        size_t window_size) {
  auto cloned_env = std::move(env.clone());
  auto sub_actions = {actions.begin(),
                      actions.begin() + std::min(window_size, actions.size())};
  for (auto &action : sub_actions) {
     
  }
}

}  // namespace vkc