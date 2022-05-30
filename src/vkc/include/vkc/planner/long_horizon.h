#ifndef VKC_LONG_HORIZON_H
#define VKC_LONG_HORIZON_H

#include <tesseract_common/macros.h>
#include <vkc/action/action_base.h>
#include <vkc/env/vkc_env_basic.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

namespace vkc {
class LongHorizonSeedGenerator {
 public:
  LongHorizonSeedGenerator();
  /**
   * @brief generate long horizon seeds inside actions
   *
   */
  void generate(VKCEnvBasic &env, std::vector<ActionBase::Ptr> actions,
                size_t window_size);
};
};  // namespace vkc

#endif