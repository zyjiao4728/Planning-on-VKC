#include <vkc/env/urdf_scene_env.h>

namespace vkc {
class HouseholdEnv : public UrdfSceneEnv {
 public:
  HouseholdEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps);
  HouseholdEnv(ros::NodeHandle nh, bool plotting, bool rviz, int steps,
               const AttachObjectInfos &attaches,
               const InverseChainsInfos &inverse_chains);
  ~HouseholdEnv() = default;

  bool createEnvironment() override;
}
}  // namespace vkc