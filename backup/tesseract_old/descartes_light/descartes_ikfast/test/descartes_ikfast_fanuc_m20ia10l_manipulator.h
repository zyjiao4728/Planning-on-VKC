#ifndef DESCARTES_IKFAST_FANUC_M20IA10L_MANIPULATOR_H
#define DESCARTES_IKFAST_FANUC_M20IA10L_MANIPULATOR_H

#include <descartes_light/visibility_control.h>
#include <descartes_ikfast/impl/ikfast_kinematics.hpp>
#include <descartes_light/utils.h>

namespace descartes_ikfast_unit
{
template <typename FloatType>
class FanucM20ia10lKinematics : public descartes_light::IKFastKinematics<FloatType>
{
public:
  FanucM20ia10lKinematics(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_robot_base,
                          const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
                          const descartes_light::IsValidFn<FloatType>& is_valid_fn,
                          const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn)
    : descartes_light::IKFastKinematics<FloatType>(world_to_robot_base, tool0_to_tip, is_valid_fn, redundant_sol_fn)
  {
  }
};

using FanucM20ia10lKinematicsD = FanucM20ia10lKinematics<double>;
using FanucM20ia10lKinematicsF = FanucM20ia10lKinematics<float>;

}  // namespace descartes_ikfast_unit
#endif  // DESCARTES_IKFAST_FANUC_M20IA10L_MANIPULATOR_H
