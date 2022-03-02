#ifndef DESCARTES_OPW_DESCARTES_OPW_KINEMATICS_H
#define DESCARTES_OPW_DESCARTES_OPW_KINEMATICS_H

#include <descartes_light/descartes_macros.h>
DESCARTES_IGNORE_WARNINGS_PUSH
#include <opw_kinematics/opw_kinematics.h>
DESCARTES_IGNORE_WARNINGS_POP

#include <descartes_light/visibility_control.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/utils.h>

namespace descartes_light
{
template <typename FloatType>
class OPWKinematics : public KinematicsInterface<FloatType>
{
public:
  OPWKinematics(const opw_kinematics::Parameters<FloatType>& params,
                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_base,
                const Eigen::Transform<FloatType, 3, Eigen::Isometry>& tool0_to_tip,
                const IsValidFn<FloatType>& is_valid_fn,
                const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn);

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          std::vector<FloatType>& solution_set) const override;
  bool fk(const FloatType* pose, Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const override;

  int dof() const override;

  void analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const override;

private:
  opw_kinematics::Parameters<FloatType> params_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_base_;
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool0_to_tip_;
  IsValidFn<FloatType> is_valid_fn_;
  GetRedundantSolutionsFn<FloatType> redundant_sol_fn_;

  bool ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
          const IsValidFn<FloatType>& is_valid_fn,
          const GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
          std::vector<FloatType>& solution_set) const;
};

using OPWKinematicsF = OPWKinematics<float>;
using OPWKinematicsD = OPWKinematics<double>;

}  // namespace descartes_light

#endif  // DESCARTES_OPW_DESCARTES_OPW_KINEMATICS_H
