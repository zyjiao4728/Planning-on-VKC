#ifndef SPHERE_CONSTRAINTS_H
#define SPHERE_CONSTRAINTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/MotionValidator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
class SphereConstraint : public ompl::base::Constraint
{
public:
  SphereConstraint(const unsigned int ambientDim, const unsigned int coDim, double tolerance = 0.001)
    : ompl::base::Constraint(ambientDim, coDim, tolerance)
  {
  }

  void SphereConstraintSettings(tesseract_environment::Environment::ConstPtr env,
                                tesseract_kinematics::ForwardKinematics::ConstPtr kin ,std::vector<double> example);

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  // void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const override;

private:
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  std::vector<std::string> links_;
  std::vector<std::string> joints_;
  Eigen::Isometry3d goal_;
};
}  // namespace tesseract_motion_planners

#endif  // SPHERE_CONSTRAINTS_H
