

#include "tesseract_motion_planners/ompl/sphere_constraints.h"

namespace tesseract_motion_planners
{
void SphereConstraint::SphereConstraintSettings(tesseract_environment::Environment::ConstPtr env,
                                                tesseract_kinematics::ForwardKinematics::ConstPtr kin, std::vector<double> example)
{
  this->env_ = env;
  this->kin_ = kin;
  this->joints_ = kin->getJointNames();
  this->links_ = kin->getLinkNames();

  tesseract_environment::EnvState::Ptr env_state = env_->getState(joints_, example);
  std::string tip = kin_->getTipLinkName();
  this->goal_ = env_state->transforms[tip];
}

void SphereConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  tesseract_environment::EnvState::Ptr env_state = env_->getState(joints_, x);
  std::string tip = kin_->getTipLinkName();
  Eigen::Isometry3d trans = env_state->transforms[tip];
  Eigen::Vector3d diff = trans.translation() - this->goal_.translation(); 
  out[0] = diff.norm();
}

// void SphereConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
// {
//   Eigen::MatrixXd manip_jacob;
//   kin_->calcJacobian(manip_jacob, x);
//   out = x.transpose().normalized();
  
// }

}  // namespace tesseract_motion_planners
