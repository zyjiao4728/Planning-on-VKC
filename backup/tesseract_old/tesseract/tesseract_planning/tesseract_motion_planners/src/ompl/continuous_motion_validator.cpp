#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_motion_planners/ompl/continuous_motion_validator.h"

namespace tesseract_motion_planners
{
ContinuousMotionValidator::ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                                                     tesseract_environment::Environment::ConstPtr env,
                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : MotionValidator(space_info), env_(std::move(env)), kin_(std::move(kin))
{
  joints_ = kin_->getJointNames();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  links_ = adj_map.getActiveLinkNames();
  this->joint_names_ = kin_->getJointNames();
  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(links_);
  contact_manager_->setContactDistanceThreshold(0);
  populateAdjacent();
}

ContinuousMotionValidator::ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                                                     tesseract_environment::Environment::ConstPtr env,
                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                                                     std::string link_name,
                                                     Eigen::Isometry3d& trans,
                                                     double tor)
  : MotionValidator(space_info), env_(std::move(env)), kin_(std::move(kin))
{
  // std::cout << "Pass-1" << std::endl;
  joints_ = kin_->getJointNames();
  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  // std::cout << "Pass0" << std::endl;
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  links_ = adj_map.getActiveLinkNames();
  // std::cout << "Pass1" << std::endl;
  this->joint_names_ = kin_->getJointNames();
  // std::cout << "Pass2" << std::endl;
  contact_manager_ = env_->getContinuousContactManager();
  // std::cout << "Pass3" << std::endl;
  contact_manager_->setActiveCollisionObjects(links_);
  // std::cout  <<"Pass4" << std::endl;
  contact_manager_->setContactDistanceThreshold(0);

  setConstraintsStd(link_name, trans, tor);
    // std::cout << "Pass5" << std::endl;
  populateAdjacent();

  // populateInclusion(kin->getActiveLinkNames());
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::pair<ompl::base::State*, double> dummy = { nullptr, 0.0 };
  return checkMotion(s1, s2, dummy);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1,
                                            const ompl::base::State* s2,
                                            std::pair<ompl::base::State*, double>& lastValid) const
{
  const ompl::base::StateSpace& state_space = *si_->getStateSpace();

  unsigned n_steps = state_space.validSegmentCount(s1, s2);

  // std::cout << "num of segments " << n_steps << std::endl;

  ompl::base::State* start_interp = si_->allocState();
  ompl::base::State* end_interp = si_->allocState();

  bool is_valid = true;
  unsigned i = 1;
  for (i = 1; i <= n_steps; ++i)
  {
    state_space.interpolate(s1, s2, static_cast<double>(i - 1) / n_steps, start_interp);
    state_space.interpolate(s1, s2, static_cast<double>(i) / n_steps, end_interp);

    if (!continuousCollisionCheck(start_interp, end_interp))
    {
      // std::cout << "collision failed" << std::endl;
      is_valid = false;
      break;
    }

    const ompl::base::RealVectorStateSpace::StateType* s =
        start_interp->as<ompl::base::RealVectorStateSpace::StateType>();
    if (!checkConstraints(start_interp, end_interp))
    {
      is_valid = false;
      break;
    }
  }

  if (!is_valid)
  {
    lastValid.second = static_cast<double>(i - 1) / n_steps;
    if (lastValid.first != nullptr)
      state_space.interpolate(s1, s2, lastValid.second, lastValid.first);
  }

  si_->freeState(start_interp);
  si_->freeState(end_interp);
  return is_valid;
}

void ContinuousMotionValidator::populateAdjacent()
{
  for (int it = 0; it < this->links_.size() - 1; it++)
  {
    this->adj_map[this->links_[it]] = this->links_[it + 1];
  }
}

// if set, will only check these links
void ContinuousMotionValidator::populateInclusion(std::vector<std::string> link_names)
{
  this->inclusive_names_ = link_names;
}

bool ContinuousMotionValidator::filterInclusion(std::string link_name1, std::string link_name2) const
{
  std::vector<std::string> am = getInclusion();

  if (am.size() == 0)
    return true;

  for (std::string name : am)
  {
    if (name.compare(link_name1) == 0 || name.compare(link_name2) == 0)
      return true;
  }

  return false;
}

bool ContinuousMotionValidator::filterAdjacent(std::string link_name1, std::string link_name2) const
{
  // std::vector<std::string>::iterator it = find (this->links_.begin(), this->links_.end(), link_name1)
  // if (it != this->links_.end()){

  // }

  std::map<std::string, std::string> am = getAdjacent();
  if (am.find(link_name1) != am.end())
  {
    if (am[link_name1].compare(link_name2) == 0)
    {
      return true;
    }
  }

  return false;
}

bool ContinuousMotionValidator::continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  const ompl::base::RealVectorStateSpace::StateType* start = s1->as<ompl::base::RealVectorStateSpace::StateType>();
  const ompl::base::RealVectorStateSpace::StateType* finish = s2->as<ompl::base::RealVectorStateSpace::StateType>();

  // Need to get thread id
  tesseract_collision::ContinuousContactManager::Ptr cm = contact_manager_->clone();

  const auto dof = si_->getStateDimension();
  Eigen::Map<Eigen::VectorXd> start_joints(start->values, dof);
  Eigen::Map<Eigen::VectorXd> finish_joints(finish->values, dof);

  tesseract_environment::EnvState::Ptr state0 = env_->getState(joints_, start_joints);
  tesseract_environment::EnvState::Ptr state1 = env_->getState(joints_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  // if (contact_map.size() == 1)
  // {
  //   if (contact_map.begin()->first.first.compare("ur_arm_forearm_link") == 0 &&
  //       contact_map.begin()->first.second.compare("ur_arm_wrist_3_link") == 0)
  //   {
  //     std::cout << " This is the bs" << std::endl;
  //   }
  // }

  auto it = contact_map.begin();

  // filter adjacent joints
  while (it != contact_map.end())
  {
    // std::cout << it->first.first.c_str() << " " <<  it->first.second.c_str() << std::endl;
    if (this->filterAdjacent(it->first.first, it->first.second))
    {
      // supported in C++11
      it = contact_map.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // it = contact_map.begin();
  // // filter non-inclusive links
  // while (it != contact_map.end())
  // {
  //   if (this->filterInclusion(it->first.first, it->first.second))
  //   {
  //     ++it;
  //   }
  //   else
  //   {
  //     // supported in C++11
  //     it = contact_map.erase(it);
  //   }
  // }

  return contact_map.empty();
}

void ContinuousMotionValidator::setConstraints(std::string link_name, const ompl::base::State* state, double tor)
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = this->joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(this->joint_names_, joint_angles);

  this->constraint_name_ = link_name;
  this->goal_transform_ = env_state->transforms.find(link_name)->second;
  this->tor_ = tor;
}

void ContinuousMotionValidator::setConstraintsStd(std::string link_name, Eigen::Isometry3d trans, double tor)
{
  // const auto dof = this->joint_names_.size();

  // Eigen::Map<Eigen::VectorXd> joint_angles(state.data(), long(dof));

  //   std::cout << "joints are " << std::endl;
  // std::cout << this->joint_names_[0].c_str() << std::endl;
  // std::cout << this->joint_names_[1].c_str() << std::endl;
  // std::cout << this->joint_names_[2].c_str() << std::endl;
  // std::cout << this->joint_names_[3].c_str() << std::endl;
  // std::cout << this->joint_names_[4].c_str() << std::endl;

  // tesseract_environment::EnvState::ConstPtr env_state = env_->getState(this->joint_names_, joint_angles);

  this->constraint_name_ = link_name;
  // this->goal_transform_ = env_state->transforms.find(link_name)->second;
  this->goal_transform_ = trans;
  this->tor_ = tor;
}

bool ContinuousMotionValidator::checkConstraints(const ompl::base::State* state1, const ompl::base::State* state2) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state2->as<ompl::base::RealVectorStateSpace::StateType>();
  const int dof = this->joint_names_.size();

  // std::cout << "num of joint is" << std::endl;
  // std::cout << dof << std::endl;

  // const auto dof = si_->getStateDimension();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(this->joint_names_, joint_angles);

  if (env_state->transforms.find(this->constraint_name_) != env_state->transforms.end())
  {
    Eigen::Isometry3d temp = env_state->transforms.find(this->constraint_name_)->second;

    // std::cout << "current transform is" << std::endl;
    // std::cout << temp.matrix() << std::endl;

    // std::cout << "goal transform is" << std::endl;
    // std::cout << this->goal_transform_.matrix() << std::endl;

    // const ompl::base::RealVectorStateSpace::StateType* s2 =
    // state2->as<ompl::base::RealVectorStateSpace::StateType>();

    // Eigen::Map<Eigen::VectorXd> joint_angles2(s2->values, long(dof));

    // std::cout << "states are " << std::endl;
    // std::cout << joint_angles << std::endl;
    // std::cout << joint_angles2 << std::endl;

    //     if (i == 2)
    // {
    // const auto dof = si_->getStateDimension();
    // const ompl::base::RealVectorStateSpace::StateType* ss1 =
    // state1->as<ompl::base::RealVectorStateSpace::StateType>(); const ompl::base::RealVectorStateSpace::StateType* ss2
    // = state2->as<ompl::base::RealVectorStateSpace::StateType>(); Eigen::Map<Eigen::VectorXd>
    // joint_angles(ss1->values, long(dof)); Eigen::Map<Eigen::VectorXd> joint_angles2(ss2->values, long(dof));

    // std::cout << "states are " << std::endl;
    // std::cout << joint_angles << std::endl;

    // std::cout << "joints are " << std::endl;
    // std::cout << this->joint_names_[0].c_str() << std::endl;
    // std::cout << joint_angles2 << std::endl;
    // }

    // Eigen::Vector3d diff = temp.translation() - this->goal_transform_.translation();
    // if (diff.norm() > this->tor_)
    // {
    //   // std::cout << "failed";
    //   // abort();
    //   return false;
    // }

    Eigen::Isometry3d diff_trans = this->goal_transform_ * temp.inverse();
    Eigen::Vector3d lin = diff_trans.translation();
    Eigen::Vector3d rot = diff_trans.rotation().eulerAngles(0, 1, 2);
    // if (sqrt(pow(lin.norm(),2)+pow(rot.norm(),2)) > this->tor_)
    // return false;
    if (lin.norm() > this->tor_ || rot.norm() > this->tor_ )
      return false;

    std::cout << " found a state within constraint" << std::endl;
  }

  return true;
}

}  // namespace tesseract_motion_planners
