#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "tesseract_motion_planners/ompl/chain_ompl_interface.h"

namespace tesseract_motion_planners
{
ChainOmplInterface::ChainOmplInterface(tesseract_environment::Environment::ConstPtr env,
                                       tesseract_kinematics::ForwardKinematics::ConstPtr kin)
  : env_(std::move(env)), kin_(std::move(kin))
{
  assert(nullptr != env_);
  assert(nullptr != kin_);
  CONSOLE_BRIDGE_logInform("[%s]precondition check passed", __func__);
  joint_names_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  CONSOLE_BRIDGE_logInform("[%s]create adjacency map", __func__);
  tesseract_environment::AdjacencyMap adj_map(
      env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  link_names_ = adj_map.getActiveLinkNames();

  populateInclusion(kin_->getActiveLinkNames());
  populateAdjacent();
  const auto dof = kin_->numJoints();
  const auto& limits = kin_->getLimits();

  // Construct the OMPL state space for this manipulator
  CONSOLE_BRIDGE_logInform("[%s]create state space instance", __func__);
  ompl::base::RealVectorStateSpace* space = new ompl::base::RealVectorStateSpace();
  for (unsigned i = 0; i < dof; ++i)
  {
    space->addDimension(joint_names_[i], limits(i, 0), limits(i, 1));
  }

  // weighted sampling does not affect chain manipulators much
  this->weights = Eigen::VectorXd::Ones(kin_->numJoints());

  // The state sampler allocator.
  CONSOLE_BRIDGE_logInform("[%s]create state sampler allocator", __func__);
  this->state_sampler_allocator =
      std::bind(&ChainOmplInterface::allocWeightedRealVectorStateSampler, this, std::placeholders::_1);

  if (this->state_sampler_allocator)
  {
    space->setStateSamplerAllocator(this->state_sampler_allocator);
  }

  ompl::base::StateSpacePtr state_space_ptr(space);

  // IMPORTANT
  // these 2 parameters decide how fine the collision checking is, make them larger if the end result trajectory is
  // failing
  CONSOLE_BRIDGE_logInform("[%s]set collision configurations", __func__);
  state_space_ptr->setValidSegmentCountFactor(5);
  state_space_ptr->setLongestValidSegmentFraction(0.1);

  ss_.reset(new ompl::geometric::SimpleSetup(state_space_ptr));

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(link_names_);
  contact_manager_->setContactDistanceThreshold(0);

  // Setup state checking functionality
  ss_->setStateValidityChecker(std::bind(&ChainOmplInterface::isStateValid, this, std::placeholders::_1));

  auto cmv = std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(spaceInformation(), env_, kin_);

  setMotionValidator(cmv);

  CONSOLE_BRIDGE_logInform("[%s]Chain OMPL Interface finished", __func__);
}

void ChainOmplInterface::resetKinematicsLimit(Eigen::MatrixX2d joint_limit)
{
  const auto dof = kin_->numJoints();

  ompl::base::RealVectorStateSpace* space = new ompl::base::RealVectorStateSpace();

  for (unsigned i = 0; i < dof; ++i)
  {
    space->addDimension(joint_names_[i], joint_limit(i, 0), joint_limit(i, 1));
  }

  this->weights = Eigen::VectorXd::Ones(kin_->numJoints());

  // The state sampler allocator.
  this->state_sampler_allocator =
      std::bind(&ChainOmplInterface::allocWeightedRealVectorStateSampler, this, std::placeholders::_1);

  if (this->state_sampler_allocator)
  {
    std::cout << "state sampler set to WGS" << std::endl;
    space->setStateSamplerAllocator(this->state_sampler_allocator);
  }

  ompl::base::StateSpacePtr state_space_ptr(space);

  state_space_ptr->setValidSegmentCountFactor(1);
  state_space_ptr->setLongestValidSegmentFraction(0.3);

  ss_.reset(new ompl::geometric::SimpleSetup(state_space_ptr));

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(link_names_);
  contact_manager_->setContactDistanceThreshold(0);

  auto cmv = std::make_shared<tesseract_motion_planners::ContinuousMotionValidator>(spaceInformation(), env_, kin_);

  setMotionValidator(cmv);

  std::cout << "Chain OMPL Interface reset completed " << std::endl;
}

ompl::base::StateSamplerPtr
ChainOmplInterface::allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const
{
  Eigen::MatrixX2d limits = kin_->getLimits();
  return std::make_shared<WeightedRealVectorStateSampler>(space, this->weights, limits);
}

void ChainOmplInterface::setAdjacencyMap()
{
  this->contact_manager_ = this->env_->getDiscreteContactManager();

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      this->env_->getSceneGraph(), this->kin_->getActiveLinkNames(), this->env_->getCurrentState()->transforms);

  this->contact_manager_->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  this->contact_manager_->setContactDistanceThreshold(0);

  populateAdjacent();
}

boost::optional<ompl::geometric::PathGeometric> ChainOmplInterface::plan(ompl::base::PlannerPtr planner,
                                                                         const std::vector<double>& from,
                                                                         const std::vector<double>& to,
                                                                         const OmplPlanParameters& params)
{
  return this->plan(from, to, params);
}

boost::optional<ompl::geometric::PathGeometric> ChainOmplInterface::plan(const Eigen::VectorXd& from,
                                                                         const Eigen::VectorXd& to,
                                                                         const OmplPlanParameters& params)
{
  std::vector<double> f(from.data(), from.data() + from.size());
  std::vector<double> t(to.data(), to.data() + to.size());
  return this->plan(f, t, params);
}
boost::optional<ompl::geometric::PathGeometric> ChainOmplInterface::plan(const std::vector<double>& from,
                                                                         const std::vector<double>& to,
                                                                         const OmplPlanParameters& params)
{
  const auto dof = ss_->getStateSpace()->getDimension();

  ompl::base::ScopedState<> start_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = from[i];

  ompl::base::ScopedState<> goal_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = to[i];

  ss_->setStartAndGoalStates(start_state, goal_state);

  std::cout << "Checking goal state" << std::endl;
  if (ss_->getStateSpace()->satisfiesBounds(goal_state->as<ompl::base::State>()))
  {
    std::cout << "Goal state is within bound " << std::endl;
  }
  else
  {
    std::cout << "Goal state is out of bound " << std::endl;
  }

  ompl::base::PlannerStatus status = ss_->solve(params.planning_time);


  if (status)
  {
    ompl::geometric::PathGeometric& path = ss_->getSolutionPath();
    if (params.simplify)
    {
      ss_->simplifySolution();
    }
    path = ss_->getSolutionPath();
    if (path.getStateCount() < params.n_steps)
    {
      path.interpolate(params.n_steps);
      // if (this->use_l2_weighted_norm)
      // {
      //   Eigen::MatrixXd eig_path = L2NormWeightedInterpolation(path, params);
      //   // path.clear();

      //   for (int sdx = 0; sdx < params.n_steps; sdx++)
      //   {
      //     for (int jdx = 0; jdx < eig_path.cols(); jdx++)
      //     {
      //       path.getState(sdx)->as<ompl::base::RealVectorStateSpace::StateType>()->values[jdx] = eig_path(sdx, jdx);
      //     }
      //   }
      // }
    }

    // ompl::geometric::PathGeometric& path = ss_->getSolutionPath();

    // if (checkResultSatisfy(path, to))
    // {
    return boost::optional<ompl::geometric::PathGeometric>{ path };
    // }
    // return {};
  }

  return {};
}

Eigen::MatrixXd ChainOmplInterface::toEigenArray(const ompl::geometric::PathGeometric& path)
{
  const long n_points = static_cast<long>(path.getStateCount());
  const long dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  Eigen::MatrixXd result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
  {
    const auto& state = path.getState(static_cast<unsigned>(i))->as<ompl::base::RealVectorStateSpace::StateType>();
    for (long j = 0; j < dof; ++j)
    {
      result(i, j) = state->values[j];
    }
  }
  return result;
}

tesseract_kinematics::ForwardKinematics::ConstPtr ChainOmplInterface::GetKin() { return kin_; }

ompl::base::SpaceInformationPtr ChainOmplInterface::spaceInformation() { return ss_->getSpaceInformation(); }

bool ChainOmplInterface::isStateValid(const ompl::base::State* state) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  tesseract_collision::DiscreteContactManager::Ptr cm = contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  auto it = contact_map.begin();

  while (it != contact_map.end())
  {
    if (filterAdjacent(it->first.first, it->first.second))
    {
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
  //     it = contact_map.erase(it);
  //   }
  // }

  if (contact_map.empty())
    return true;

  return false;
}

bool ChainOmplInterface::isStateValidStd(std::vector<double> wp) const
{
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(wp.data(), long(dof));
  std::cout << "checking states" << joint_angles << std::endl;

  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

  tesseract_collision::DiscreteContactManager::Ptr cm = contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

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

  while (it != contact_map.end())
  {
    std::cout << it->first.first << " " << it->first.second << std::endl;
    if (filterAdjacent(it->first.first, it->first.second))
    {
      it = contact_map.erase(it);
    }
    else
    {
      ++it;
    }
  }

  return contact_map.empty();
}

void ChainOmplInterface::populateAdjacent()
{
  for (size_t it = 1; it < this->link_names_.size(); it++)
  {
    this->adj_map[this->link_names_[it]] = this->link_names_[it];
  }
}

// if set, will not check these links
void ChainOmplInterface::populateInclusion(std::vector<std::string> link_names) { this->inclusive_names_ = link_names; }

bool ChainOmplInterface::filterInclusion(std::string link_name1, std::string link_name2) const
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

// void ChainOmplInterface::setConstraints(std::string link_name, const ompl::base::State* state, double tor)
// {
//   const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
//   const auto dof = joint_names_.size();

//   Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
//   tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

//   this->constraint_name_ = link_name;
//   this->goal_transform_ = env_state->transforms.find(link_name)->second;
//   this->tor_ = tor;
// }

// void ChainOmplInterface::setConstraintsStd(std::string link_name, Eigen::Isometry3d& trans, double tor)
// {
//   // const auto dof = joint_names_.size();

//   // Eigen::Map<Eigen::VectorXd> joint_angles(state.data(), long(dof));

//   // tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

//   this->constraint_name_ = link_name;
//   // this->goal_transform_ = env_state->transforms.find(link_name)->second;
//   this->goal_transform_ = trans;
//   this->tor_ = tor;
// }

// bool ChainOmplInterface::checkConstraints(const ompl::base::State* state) const
// {
//   const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
//   const auto dof = joint_names_.size();

//   Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
//   tesseract_environment::EnvState::ConstPtr env_state = env_->getState(joint_names_, joint_angles);

//   if (env_state->transforms.find(this->constraint_name_) != env_state->transforms.end())
//   {
//     Eigen::Isometry3d temp = env_state->transforms.find(this->constraint_name_)->second;

//     // std::cout << "current transform is" << std::endl;
//     // std::cout << temp.matrix() << std::endl;

//     // std::cout << "goal transform is" << std::endl;
//     // std::cout << this->goal_transform_.matrix() << std::endl;
//     Eigen::Isometry3d diff_trans = this->goal_transform_ * temp.inverse();
//     Eigen::Vector3d lin = diff_trans.translation();
//     Eigen::Vector3d rot = diff_trans.rotation().eulerAngles(0, 1, 2);
//     if (lin.norm() > this->tor_ || rot.norm() > this->tor_)
//       return false;
//   }

//   return true;
// }

// bool ChainOmplInterface::satisfyVelocity(boost::optional<ompl::geometric::PathGeometric>& sol)
// {
//   // this->kin_->

//   return false;
// }

// bool ChainOmplInterface::checkResultSatisfy(ompl::geometric::PathGeometric& path, const std::vector<double>& to)
// {
//   // static double tol = 0.005;
//   // const auto dof = this->ss_->getStateSpace()->getDimension();

//   // // const auto &state =
//   path.getState(static_cast<unsigned>(i))->as<ompl::base::RealVectorStateSpace::StateType>();
//   // auto ewp = path.getState(path.getStateCount() - 1)->as<ompl::base::RealVectorStateSpace::StateType>();

//   // for (unsigned i = 0; i < dof; ++i)
//   // {
//   //   if (abs(ewp->values[i] - to[i]) > tol)
//   //     return false;
//   // }

//   return true;
// }

bool ChainOmplInterface::filterAdjacent(std::string link_name1, std::string link_name2) const
{
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

// void ChainOmplInterface::useL2NormWeightedInterpolation(bool option) { this->use_l2_weighted_norm = option; }

// void ChainOmplInterface::setSamplerWeights(Eigen::VectorXd wts)
// {
//   // if (wts.size() != kin_->numJoints())
//   // {
//   //     std::cout << " Sampler weight size not correct" << std::endl;
//   // }
//   // else {
//   this->weights = wts;
//   // }

//   // if (this->weights.size() == 0)
//   // {
//   //   this->weights = Eigen::VectorXd::Ones(kin_->numJoints());
//   // }

//   return;
// }

// Eigen::MatrixXd ChainOmplInterface::L2NormWeightedInterpolation(ompl::geometric::PathGeometric& wpts,
//                                                                 const OmplPlanParameters& params)
// {
//   std::vector<ompl::base::State*> states = wpts.getStates();
//   std::vector<double> weights;
//   Eigen::MatrixXd path = toEigenArray(wpts);
//   double weight_sum = 0;
//   for (int idx = 0; idx < (states.size() - 1); idx++)
//   {
//     Eigen::VectorXd dis = path.row(idx) - path.row(idx + 1);
//     weights.push_back(dis.norm());
//     weight_sum += weights.back();
//   }

//   Eigen::MatrixXd res_path(params.n_steps, path.cols());
//   int current_step = 0;
//   for (int idx = 0; idx < (states.size() - 1); idx++)
//   {
//     int steps = round(weights[idx] / weight_sum * float(params.n_steps));
//     for (int jdx = 0; jdx < steps; jdx++)
//     {
//       for (int col_dx = 0; col_dx < path.cols(); col_dx++)
//       {
//         res_path(current_step, col_dx) =
//             (path(idx + 1, col_dx) - path(idx, col_dx)) / float(steps) * float(jdx) + path(idx, col_dx);
//       }
//       current_step++;
//     }
//   }
//   while (current_step < params.n_steps)
//   {
//     res_path.row(current_step) = path.row(path.rows() - 1);
//     current_step++;
//   }

//   return res_path;
// }

}  // namespace tesseract_motion_planners
