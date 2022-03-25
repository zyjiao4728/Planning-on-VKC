#ifndef TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H
#define TESSERACT_MOTION_PLANNERS_CHAIN_OMPL_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

namespace tesseract_motion_planners
{
struct OmplPlanParameters
{
  double planning_time = 5.0;
  bool simplify = true;
  int n_steps = 10;
};

class ChainOmplInterface
{
public:
  using Ptr = std::shared_ptr<ChainOmplInterface>;
  using ConstPtr = std::shared_ptr<const ChainOmplInterface>;

  ChainOmplInterface(tesseract_environment::Environment::ConstPtr env,
                     tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  /**
   * @brief Overwrite joint limits
   * @param joint_limit 2d vector, 0th row is lower
   */
  void resetKinematicsLimit(Eigen::MatrixX2d joint_limit);


  /**
   * @brief Solve the planning problem, output is empty if it fails
   */

  boost::optional<ompl::geometric::PathGeometric> plan(ompl::base::PlannerPtr planner,
                                                       const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  boost::optional<ompl::geometric::PathGeometric> plan(const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  boost::optional<ompl::geometric::PathGeometric> plan(const Eigen::VectorXd & from,
                                                       const Eigen::VectorXd & to,
                                                       const OmplPlanParameters& params);

  ompl::base::SpaceInformationPtr spaceInformation();

  Eigen::MatrixXd toEigenArray(const ompl::geometric::PathGeometric& path);

  void useL2NormWeightedInterpolation(bool option);

  void setMotionValidator(ompl::base::MotionValidatorPtr mv)
  {
    ss_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }

  tesseract_kinematics::ForwardKinematics::ConstPtr GetKin();

  void setAdjacencyMap();

  void populateAdjacent();

  void populateInclusion(std::vector<std::string> link_names);

  bool filterAdjacent(std::string link_name1, std::string link_name2) const;

  bool filterInclusion(std::string link_name1, std::string link_name2) const;
  std::map<std::string, std::string> getAdjacent() const { return this->adj_map; };
  std::vector<std::string> getInclusion() const { return this->inclusive_names_; };

  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const;

  ompl::geometric::SimpleSetupPtr ss_;

  // bool checkResultSatisfy(ompl::geometric::PathGeometric& path, const std::vector<double>& to);
  // bool satisfyVelocity(boost::optional<ompl::geometric::PathGeometric>& sol);
  // void setSamplerWeights(Eigen::VectorXd wts);

  // Eigen::MatrixXd L2NormWeightedInterpolation( ompl::geometric::PathGeometric & wpts , const OmplPlanParameters&
  // params); void setConstraints(std::string link_name, const ompl::base::State* state, double tor = 0.05); void
  // setConstraintsStd(std::string link_name, Eigen::Isometry3d& trans, double tor = 0.05); bool checkConstraints(const
  // ompl::base::State* state) const;

private:
  bool isStateValid(const ompl::base::State* state) const;
  bool isStateValidStd(std::vector<double> wp) const;

private:
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  Eigen::VectorXd weights;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::string> inclusive_names_;
  std::map<std::string, std::string> adj_map;

  ompl::base::StateSamplerAllocator state_sampler_allocator;

  // double tor_;
  // Eigen::Isometry3d goal_transform_;
  // bool use_l2_weighted_norm;
  // std::string constraint_name_;
};
}  // namespace tesseract_motion_planners

#endif
