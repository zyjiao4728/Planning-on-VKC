#ifndef TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H
#define TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/MotionValidator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
class ContinuousMotionValidator : public ompl::base::MotionValidator
{
public:
  ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                            tesseract_environment::Environment::ConstPtr env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                            tesseract_environment::Environment::ConstPtr env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                            std::string link_name, Eigen::Isometry3d & trans, double tor = 0.05);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;


    // if set, will only check these links
    void populateInclusion(std::vector<std::string> link_names);

    void populateAdjacent();

    std::map<std::string, std::string> getAdjacent() const { return this->adj_map; };

    std::vector<std::string> getInclusion() const { return this->inclusive_names_; };

    bool filterAdjacent(std::string link_name1, std::string link_name2) const;

    bool filterInclusion(std::string link_name1, std::string link_name2) const;

    void setConstraints(std::string link_name, const ompl::base::State *state, double tor = 0.05);

    void setConstraintsStd(std::string link_name, Eigen::Isometry3d trans, double tor);

    bool checkConstraints(const ompl::base::State *state1, const ompl::base::State *state2) const;

  private:

    bool continuousCollisionCheck(const ompl::base::State *s1, const ompl::base::State *s2) const;

    tesseract_environment::Environment::ConstPtr env_;
    tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
    tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
    std::vector<std::string> links_;
    std::vector<std::string> joints_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<std::string> inclusive_names_;
    std::map<std::string, std::string> adj_map;

    std::string constraint_name_;
    Eigen::Isometry3d goal_transform_;
    double tor_;
};
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H
