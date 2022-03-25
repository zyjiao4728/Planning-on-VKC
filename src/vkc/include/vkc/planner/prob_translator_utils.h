#ifndef VKC_PROB_TRANSLATOR_UTILS_H
#define VKC_PROB_TRANSLATOR_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <vkc/planner/prob_generator.h>
#include <vkc/planner/huskey_ik_define.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ProblemDefinition.h>

#include <vkc/planner/prob_ompl_define.h>
#include <vkc/planner/traj_init.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


// #include <tesseract_motion_planners/ompl/chain_ompl_interface.h>
#include <tesseract_motion_planners/default_planner_namespaces.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>

#include <iostream>
#include <string>
#include <vector>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;

namespace vkc
{
    tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric &path);

    std::vector<double> HuskeyIK(VKCEnvBasic& env, std::vector<LinkDesiredPose>& link_objectives,
                                  std::vector<JointDesiredPose>& joint_objectives, MapInfo map, int n_steps, Eigen::MatrixX2d & joint_limits);

    void solveOptProb(TrajOptProb::Ptr prob_ptr, PlannerResponse &response, int n_iter);

    // std::vector<double> BaseFirstIK(VKCEnvBasic& env, std::vector<LinkDesiredPose>& link_objectives,
    //                               std::vector<JointDesiredPose>& joint_objectives, MapInfo map,
    //                               trajopt::TrajArray& init_traj, int n_steps, Eigen::MatrixX2d & joint_limits);                                  

    void cleanState(std::vector<double> &state);
} // namespace vkc



#endif