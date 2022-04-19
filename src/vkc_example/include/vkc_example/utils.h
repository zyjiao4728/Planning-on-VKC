#ifndef VKC_EXAMPLE_UTILS_H
#define VKC_EXAMPLE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>

#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <vkc/action/actions.h>
#include <vkc/env/vkc_env_basic.h>

#include <fstream>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <vector>

class CostInfo {
 public:
  std::vector<double> cost_vals;
  std::vector<double> cnt_viols;
  double total_cost;

 public:
  double getSmoothnessCost() {
    double ret = 0;

    for (unsigned int i = 0; i < cost_vals.size(); i++) {
      ret += cost_vals[i];
    }
    return ret;
  }

  double getCollisionCost() {
    double ret = 0;

    for (unsigned int i = 1; i < cnt_viols.size(); i++) {
      ret += cnt_viols[i];
    }

    return ret;
  }

  double getTotalCost() { return total_cost; }

  double getGoalCost() { return cnt_viols[0]; }
};

void solveProb(tesseract_planning::PlannerRequest request,
               tesseract_planning::PlannerResponse &response, int n_iter);

// CostInfo solveProb_cost(trajopt::TrajOptProb::Ptr prob_ptr,
// tesseract_planning::PlannerResponse &response,
//                         int n_iter, bool enable_ploting = false);

void refineTrajectory(tesseract_common::TrajArray &traj);

int saveTrajToFile(const tesseract_common::TrajArray &traj,
                   const std::string filename);

std::vector<vkc::JointDesiredPose> getJointHome(
    std::unordered_map<std::string, double> home_pose);

void TrajectoryVisualize(
    vkc::VKCEnvBasic &env, vkc::ActionSeq &actions,
    std::vector<tesseract_common::JointTrajectory> &joint_trajs);

#endif