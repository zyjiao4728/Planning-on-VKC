#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <vkc/planner/prob_translator_utils.h>
using namespace tesseract_planning;

namespace vkc {

tesseract_common::TrajArray toTrajArray(
    const ompl::geometric::PathGeometric &path) {
  const long n_points = static_cast<long>(path.getStateCount());
  const long dof =
      static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract_common::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i) {
    const auto &state = path.getState(static_cast<unsigned>(i))
                            ->as<ompl::base::RealVectorStateSpace::StateType>();
    for (long j = 0; j < dof; ++j) {
      result(i, j) = state->values[j];
    }
  }
  return result;
}

std::vector<double> HuskeyIK(VKCEnvBasic &env,
                             std::vector<LinkDesiredPose> &link_objectives,
                             std::vector<JointDesiredPose> &joint_objectives,
                             MapInfo map, int n_steps,
                             Eigen::MatrixX2d &joint_limits) {
  // trajopt::TrajArray init_bot = init_traj.bottomRows(1);
  // trajopt::TrajArray res_array = initTrajectory(env, link_objectives,
  //                                               joint_objectives, map,
  //                                               init_traj, n_steps);

  // Eigen::MatrixXd ewp = res_array.bottomRows(1);

  // std::vector<double> res(ewp.data(), ewp.data() + ewp.size());

  std::vector<double> res =
      initIK(env, link_objectives, joint_objectives, map, n_steps);

  cleanState(res);
  // if theres only a base objective
  if (link_objectives.size() == 1 &&
      link_objectives[0].link_name == "base_link" &&
      joint_objectives.size() == 0) {
    ROS_INFO("No Joint objective, only base objective");
    for (long idx = 3; idx < joint_limits.rows(); idx++)
      // res[idx] = start_pos[idx];

      if (idx == 6 || idx == 7) {
        res[idx] = (double)rand() / RAND_MAX * (joint_limits(idx, 1)) -
                   0.5 * joint_limits(idx, 1);
      } else if (idx == 4) {
        res[idx] = (double)rand() / RAND_MAX * (-joint_limits(idx, 1));
      } else {
        res[idx] = (double)rand() / RAND_MAX * (2 * joint_limits(idx, 1)) -
                   joint_limits(idx, 1);
      }
  }

  return res;
}

// void solveOptProb(TrajOptProb::Ptr prob_ptr, PlannerResponse &response, int
// n_iter)
// {
//     // Set the optimization parameters (Most are being left as defaults)

//     auto trajopt_solver_profile =
//     std::make_shared<TrajOptDefaultSolverProfile>();
//     trajopt_solver_profile->opt_info.max_iter = n_iter;
//     trajopt_solver_profile->opt_info.cnt_tolerance = 1e-3;
//     trajopt_solver_profile->opt_info.trust_expand_ratio = 1.5;
//     trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.5;
//     trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
//     trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;

//     ROS_WARN("Using TrajOpt to solve for IK");

//     // Solve problem. Results are stored in the response
//     planner.solve(response);

//     return;
// }

void cleanState(std::vector<double> &state) {
  for (size_t idx = 2; idx < 9; idx++) {
    if (state[idx] > M_PI) {
      state[idx] -= 2.0 * M_PI;
    } else if (state[idx] < -M_PI) {
      state[idx] += 2.0 * M_PI;
    }
  }
}
}  // namespace vkc
