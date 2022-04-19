#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <vkc_example/utils.h>

using namespace std;
using namespace tesseract_rosutils;
using namespace tesseract_planning;
using namespace trajopt;

void solveProb(PlannerRequest request, PlannerResponse &response, int n_iter) {
  // Set the optimization parameters (Most are being left as defaults)

  ROS_WARN("Constructed optimization problem. Starting optimization.");

  // Solve problem. Results are stored in the response
  TrajOptMotionPlanner planner;

  auto trajopt_status = planner.solve(request, response);
  return;
}

CostInfo solveProb_cost(TrajOptProb::Ptr prob_ptr, PlannerResponse &response,
                        int n_iter, bool enable_plotting) {
  // ProcessPlanningServer
  // planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
  // planning_server.loadDefaultProcessPlanners();

  // auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();

  // auto trajopt_composite_profile =
  // std::make_shared<TrajOptDefaultCompositeProfile>();
  // trajopt_composite_profile->collision_constraint_config.enabled = false;
  // trajopt_composite_profile->collision_cost_config.safety_margin = 0.005;
  // trajopt_composite_profile->collision_cost_config.coeff = 50;

  // // Set the optimization parameters (Most are being left as defaults)
  // auto trajopt_solver_profile =
  // std::make_shared<TrajOptDefaultSolverProfile>();
  // // TrajOptPlannerConfig config(prob_ptr);
  // trajopt_solver_profile->opt_info.max_iter = n_iter;
  // trajopt_solver_profile->opt_info.cnt_tolerance = 1e-2;
  // trajopt_solver_profile->opt_info.trust_expand_ratio = 1.5;
  // trajopt_solver_profile->opt_info.trust_shrink_ratio = 0.5;
  // trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
  // trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;

  // planning_server.getProfiles()->addProfile<TrajOptPlanProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN",
  //     trajopt_plan_profile);
  // planning_server.getProfiles()->addProfile<TrajOptCompositeProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
  //     trajopt_composite_profile);
  // planning_server.getProfiles()->addProfile<TrajOptSolverProfile>(
  //     profile_ns::TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT",
  //     trajopt_solver_profile);

  // // Create the planner and the responses that will store the results
  // TrajOptMotionPlanner planner;

  // // Set Planner Configuration
  // // planner.c(config);

  // // ROS_WARN("Constructed optimization problem with cost constraint.
  // Starting optimization.");
  // // // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // // CostInfo cost;
  // // // Solve problem. Results are stored in the response
  // // planner.solve(response, cost.cost_vals, cost.cnt_viols,
  // cost.total_cost);

  // // if(enable_plotting)
  // // {
  // //   ROS_WARN("[%s]plotting is enabled, plot callback is configured.",
  // __func__);
  // //   tesseract_rosutils::ROSPlottingPtr plotter =
  // // std::make_shared<tesseract_rosutils::ROSPlotting>(prob_ptr->GetEnv());

  // //   config.callbacks.push_back(PlotCallback(*prob_ptr, plotter));
  // // }
  CostInfo cost;

  return cost;
}

void refineTrajectory(tesseract_common::TrajArray &traj) {
  long int n_rows = traj.rows();
  long int n_cols = traj.cols();

  for (int i = 0; i < n_rows - 1; i++) {
    double delta_y = traj(i + 1, 1) - traj(i, 1);
    double delta_x = traj(i + 1, 0) - traj(i, 0);
    double delta_orientation = atan2(delta_y, delta_x);

    if (delta_orientation > 3.14159265359) delta_orientation -= 3.14159265359;

    if (delta_orientation < -3.14159265359) delta_orientation += 3.14159265359;

    if (n_cols > 3) {
      traj(i, 3) += traj(i, 2) - delta_orientation;
    }
    traj(i, 2) = delta_orientation;
  }

  // the last orientation is as same as the 2nd last
  if (n_rows > 1) {
    if (n_cols > 3) {
      traj(n_rows - 1, 3) += traj(n_rows - 1, 2) - traj(n_rows - 2, 2);
    }
    traj(n_rows - 1, 2) = traj(n_rows - 2, 2);
  }
}

int saveTrajToFile(const tesseract_common::TrajArray &traj,
                   const std::string filename) {
  ofstream fileout;
  fileout.open(filename, std::ios_base::app);

  if (!fileout.is_open()) {
    std::cout << "Cannot open file: " << filename << std::endl;
    return -1;
  }

  long n_rows = traj.rows();
  long n_cols = traj.cols();

  for (size_t i = 0; i < n_rows; i++) {
    for (size_t j = 0; j < n_cols; j++) {
      fileout << traj(i, j) << ",\n"[j == n_cols - 1];
    }
  }

  fileout.close();
  std::cout << "Trajectory file save at: " << filename << std::endl;

  return 0;
}

std::vector<vkc::JointDesiredPose> getJointHome(
    std::unordered_map<std::string, double> home_pose) {
  std::vector<vkc::JointDesiredPose> joint_home;
  for (auto &pose : home_pose) {
    if (pose.first.substr(0, pose.first.find("_", 0)) == "base") continue;
    joint_home.push_back(vkc::JointDesiredPose(pose.first, pose.second));
  }
  return joint_home;
}

void TrajectoryVisualize(
    vkc::VKCEnvBasic &env, vkc::ActionSeq &actions,
    vector<tesseract_common::JointTrajectory> &joint_trajs) {
  ROS_INFO("[%s]actions size: %ld, traj size: %ld", __func__, actions.size(),
           joint_trajs.size());

  // reset rviz
  ROS_INFO("[%s]please reset rviz and presss ENTER to go on...", __func__);
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  env.reInit();

  long int max_traj_len{0};
  for (auto &traj : joint_trajs) {
    max_traj_len = traj.size() > max_traj_len ? traj.size() : max_traj_len;
    // std::cout << ">> traj details: " << std::endl
    //           << traj << std::endl;
  }

  // plot current `action` result
  auto action_iter = actions.begin();
  auto joint_traj_iter = joint_trajs.begin();

  // plot current `action` trajectory data
  for (; joint_traj_iter != joint_trajs.end();
       ++action_iter, ++joint_traj_iter) {
    // ROS_INFO("joints names number: %d, joint number: %d, joint states number:
    // %d, revision: %d",
    //          joint_traj_iter->joint_names.size(),
    //          joint_traj_iter->trajectory.cols(),
    //          joint_traj_iter->trajectory.rows(),
    //          env.getPlotVKCEnv()->getTesseract()->getRevision());
    // tesseract_common::TrajArray traj = joint_traj_iter->
    // ->trajectory.leftCols(static_cast<long>(joint_traj_iter->joint_names.size()));
    // std::cout << "Traj:" << std::endl
    //           << traj << std::endl;

    // ROSPlottingPtr plotter =
    // std::make_shared<ROSPlotting>(env.getPlotVKCEnv()->getTesseract()->getEnvironment());
    // plotter->plotTrajectory(joint_traj_iter->joint_names, traj);
    // usleep((useconds_t)(joint_traj_iter->trajectory.size() * 1500000.0 /
    // max_traj_len));

    // // update env according to the action
    // // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    // env.updatePlotEnv(joint_traj_iter->joint_names,
    // joint_traj_iter->trajectory.bottomRows(1).transpose(), *action_iter);
    // plotter->clear();
  }
}
