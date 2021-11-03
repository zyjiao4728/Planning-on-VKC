#include <vkc_example/utils.h>

using namespace std;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
using namespace trajopt;

void solveProb(TrajOptProb::Ptr prob_ptr, PlannerResponse &response, int n_iter)
{
  // Set the optimization parameters (Most are being left as defaults)
  TrajOptPlannerConfig config(prob_ptr);
  config.params.max_iter = n_iter;
  config.params.cnt_tolerance = 1e-3;
  config.params.trust_expand_ratio = 1.2;
  config.params.trust_shrink_ratio = 0.8;
  config.params.min_trust_box_size = 1e-3;
  config.params.min_approx_improve = 1e-3;

  // Create the planner and the responses that will store the results
  TrajOptMotionPlanner planner;

  // Set Planner Configuration
  planner.setConfiguration(config);

  ROS_WARN("Constructed optimization problem. Starting optimization.");
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  // Solve problem. Results are stored in the response
  planner.solve(response);
}

CostInfo solveProb_cost(TrajOptProb::Ptr prob_ptr, PlannerResponse &response, int n_iter, bool enable_plotting)
{
  // Set the optimization parameters (Most are being left as defaults)
  TrajOptPlannerConfig config(prob_ptr);
  config.params.max_iter = n_iter;
  config.params.cnt_tolerance = 1e-2;
  config.params.trust_expand_ratio = 1.5;
  config.params.trust_shrink_ratio = 0.5;
  config.params.min_trust_box_size = 1e-3;
  config.params.min_approx_improve = 1e-3;

  if(enable_plotting)
  {
    ROS_WARN("[%s]plotting is enabled, plot callback is configured.", __func__);
    tesseract_rosutils::ROSPlottingPtr plotter =
        std::make_shared<tesseract_rosutils::ROSPlotting>(prob_ptr->GetEnv());

    config.callbacks.push_back(PlotCallback(*prob_ptr, plotter));
  }

  // Create the planner and the responses that will store the results
  TrajOptMotionPlanner planner;

  // Set Planner Configuration
  planner.setConfiguration(config);

  ROS_WARN("Constructed optimization problem with cost constraint. Starting optimization.");
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  CostInfo cost;
  // Solve problem. Results are stored in the response
  planner.solve(response, cost.cost_vals, cost.cnt_viols, cost.total_cost);

  return cost;
}


void refineTrajectory(tesseract_common::TrajArray &traj)
{
  long int n_rows = traj.rows();
  long int n_cols = traj.cols();

  for (int i = 0; i < n_rows - 1; i++)
  {
    double delta_y = traj(i + 1, 1) - traj(i, 1);
    double delta_x = traj(i + 1, 0) - traj(i, 0);
    double delta_orientation = atan2(delta_y, delta_x);

    if (delta_orientation > 3.14159265359)
      delta_orientation -= 3.14159265359;

    if (n_cols > 3)
    {
      traj(i, 3) += traj(i, 2) - delta_orientation;
    }
    traj(i, 2) = delta_orientation;
  }

  // the last orientation is as same as the 2nd last
  if (n_rows > 1)
  {
    if (n_cols > 3)
    {
      traj(n_rows - 1, 3) += traj(n_rows - 1, 2) - traj(n_rows - 2, 2);
    }
    traj(n_rows - 1, 2) = traj(n_rows - 2, 2);
  }
}

int saveTrajToFile(const tesseract_common::TrajArray &traj, const std::string filename)
{
  ofstream fileout;
  fileout.open(filename, std::ios_base::app);
  
  if( !fileout.is_open() )
  {
    std::cout << "Cannot open file: " << filename << std::endl;
    return -1;
  }

  size_t n_rows = traj.rows();
  size_t n_cols = traj.cols();

  for(size_t i = 0; i < n_rows; i++){
    for(size_t j = 0; j < n_cols; j++){
      fileout << traj(i, j) << ",\n"[j == n_cols - 1];
    }
  }

  fileout.close();
  std::cout << "Trajectory file save at: " << filename << std::endl;

  return 0;
}


std::vector<vkc::JointDesiredPose> getJointHome(std::unordered_map<std::string, double> home_pose)
{
  std::vector<vkc::JointDesiredPose> joint_home;
  for (auto &pose : home_pose)
  {
    if (pose.first.substr(0, pose.first.find("_", 0)) == "base")
      continue;
    joint_home.push_back(vkc::JointDesiredPose(pose.first, pose.second));
  }
  return joint_home;
}
