#include <tesseract_motion_planners/3mo/3mo_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
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

  auto trajopt_status = planner.solve(request, response, true);

  ROS_WARN("%d, %s", trajopt_status.value(), trajopt_status.message().c_str());

  return;
}

void solveOmplProb(PlannerRequest request, PlannerResponse &response,
                   int n_iter) {
  ROS_WARN("constructed ompl problem, solving...");

  OMPLMotionPlanner planner;
  tesseract_planning::MMMOMotionPlanner ik_planner;
  tesseract_common::StatusCode planning_status;
  if (request.name == "3MO_IK_TRAJ") {
    planning_status = ik_planner.solve(request, response, true);
  } else {
    planning_status = planner.solve(request, response, true);
  }

  CONSOLE_BRIDGE_logWarn("%d, %s", planning_status.value(),
                         planning_status.message().c_str());
  return;
}


void refineTrajectory(tesseract_common::JointTrajectory &traj,
                      vkc::VKCEnvBasic &env) {
  std::vector<std::string> joint_names =
      env.getVKCEnv()->getTesseract()->getActiveJointNames();
  Eigen::VectorXd initial_state =
      env.getVKCEnv()->getTesseract()->getCurrentJointValues();
  std::vector<std::string> planned_joint_names = traj.states[0].joint_names;

  long int n_rows = traj.states.size();  // number of waypoints
  long int n_cols = joint_names.size();  // number of joints

  for (int i = 0; i < n_rows; i++) {
    Eigen::VectorXd current_waypoint = traj.states[i].position;

    for (int j = 0; j < planned_joint_names.size(); j++) {
      auto it =
          find(joint_names.begin(), joint_names.end(), planned_joint_names[j]);
      if (it != joint_names.end()) {
        int index = it - joint_names.begin();
        initial_state[index] = current_waypoint[j];
      } else {
        continue;
      }
    }
    traj.states[i].joint_names = joint_names;
    traj.states[i].position = initial_state;
  }

  double prev_orientation = 0.0;

  for (int i = 0; i < n_rows - 1; i++) {
    double delta_y =
        traj.states[i + 1].position[1] - traj.states[i].position[1];
    double delta_x =
        traj.states[i + 1].position[0] - traj.states[i].position[0];
    double delta_orientation = atan2(delta_y, delta_x);

    if (i == 0) {
      prev_orientation = delta_orientation;
    }

    if (abs(delta_orientation - prev_orientation) > M_PI) {
      if (delta_orientation > prev_orientation) {
        delta_orientation -= 2.0 * M_PI;
      } else {
        delta_orientation += 2.0 * M_PI;
      }
    }
    prev_orientation = delta_orientation;

    if (n_cols > 3) {
      traj.states[i].position[3] +=
          traj.states[i].position[2] - delta_orientation;
    }
    traj.states[i].position[2] = delta_orientation;
  }

  // std::cout << traj.states[0].position  << std::endl;

  // the last orientation is as same as the 2nd last
  if (n_rows > 1) {
    if (n_cols > 3) {
      traj.states[n_rows - 1].position[3] +=
          traj.states[n_rows - 1].position[2] -
          traj.states[n_rows - 2].position[2];
    }
    traj.states[n_rows - 1].position[2] = traj.states[n_rows - 2].position[2];
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

void setBaseJoint(vkc::ActionBase::Ptr action) {
  action->setBaseJoint(std::string("base_y_base_x"),
                       std::string("base_theta_base_y"));
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

void ik2csv(const std::vector<std::string> &joint_names,
            const std::vector<Eigen::VectorXd> &joint_states,
            const std::string &file_path) {
  std::ofstream myfile;
  myfile.open(file_path);

  static const Eigen::IOFormat format(
      Eigen::StreamPrecision, Eigen::DontAlignCols, "", std::string(",", 1));
  for (std::size_t i = 0; i < joint_names.size() - 1; ++i)
    myfile << joint_names[i] << ",";

  myfile << joint_names.back() << std::endl;

  for (const auto &ik : joint_states) {
    myfile << ik.format(format) << std::endl;
  }
  myfile.close();
}

void csv2ik(std::vector<std::string> &joint_names,
            std::vector<Eigen::VectorXd> &joint_states,
            const std::string &file_path) {
  std::ifstream csv_file(file_path);
  std::string line;
  bool is_header = true;

  while (std::getline(csv_file, line)) {
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(","), boost::token_compress_on);
    if (is_header) {
      is_header = false;
      for (const auto &t : tokens) {
        joint_names.push_back(t);
      }
      continue;
    }
    if (!tesseract_common::isNumeric(tokens[0]))
      throw std::runtime_error("loadTrajectorySeed: Invalid format");
    std::vector<double> state_vector;
    for (const auto &t : tokens) {
      double value = 0;
      tesseract_common::toNumeric<double>(t, value);
      state_vector.push_back(value);
    }
    Eigen::VectorXd joint_state = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        state_vector.data(), state_vector.size());
    joint_states.push_back(joint_state);
  }
}

std::string getCurrentTime(std::string time_format) {
  auto current_time =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  auto t = std::localtime(&current_time);
  char buf[80];
  std::strftime(buf, sizeof(buf), time_format.data(), t);
  return buf;
}